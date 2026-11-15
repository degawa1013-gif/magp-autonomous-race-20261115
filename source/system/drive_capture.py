"""Bounded, asynchronous white-line diagnostics; never writes from the control loop.

Call submit(BGR_frame, binary_mask, metadata) after publishing the real actuator
sample. ``metadata['vehicle_control']['mode']`` takes precedence over ``mode``.
AUTO and STEER_AUTO are recorded automatically. update(mode) also ends a clip
when no camera frame arrives. JPEG originals, lossless PNG masks and aligned
JSONL/CSV metadata are written directly into downloadable ZIP segments.
"""
from __future__ import annotations

import copy
import json
import math
import re
import threading
import time
import zipfile
import uuid
import subprocess
import os
import sys
from collections import deque
from pathlib import Path
from typing import Any



_AUTO_MODES = frozenset(("AUTO", "STEER_AUTO"))
_PREFIX = "magp-capture-"
_README = """MAGP 白線追従・操舵 診断記録

images/000001.jpg : 入力BGR画像をJPEG保存（検出描画を加えない元画像）
masks/000001.png  : 同じフレームのカメラ白抽出マスク。PNGなので画素値を保持。
extra_masks/bird/000001.png : 同フレームを鳥瞰変換したマスク（任意）。
extra_masks/gated/000001.png : 同フレームの追跡範囲へ制限したマスク（任意）。
frames.jsonl     : 1行1フレーム。画像・マスク・時刻・検出・実際の出力指令を対応。
frames.csv       : メタデータを平坦化した表。空欄は未取得。
events.jsonl     : 「気になる」マーカーと対応フレーム・画像への参照。
summary.json     : フレーム数、欠落、処理時間の分位数、終了理由。
analysis.json    : 機械解析用の知覚状態比率・失検出区間・遅延・代表画像への参照。

AUTO/STEER_AUTOは自動記録。MANUAL/STOPで確定し、ダウンロード可能になる。
長い走行はサイズ/時間/枚数で分割される。古い診断ZIPのみ保存上限で削除する。
画像は通常10fpsの観測なので、全制御周期や電気的な実波形の記録ではない。
metadataに残る出力はソフトが発行した指令で、サーボ位置の実測ではない。
droppedはキュー超過、skipped_fpsは意図した間引き。どちらも制御を待たせないため。
"""


def _mode(metadata: dict[str, Any]) -> str:
    control = metadata.get("vehicle_control")
    if isinstance(control, dict) and control.get("mode") is not None:
        return str(control["mode"]).upper()
    return str(metadata.get("mode", "STOP")).upper()


def _safe(value: Any, depth: int = 0) -> Any:
    """Bound arbitrary diagnostics without allowing NaN/objects to break logging."""
    if depth > 10:
        return "[depth limit]"
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, str):
        return value[:4096]
    if isinstance(value, dict):
        return {str(k)[:160]: _safe(v, depth + 1) for k, v in list(value.items())[:256]}
    if isinstance(value, (list, tuple)):
        return [_safe(v, depth + 1) for v in value[:128]]
    # Preserve small geometry matrices (e.g. 3x3 perspective M) as JSON arrays.
    if hasattr(value, "tolist") and hasattr(value, "size"):
        if value.size > 128:
            return "[array size limit]"
        return _safe(value.tolist(), depth + 1)
    # NumPy scalar values appear in perception outputs.
    if hasattr(value, "item"):
        try:
            return _safe(value.item(), depth + 1)
        except (TypeError, ValueError):
            pass
    return str(value)[:256]


def _flatten(value: dict[str, Any], prefix: str = "") -> dict[str, Any]:
    result = {}
    for key, item in value.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(item, dict):
            result.update(_flatten(item, name))
        elif item is None or isinstance(item, (str, bool, int, float)):
            result[name] = item
    return result


def _distribution(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    def percentile(p):
        position = (len(ordered) - 1) * p
        low, high = math.floor(position), math.ceil(position)
        return round(ordered[low] + (ordered[high] - ordered[low]) * (position - low), 3)
    return {"count": len(ordered), "min": round(ordered[0], 3),
            "p50": percentile(.5), "p95": percentile(.95),
            "max": round(ordered[-1], 3)}


class DriveCapture:
    """Best-effort recorder. Errors are observable but never escape to driving.

    root is a directory dedicated to diagnostic archives, usually
    output/drive_capture. At most two image pairs wait for disk/encoding. Images
    must be uint8-like arrays no larger than 640x480; caller buffers are copied.
    ``close`` is idempotent and finalizes any pending frames on the worker.
    """
    def __init__(self, root, fps=10, max_frames=1800, max_session_s=180,
                 max_sessions=6, max_bytes=256 * 1024 * 1024,
                 jpeg_quality=82, finalizer_timeout=30., finalizer_pending=2):
        self.root = Path(root)
        self.fps = max(.1, min(30., float(fps)))
        self.max_frames = max(1, int(max_frames))
        self.max_session_s = max(.1, float(max_session_s))
        self.max_sessions = max(1, int(max_sessions))
        self.max_bytes = max(128 * 1024, int(max_bytes))
        # Reserving space avoids an unbounded open archive beside retained ZIPs.
        self.segment_bytes = min(64 * 1024 * 1024, self.max_bytes // 2)
        self.jpeg_quality = max(35, min(95, int(jpeg_quality)))
        self.finalizer_timeout = max(.05, float(finalizer_timeout))
        self.finalizer_pending = max(1, min(4, int(finalizer_pending)))
        self._finalizer_script = Path(__file__).with_name("drive_capture_finalize.py").resolve()
        self._finalize_jobs = deque()
        self._finalize_slots = threading.BoundedSemaphore(self.finalizer_pending)
        self._abort_finalizers = threading.Event()
        self._disk_lock = threading.Lock()
        self._writer_done = False
        self._finalizer_done = False
        self._mode_finish_pending = False
        self._child = None
        self._finalizing_job = None
        self._condition = threading.Condition()
        self._pending = deque()
        self._marks = deque(maxlen=32)
        self._generation = 0
        self._desired_generation = None
        self._mode = "STOP"
        self._run_counters = {"dropped": 0, "skipped_fps": 0}
        self._last_submit = -float("inf")
        self._closed = False
        self._worker_done = False
        self._stats = {
            "auto": True, "active": False, "finalizing": False,
            "frames": 0, "total_frames": 0, "dropped": 0,
            "skipped_fps": 0, "errors": 0, "error": None,
            "session_id": None, "finalizer_pid": None,
            "finalizer_completed": 0, "finalizer_queue_depth": 0,
            "latest_archive_href": None,
            "latest_archive_url": None, "archive_name": None,
        }
        self._thread = threading.Thread(target=self._run, name="drive-capture", daemon=True)
        self._finalizer_thread = threading.Thread(target=self._finalize_run, name="capture-finalizer-manager", daemon=True)
        self._finalizer_thread.start()
        self._thread.start()

    def _update_locked(self, mode):
        value = str(mode).upper()
        was_auto = self._desired_generation is not None
        now_auto = value in _AUTO_MODES
        if now_auto and not was_auto:
            self._generation += 1
            self._run_counters = {"dropped": 0, "skipped_fps": 0}
            self._desired_generation = self._generation
            self._last_submit = -float("inf")
            self._mode_finish_pending = False
            self._stats.update(frames=0, active=True, error=None)
            self._refresh_finalizing_locked()
        elif not now_auto and was_auto:
            self._desired_generation = None
            self._mode_finish_pending = True
            self._stats.update(active=False, finalizing=True)
            self._marks.clear()
        self._mode = value

    def update(self, mode):
        """Notify a real control mode even if there is no new image; never does I/O."""
        with self._condition:
            if self._closed or self._worker_done:
                return
            self._update_locked(mode)
            self._condition.notify_all()

    def submit(self, frame, mask, metadata, extra_masks=None):
        """Offer a synchronized frame without waiting for disk or image encoding."""
        try:
            with self._condition:
                if self._closed or self._worker_done:
                    return False
                self._update_locked(_mode(metadata))
                if self._desired_generation is None:
                    self._condition.notify_all()
                    return False
                now = time.monotonic()
                if not self._marks and now - self._last_submit < 1. / self.fps:
                    self._stats["skipped_fps"] += 1
                    self._run_counters["skipped_fps"] += 1
                    return False
                if (frame is None or mask is None or len(frame.shape) != 3 or
                        frame.shape[2] != 3 or len(mask.shape) != 2 or
                        frame.shape[:2] != mask.shape or
                        frame.shape[0] * frame.shape[1] > 640 * 480):
                    raise ValueError("capture requires a matching BGR image and gray mask, <=640x480")
                if extra_masks is None:
                    extra_masks = {}
                if not isinstance(extra_masks, dict) or len(extra_masks) > 4:
                    raise ValueError("extra_masks must be a dict with at most four matching gray masks")
                for key, extra in extra_masks.items():
                    if not isinstance(key, str) or re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,31}", key) is None:
                        raise ValueError("extra mask keys must be 1-32 ASCII letters/digits/underscore")
                    if extra is None or len(extra.shape) != 2 or extra.shape != mask.shape:
                        raise ValueError("extra mask shape must match the BGR image and primary mask")
                task = {
                    "generation": self._desired_generation, "mode": self._mode,
                    "time": time.time(), "monotonic": now,
                    "frame": frame.copy(), "mask": mask.copy(),
                    "extra_masks": {key: value.copy() for key, value in extra_masks.items()},
                    "metadata": copy.deepcopy(metadata), "marks": list(self._marks),
                    "run_counters": self._run_counters,
                }
                self._marks.clear()
                if len(self._pending) >= 2:
                    dropped = self._pending.popleft()
                    # A marked frame stays traceable when overload discards it.
                    task["marks"] = (dropped["marks"] + task["marks"])[-32:]
                    self._stats["dropped"] += 1
                    dropped["run_counters"]["dropped"] += 1
                self._pending.append(task)
                self._last_submit = now
                self._condition.notify_all()
                return True
        except Exception as exc:
            self._error(exc)
            return False

    def mark(self, label="確認ポイント"):
        """Mark the next image pair (also bypasses FPS sampling) during AUTO."""
        with self._condition:
            if self._closed or self._worker_done or self._desired_generation is None:
                return False
            self._marks.append({"label": str(label)[:160], "time_unix_ms": round(time.time() * 1000)})
            self._condition.notify_all()
            return True

    def status(self):
        with self._condition:
            return {**self._stats, "mode": self._mode,
                    "queue_depth": len(self._pending), "fps": self.fps,
                    "closed": self._closed, "worker_done": self._worker_done}

    def _refresh_finalizing_locked(self):
        self._stats["finalizer_queue_depth"] = len(self._finalize_jobs)
        self._stats["finalizing"] = bool(self._mode_finish_pending or self._finalize_jobs or self._finalizing_job or self._child)

    def close(self, timeout=10):
        deadline = time.monotonic() + max(0., timeout)
        with self._condition:
            self._closed = True
            self._desired_generation = None
            self._stats["active"] = False
            self._mode_finish_pending = not self._writer_done
            self._refresh_finalizing_locked()
            self._condition.notify_all()
        for thread in (self._thread, self._finalizer_thread):
            if threading.current_thread() is not thread:
                thread.join(timeout=max(0., deadline-time.monotonic()))
        if self._thread.is_alive() or self._finalizer_thread.is_alive():
            self._error(TimeoutError("capture close deadline exceeded; finalizers cancelled"))
            self._abort_finalizers.set()
            with self._condition:
                child = self._child
                self._condition.notify_all()
            if child is not None:
                try:
                    child.terminate()
                except ProcessLookupError:
                    pass
            # The manager reaps the terminated process; this bounded cleanup
            # allowance avoids leaving an orphan after a timed-out close.
            if threading.current_thread() is not self._finalizer_thread:
                self._finalizer_thread.join(timeout=1.)
            if threading.current_thread() is not self._thread:
                self._thread.join(timeout=.25)

    def _error(self, exc):
        with self._condition:
            self._stats["errors"] += 1
            self._stats["error"] = f"{type(exc).__name__}: {exc}"[:240]

    def _owned_paths(self):
        return [path for path in self.root.glob(_PREFIX + "*")
                if path.is_file() and (path.name.endswith(".zip") or
                   path.name.endswith(".rows.jsonl") or path.name.endswith(".finalize.json"))]

    def _prune(self, reserve=0):
        # Includes active ZIPs/metadata sidecars. Only completed diagnostic ZIPs
        # may be pruned; a queued or active finalizer's inputs are protected.
        with self._disk_lock:
            # The child atomically renames its ZIP and removes sidecars. A
            # snapshot entry may disappear between glob and stat; that is a
            # successful finalization transition, not a recording error.
            snapshot = []
            for path in self._owned_paths():
                try:
                    info = path.stat()
                except FileNotFoundError:
                    continue
                snapshot.append((path, info.st_size, info.st_mtime))
            with self._condition:
                jobs = list(self._finalize_jobs)
                if self._finalizing_job is not None:
                    jobs.append(self._finalizing_job)
                protected = {job["final_name"] for job in jobs}
            completed = sorted((entry for entry in snapshot if entry[0].name.endswith(".zip")
                                and not entry[0].name.endswith(".partial.zip")),
                               key=lambda entry: entry[2])
            completed_count = len(completed)
            size = sum(entry[1] for entry in snapshot)
            archives = [entry for entry in completed if entry[0].name not in protected]
            while archives and (completed_count > self.max_sessions - bool(reserve) or size + reserve > self.max_bytes):
                path, stored_size, _ = archives.pop(0)
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
                completed_count -= 1
                size -= stored_size
                with self._condition:
                    if self._stats["archive_name"] == path.name:
                        self._stats.update(archive_name=None, latest_archive_href=None, latest_archive_url=None)
            return size + reserve <= self.max_bytes

    def _wait_for_storage(self):
        while True:
            # While its sidecar still exists, a child adds frames.jsonl and CSV
            # to the ZIP. Reserve that temporary duplication conservatively,
            # including queued jobs, before opening another recording segment.
            with self._condition:
                jobs = list(self._finalize_jobs)
                if self._finalizing_job is not None:
                    jobs.append(self._finalizing_job)
                extra = sum(2 * job["metadata_bytes"] + 65536 for job in jobs)
            if self._prune(self.segment_bytes + extra):
                return
            if self._abort_finalizers.wait(.025):
                raise RuntimeError("capture finalization cancelled")
            with self._condition:
                if self._finalizer_done:
                    raise OSError("recording storage full and no finalizer can make progress")

    @staticmethod
    def _discard_job(job):
        for key in ("partial_name", "rows_name", "job_name"):
            try:
                (job["root"] / job[key]).unlink(missing_ok=True)
            except OSError:
                pass

    @staticmethod
    def _discard_clip(clip):
        try:
            clip["rows_file"].close()
        except Exception:
            pass
        try:
            clip["archive"].close()
        except Exception:
            pass
        for path in (clip["path"], clip["rows_path"]):
            try:
                path.unlink(missing_ok=True)
            except OSError:
                pass

    def _new_clip(self, task, segment, run_id):
        self._wait_for_storage()
        name = f"{_PREFIX}{run_id}-s{segment:03d}"
        path = self.root / (name + ".partial.zip")
        archive = zipfile.ZipFile(path, "w", compression=zipfile.ZIP_STORED, allowZip64=True)
        rows_path = self.root / (name + ".rows.jsonl")
        rows_file = rows_path.open("w", encoding="utf-8")
        clip = {
            "archive": archive, "path": path, "name": name,
            "generation": task["generation"], "session_id": run_id,
            "segment": segment, "start": task["monotonic"],
            "start_time": task["time"], "frames": 0, "events": [],
            "rows_path": rows_path, "rows_file": rows_file,
            "modes": set(), "extra_masks": set(), "last_row": None,
            "metadata_bytes": 0, "run_counters": task["run_counters"],
            "dropped_start": 0 if segment == 1 else task["run_counters"]["dropped"],
            "fps_start": 0 if segment == 1 else task["run_counters"]["skipped_fps"],
        }
        archive.writestr("README.txt", _README.encode("utf-8"))
        with self._condition:
            self._stats["session_id"] = run_id
            self._stats["frames"] = 0
        return clip

    def _write_frame(self, clip, task, cv2):
        encode_start = time.monotonic()
        ok, image = cv2.imencode(".jpg", task["frame"], [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise ValueError("JPEG encoder returned false")
        ok, mask = cv2.imencode(".png", task["mask"], [cv2.IMWRITE_PNG_COMPRESSION, 2])
        if not ok:
            raise ValueError("PNG encoder returned false")
        extra_images = {}
        for key, value in task["extra_masks"].items():
            ok, extra = cv2.imencode(".png", value, [cv2.IMWRITE_PNG_COMPRESSION, 2])
            if not ok:
                raise ValueError("extra PNG encoder returned false")
            extra_images[key] = extra
        index = clip["frames"] + 1
        image_path, mask_path = f"images/{index:06d}.jpg", f"masks/{index:06d}.png"
        extra_paths = {key: f"extra_masks/{key}/{index:06d}.png" for key in extra_images}
        metadata = _safe(task["metadata"])
        serialized = json.dumps(metadata, ensure_ascii=False, allow_nan=False)
        if len(serialized.encode("utf-8")) > 32768:
            metadata = {"mode": task["mode"], "metadata_truncated": True,
                        "note": "metadata exceeded 32768 bytes"}
        row = {
            "frame": index, "image": image_path, "mask": mask_path, "extra_masks": extra_paths,
            "capture_time_unix_ms": round(task["time"] * 1000, 3),
            "relative_time_ms": round((task["monotonic"] - clip["start"]) * 1000, 3),
            "mode": task["mode"], "metadata": metadata,
            "recording": {
                "queue_ms": round((encode_start - task["monotonic"]) * 1000, 3),
                "encode_ms": round((time.monotonic() - encode_start) * 1000, 3),
            },
        }
        archive = clip["archive"]
        row_json = json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n"
        row_size = len(row_json.encode("utf-8"))
        projected = archive.fp.tell() + len(image) + len(mask) + sum(len(extra) for extra in extra_images.values()) + 2 * (clip["metadata_bytes"] + row_size) + 32768
        if projected > self.segment_bytes:
            if clip["frames"]:
                return False
            raise ValueError("one capture frame exceeds archive byte budget")
        archive.writestr(image_path, image.tobytes())
        archive.writestr(mask_path, mask.tobytes())
        for key, extra in extra_images.items():
            archive.writestr(extra_paths[key], extra.tobytes())
        clip["rows_file"].write(row_json)
        clip["frames"] = index
        clip["last_row"] = row
        clip["modes"].add(row["mode"])
        clip["extra_masks"].update(extra_paths)
        clip["metadata_bytes"] += row_size
        for marker in task["marks"]:
            clip["events"].append({**marker, "frame": index, "image": image_path, "mask": mask_path})
        clip["events"] = clip["events"][-256:]
        with self._condition:
            self._stats["frames"] = index
            self._stats["total_frames"] += 1
        return True

    def _finish(self, clip, reason):
        """Seal small writer state and enqueue CPU-heavy work in another PID."""
        if not clip["frames"]:
            self._discard_clip(clip)
            return
        clip["rows_file"].close()
        clip["archive"].close()
        stats = self.status()
        summary = {
            "schema_version": 1, "session_id": clip["session_id"],
            "segment": clip["segment"], "frames": clip["frames"],
            "started_time_unix_ms": round(clip["start_time"] * 1000),
            "last_frame_time_unix_ms": clip["last_row"]["capture_time_unix_ms"],
            "duration_s": round(clip["last_row"]["relative_time_ms"] / 1000, 3),
            "reason": reason, "fps_target": self.fps,
            "dropped": clip["run_counters"]["dropped"] - clip["dropped_start"],
            "skipped_fps": clip["run_counters"]["skipped_fps"] - clip["fps_start"],
            "dropped_process_total": stats["dropped"],
            "recording_errors_total": stats["errors"],
            "modes": sorted(clip["modes"]), "markers": len(clip["events"]),
            "image_format": "JPEG", "mask_format": "lossless PNG",
            "extra_masks": sorted(clip["extra_masks"]),
            "actuator_values_are_commands_not_position_measurements": True,
        }
        job = dict(schema_version=1, parent_pid=os.getpid(), root=self.root.resolve(),
                   partial_name=clip["path"].name, rows_name=clip["rows_path"].name,
                   final_name=clip["name"] + ".zip", job_name=clip["name"] + ".finalize.json",
                   metadata_bytes=clip["metadata_bytes"], summary=summary, events=clip["events"])
        # Queue capacity waits only on this writer thread, never the producer
        # condition. The existing two-frame producer queue stays bounded.
        while not self._finalize_slots.acquire(timeout=.025):
            if self._abort_finalizers.is_set():
                self._discard_clip(clip)
                raise RuntimeError("capture finalization cancelled")
        try:
            if self._abort_finalizers.is_set():
                raise RuntimeError("capture finalization cancelled")
            serializable = {key: value for key, value in job.items() if key != "root"}
            (job["root"] / job["job_name"]).write_text(
                json.dumps(serializable, ensure_ascii=False, allow_nan=False), encoding="utf-8")
            with self._condition:
                self._finalize_jobs.append(job)
                self._refresh_finalizing_locked()
                self._condition.notify_all()
        except BaseException:
            self._finalize_slots.release()
            self._discard_job(job)
            raise

    def _run_finalizer_job(self, job):
        command = [str(Path(sys.executable).resolve()), str(self._finalizer_script.resolve()),
                   "--job", str(job["root"] / job["job_name"])]
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.PIPE, start_new_session=True)
        with self._condition:
            self._child = process
            self._stats["finalizer_pid"] = process.pid
            self._refresh_finalizing_locked()
        deadline = time.monotonic() + self.finalizer_timeout
        failure = None
        try:
            while True:
                if self._abort_finalizers.is_set() or time.monotonic() >= deadline:
                    failure = TimeoutError("capture finalizer cancelled or timed out")
                    process.terminate()
                    try:
                        _, error = process.communicate(timeout=.3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        _, error = process.communicate(timeout=1.)
                    break
                try:
                    _, error = process.communicate(timeout=.025)
                    break
                except subprocess.TimeoutExpired:
                    continue
            if failure is not None:
                raise failure
            if process.returncode:
                detail = error.decode("utf-8", errors="replace").strip()[:240]
                raise RuntimeError(f"capture finalizer exit {process.returncode}: {detail}")
            final = job["root"] / job["final_name"]
            if not final.is_file():
                raise RuntimeError("capture finalizer did not produce its archive")
            self._prune()
            if final.exists():
                href = "/output/drive_capture/" + final.name
                with self._condition:
                    self._stats.update(archive_name=final.name, latest_archive_href=href,
                                       latest_archive_url=href)
            with self._condition:
                self._stats["finalizer_completed"] += 1
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=1.)
            with self._condition:
                self._child = None
                self._stats["finalizer_pid"] = None
                self._refresh_finalizing_locked()
                self._condition.notify_all()

    def _finalize_run(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._finalize_jobs or self._writer_done or self._abort_finalizers.is_set())
                    if self._finalize_jobs:
                        job = self._finalize_jobs.popleft()
                        self._finalizing_job = job
                        self._finalize_slots.release()
                        self._refresh_finalizing_locked()
                    elif self._writer_done or self._abort_finalizers.is_set():
                        break
                    else:
                        continue
                try:
                    if self._abort_finalizers.is_set():
                        self._discard_job(job)
                    else:
                        self._run_finalizer_job(job)
                except Exception as error:
                    self._error(error)
                    self._discard_job(job)
                finally:
                    with self._condition:
                        self._finalizing_job = None
                        self._refresh_finalizing_locked()
                        self._condition.notify_all()
        finally:
            with self._condition:
                pending = list(self._finalize_jobs)
                self._finalize_jobs.clear()
                self._finalizer_done = True
                self._worker_done = self._writer_done
                self._refresh_finalizing_locked()
                self._condition.notify_all()
            for job in pending:
                self._discard_job(job)

    def _run(self):
        clip = None
        generation, segment, run_id = None, 0, None
        try:
            # Initialization, discovery and cleanup are worker-only I/O.
            self.root.mkdir(parents=True, exist_ok=True)
            for pattern in ("*.partial.zip", "*.rows.jsonl", "*.finalize.json"):
                for partial in self.root.glob(_PREFIX + pattern):
                    partial.unlink()
            self._prune()
            archives = sorted(self.root.glob(_PREFIX + "*.zip"), key=lambda p: p.stat().st_mtime)
            if archives:
                name = archives[-1].name
                with self._condition:
                    self._stats.update(archive_name=name, latest_archive_href="/output/drive_capture/" + name,
                                       latest_archive_url="/output/drive_capture/" + name)
            import cv2  # Lazy, and an unavailable encoder cannot terminate vehicle control.
            while True:
                if self._abort_finalizers.is_set():
                    break
                with self._condition:
                    if not self._pending:
                        desired = self._desired_generation
                        closing = self._closed
                        should_finalize = clip is not None and (closing or desired != clip["generation"])
                        if not should_finalize and not closing:
                            self._condition.wait(timeout=.5)
                            continue
                        task = None
                    else:
                        task = self._pending.popleft()
                        closing = False
                if task is None:
                    if clip is not None:
                        self._finish(clip, "close" if closing else "mode_changed")
                        clip = None
                    with self._condition:
                        if self._desired_generation is None:
                            self._mode_finish_pending = False
                            self._refresh_finalizing_locked()
                    if closing:
                        break
                    continue
                try:
                    if clip is not None and task["generation"] != clip["generation"]:
                        self._finish(clip, "mode_changed")
                        clip = None
                    if task["generation"] != generation:
                        generation, segment = task["generation"], 0
                        run_id = time.strftime("%Y%m%d-%H%M%S", time.localtime(task["time"]))
                        run_id += "-" + uuid.uuid4().hex[:8]
                    if clip is not None and (
                            clip["frames"] >= self.max_frames or
                            task["monotonic"] - clip["start"] >= self.max_session_s or
                            clip["archive"].fp.tell() + clip["metadata_bytes"] * 2 + 65536 >= self.segment_bytes or
                            clip["metadata_bytes"] >= 8 * 1024 * 1024):
                        self._finish(clip, "segment_limit")
                        clip = None
                    if clip is None:
                        segment += 1
                        clip = self._new_clip(task, segment, run_id)
                    if not self._write_frame(clip, task, cv2):
                        self._finish(clip, "segment_bytes")
                        segment += 1
                        clip = self._new_clip(task, segment, run_id)
                        self._write_frame(clip, task, cv2)
                except Exception as exc:
                    self._error(exc)
                    if clip is not None:
                        self._discard_clip(clip)
                        clip = None
        except Exception as exc:
            self._error(exc)
        finally:
            if clip is not None:
                if self._abort_finalizers.is_set():
                    self._discard_clip(clip)
                else:
                    try:
                        self._finish(clip, "worker_exit")
                    except Exception as exc:
                        self._error(exc)
                        self._discard_clip(clip)
            with self._condition:
                self._writer_done = True
                self._worker_done = self._finalizer_done
                self._mode_finish_pending = False
                self._stats["active"] = False
                self._pending.clear()
                self._refresh_finalizing_locked()
                self._condition.notify_all()
