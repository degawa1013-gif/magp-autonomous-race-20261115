"""Find connected far-road curvature without changing the normal line tracker.

Coordinates remain in the existing bird-view plane.  Negative y values are
visible road above that plane's current output crop, not extrapolated points.
The result contains geometry only; the caller owns any steering policy.
"""
import math
from functools import lru_cache

import cv2
import numpy as np


_X_OFFSET = 160
_SAMPLE_WIDTH = 640
_FARTHEST_Y = -190
_ROW_STEP = 10
_MIN_EXTRA_SPAN = 100
_MAX_ROW_WIDTH = 80
_MAX_PREDICTION_ERROR = 25.0


def _empty(reason, **diagnostics):
    return {"valid": False, "reason": reason, "points": [],
            "near_heading_deg": 0.0, "far_heading_deg": 0.0,
            "tangent_delta_deg": 0.0, "far_departure_px": 0.0,
            "point_count": 0, **diagnostics}


def _line_fit(points):
    values = np.asarray(points, dtype=np.float64)
    mean_x, mean_y = np.mean(values, axis=0)
    centered_y = values[:, 1] - mean_y
    denominator = float(np.dot(centered_y, centered_y))
    slope = float(np.dot(centered_y, values[:, 0] - mean_x)) / denominator
    return slope, float(mean_x - slope * mean_y)


@lru_cache(maxsize=8)
def _sampling_projection(matrix_key, last_y):
    """Cache inverse maps for measured cross sections, not a full bird image."""
    matrix = np.asarray(matrix_key, dtype=np.float64).reshape(3, 3)
    inverse = np.linalg.inv(matrix)
    ys = np.arange(last_y - _ROW_STEP, _FARTHEST_Y - 1, -_ROW_STEP)
    grid_x, grid_y = np.meshgrid(np.arange(_SAMPLE_WIDTH) - _X_OFFSET, ys)
    world = np.stack((grid_x, grid_y), axis=-1).astype(np.float32)
    camera = cv2.perspectiveTransform(world, inverse)
    # Integer nearest-neighbour maps avoid recomputing perspective division on
    # every frame and sample only the cross sections actually used below.
    map_one, map_two = cv2.convertMaps(camera[:, :, 0], camera[:, :, 1],
                                      cv2.CV_16SC2, nninterpolation=True)
    inverse.flags.writeable = False
    ys.flags.writeable = False
    map_one.flags.writeable = False
    if map_two is not None:
        map_two.flags.writeable = False
    return inverse, ys, map_one, map_two


def extract_far_preview(mask, points, perspective_matrix):
    """Return a signed tangent change on the currently tracked white component.

    ``mask`` must be the full adaptive camera mask before the optional bright
    subset and bird-view crop.  ``points`` are independently accepted base-line
    observations.  A disconnected bright object cannot acquire a preview, and
    translation or yaw of a straight line does not create curvature.
    """
    base = sorted([(float(x), float(y)) for x, y in points],
                  key=lambda point: -point[1])
    if (len(base) < 6 or base[0][1] - base[-1][1] < 100
            or not all(math.isfinite(value) for point in base for value in point)):
        return _empty("base_support_short")
    if mask is None or mask.ndim != 2 or not np.any(mask):
        return _empty("mask_empty")
    matrix = np.asarray(perspective_matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        return _empty("invalid_projection")
    try:
        inverse, sample_ys, map_one, map_two = _sampling_projection(
            tuple(matrix.ravel()), int(base[-1][1]))
    except np.linalg.LinAlgError:
        return _empty("invalid_projection")
    # Connectedness is checked in the original 320 x 240 camera mask.  Labelling
    # an expanded 640 x 480 warp was substantially slower on the vehicle Pi.
    _, labels = cv2.connectedComponents(mask, connectivity=8)
    camera_anchors = cv2.perspectiveTransform(
        np.asarray([base], dtype=np.float32), inverse)[0]
    votes = {}
    for x, y in camera_anchors:
        ix, iy = int(round(float(x))), int(round(float(y)))
        patch = labels[max(0, iy - 3):min(labels.shape[0], iy + 4),
                       max(0, ix - 5):min(labels.shape[1], ix + 6)]
        present = patch[patch > 0]
        if present.size:
            label = int(np.bincount(present).argmax())
            votes[label] = votes.get(label, 0) + 1
    if not votes:
        return _empty("anchor_missing")
    anchor, anchor_count = max(votes.items(), key=lambda item: item[1])
    if anchor_count < max(4, len(base) - 2):
        return _empty("anchor_weak", anchor_count=anchor_count)
    component = np.asarray(labels == anchor, dtype=np.uint8)
    sections = cv2.remap(component, map_one, map_two, cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    trail = list(base)
    widths = []
    termination = "far_limit"
    for row_index, y in enumerate(sample_ys):
        row = sections[row_index]
        # Each uninterrupted run has a midpoint and width available directly
        # from its endpoints; no per-run arrays or means are needed.
        edges = np.diff(np.r_[0, row, 0].astype(np.int8))
        starts = np.flatnonzero(edges == 1)
        ends = np.flatnonzero(edges == -1)
        slope = ((trail[-1][0] - trail[-3][0]) / (trail[-1][1] - trail[-3][1]))
        predicted = trail[-1][0] + slope * (y - trail[-1][1])
        candidates = []
        for start, end in zip(starts, ends):
            width = int(end - start)
            if width > _MAX_ROW_WIDTH:
                continue
            x = (float(start) + float(end) - 1.) * .5 - _X_OFFSET
            error = abs(x - predicted)
            if error <= _MAX_PREDICTION_ERROR:
                candidates.append((error, x, width))
        if not candidates:
            termination = "component_ended"
            break
        candidates.sort()
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 8.0:
            termination = "ambiguous_branch"
            break
        _, x, width = candidates[0]
        trail.append((x, float(y)))
        widths.append(width)
    extra_span = base[-1][1] - trail[-1][1]
    # At least five rows beyond y=0 and a 100 px measured continuation are
    # required.  A mere near-line heading estimate cannot enable this cue.
    far_points = [point for point in trail[len(base):] if point[1] <= 0]
    if extra_span < _MIN_EXTRA_SPAN or len(far_points) < 5:
        return _empty("far_support_short", anchor_count=anchor_count,
                      extra_span_y_px=extra_span, termination=termination)
    near = [point for point in base if point[1] >= 70]
    near_slope, near_intercept = _line_fit(near)
    far_slope, _ = _line_fit(far_points[-7:])
    near_heading = math.degrees(math.atan(-near_slope))
    far_heading = math.degrees(math.atan(-far_slope))
    far_departure = float(np.mean([x - near_slope * y - near_intercept
                                   for x, y in far_points[-3:]]))
    return {"valid": True, "reason": "connected_far_road",
            "points": [[round(x, 3), round(y, 3)] for x, y in trail[len(base):]],
            "near_heading_deg": near_heading,
            "far_heading_deg": far_heading,
            "tangent_delta_deg": far_heading - near_heading,
            "far_departure_px": far_departure,
            "point_count": len(trail) - len(base),
            "far_top_y": trail[-1][1], "extra_span_y_px": extra_span,
            "anchor_count": anchor_count, "base_point_count": len(base),
            "max_row_width_px": max(widths), "termination": termination}
