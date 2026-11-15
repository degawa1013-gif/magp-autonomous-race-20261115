"""Bounded steering anticipation from an already observed, bending path.

This module has no camera, GPIO, throttle, or mode-changing dependencies. The
caller adds ``extra_duty`` to the proven tracking target before its existing
actuator bounds and smoothing. Command percentage schedules preview strength;
it is deliberately not presented as measured velocity.

The existing position/heading controller remains responsible for straight and
diagonal paths. Only a supported change of tangent can create an extra term.
The preview point stays inside the observation, leaving its furthest sample as
a guard against endpoint fitting errors. Geometry is in the existing 320x240
bird-view coordinates, not a calibrated ground-distance coordinate system.
"""
import math


START_COMMAND_PERCENT = 28.0
FULL_COMMAND_PERCENT = 48.0
PREVIEW_Y_START = 90.0
PREVIEW_Y_FULL = 70.0
MIN_POINTS = 6
MIN_SPAN_Y = 100.0
MIN_CONFIDENCE = 0.90
BEND_START_DEGREES = 2.0
BEND_FULL_DEGREES = 8.0
MIN_FAR_DEPARTURE_PIXELS = 2.0
MAX_FIT_RMS_PIXELS = 3.0
MIN_CURVATURE_SIGNAL_SIGMA = 3.0
MAX_EXTRA_DUTY = 0.40
RESPONSE_SECONDS = 0.060
FAR_BEND_START_DEGREES = 8.0
FAR_BEND_FULL_DEGREES = 24.0
FAR_MIN_DEPARTURE_PIXELS = 5.0
FAR_HEADING_STEPS_PER_DEGREE = 0.14
# Only a long, well-anchored upper path can increase the early-turn authority.
# Ordinary bends keep the prior 0.40 limit and gain exactly. Both shape ramps
# are continuous so a threshold crossing cannot produce a new steering step.
SHARP_BEND_START_DEGREES = 35.0
SHARP_BEND_FULL_DEGREES = 55.0
SHARP_DEPARTURE_START_PIXELS = 40.0
SHARP_DEPARTURE_FULL_PIXELS = 90.0
SHARP_MIN_ANCHORS = 6
SHARP_MIN_POINTS = 14
SHARP_MIN_EXTRA_SPAN_Y = 140.0
SHARP_MAX_EXTRA_DUTY = 1.0
SHARP_MAX_HEADING_FACTOR = 1.8
SHARP_SECONDARY_MIN_ANCHORS = 8
SHARP_SECONDARY_MIN_POINTS = 10
SHARP_SECONDARY_MIN_EXTRA_SPAN_Y = 100.0
SHARP_SECONDARY_QUALITY = 0.6
SHARP_RELEASE_SECONDS = 0.120


def _clip(value, low, high):
    return max(low, min(high, value))


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (ValueError, TypeError, OverflowError):
        return False


def _local_line(points):
    """Fit x against forward progress for a local tangent and its intercept."""
    ts = [-point[1] for point in points]
    xs = [point[0] for point in points]
    tm, xm = sum(ts) / len(ts), sum(xs) / len(xs)
    denominator = sum((t - tm) ** 2 for t in ts)
    if denominator <= 1e-9:
        return None
    slope = sum((t - tm) * (x - xm) for t, x in zip(ts, xs)) / denominator
    return slope, xm - slope * tm


def _quadratic(points):
    """Same index-space quadratic as the tracking controller, without NumPy.

    Consecutive indices centered on zero have vanishing odd moments. This
    reduces the three-parameter least-squares fit to a tiny stable calculation.
    """
    midpoint = (len(points) - 1.0) / 2.0
    ts = [index - midpoint for index in range(len(points))]
    xs = [point[0] for point in points]
    s2 = sum(t * t for t in ts)
    s4 = sum(t ** 4 for t in ts)
    mean_x = sum(xs) / len(xs)
    a = (sum(t * t * x for t, x in zip(ts, xs)) - s2 * mean_x) / (
        s4 - s2 * s2 / len(points))
    b = sum(t * x for t, x in zip(ts, xs)) / s2
    c = mean_x - a * s2 / len(points)
    residual = math.sqrt(sum((a * t * t + b * t + c - x) ** 2
                             for t, x in zip(ts, xs)) / len(points))
    return a, b, c, midpoint, residual


def _path_point(points, target_y, fit, blend):
    position = None
    for index, ((x0, y0), (x1, y1)) in enumerate(zip(points, points[1:])):
        if y0 >= target_y >= y1:
            fraction = (y0 - target_y) / (y0 - y1)
            position = index + fraction
            interp_x = x0 + (x1 - x0) * fraction
            break
    if position is None:
        return None
    a, b, c, midpoint, _ = fit
    t = position - midpoint
    fit_x = _clip(a * t * t + b * t + c,
                  min(x for x, y in points) - 18.0,
                  max(x for x, y in points) + 18.0)
    return interp_x * (1.0 - blend) + fit_x * blend, 2.0 * a * t + b


def _far_candidate(observation, command_blend, heading_gain, gain_scale):
    """Consume the connected upper-view extractor's verified geometry only."""
    result = dict(valid=False, reason="not_supplied", candidate_duty=0.0,
                  signed_bend_deg=None, far_departure_px=None, preview_y=None,
                  sharp_blend=0.0, sharp_supported=False, sharp_support_quality=0.0,
                  maximum_extra_duty=MAX_EXTRA_DUTY)
    if observation is None:
        return result
    if not isinstance(observation, dict) or observation.get("valid") is not True:
        result["reason"] = "unreliable_far_observation"
        return result
    keys = ("near_heading_deg", "far_heading_deg", "tangent_delta_deg",
            "far_departure_px", "far_top_y", "anchor_count", "point_count")
    if not all(_finite(observation.get(key)) for key in keys):
        result["reason"] = "invalid_far_observation"
        return result
    near, far = float(observation["near_heading_deg"]), float(observation["far_heading_deg"])
    bend = float(observation["tangent_delta_deg"])
    departure = float(observation["far_departure_px"])
    result.update(signed_bend_deg=bend, far_departure_px=departure,
                  preview_y=float(observation["far_top_y"]))
    # Signed tangent difference, rather than absolute global heading, is the
    # bend cue. The extractor's accepted base anchors must remain present.
    if (abs((far - near) - bend) > .25 or abs(near) >= 85.0 or abs(far) >= 85.0
            or float(observation["anchor_count"]) < 4
            or float(observation["point_count"]) < 6
            or float(observation["far_top_y"]) > 0.0):
        result["reason"] = "invalid_far_geometry"
        return result
    if abs(bend) <= FAR_BEND_START_DEGREES or abs(departure) <= FAR_MIN_DEPARTURE_PIXELS:
        result["reason"] = "far_straight_or_small_bend"
        return result
    if bend * departure <= 0.0:
        result["reason"] = "conflicting_far_geometry"
        return result
    strength = _clip((abs(bend) - FAR_BEND_START_DEGREES)
                     / (FAR_BEND_FULL_DEGREES - FAR_BEND_START_DEGREES), 0.0, 1.0)
    support = (float(observation["anchor_count"]) >= SHARP_MIN_ANCHORS
               and float(observation["point_count"]) >= SHARP_MIN_POINTS
               and _finite(observation.get("extra_span_y_px"))
               and float(observation["extra_span_y_px"]) >= SHARP_MIN_EXTRA_SPAN_Y)
    secondary_support = (float(observation["anchor_count"]) >= SHARP_SECONDARY_MIN_ANCHORS
                         and float(observation["point_count"]) >= SHARP_SECONDARY_MIN_POINTS
                         and _finite(observation.get("extra_span_y_px"))
                         and float(observation["extra_span_y_px"]) >= SHARP_SECONDARY_MIN_EXTRA_SPAN_Y)
    quality = 1.0 if support else SHARP_SECONDARY_QUALITY if secondary_support else 0.0
    sharp_blend = 0.0
    if quality:
        sharp_blend = quality * min(
            _clip((abs(bend) - SHARP_BEND_START_DEGREES)
                  / (SHARP_BEND_FULL_DEGREES - SHARP_BEND_START_DEGREES), 0.0, 1.0),
            _clip((abs(departure) - SHARP_DEPARTURE_START_PIXELS)
                  / (SHARP_DEPARTURE_FULL_PIXELS - SHARP_DEPARTURE_START_PIXELS), 0.0, 1.0))
    maximum = MAX_EXTRA_DUTY + sharp_blend * (SHARP_MAX_EXTRA_DUTY - MAX_EXTRA_DUTY)
    heading_factor = 1.0 + sharp_blend * (SHARP_MAX_HEADING_FACTOR - 1.0)
    delta = (bend * float(heading_gain) * FAR_HEADING_STEPS_PER_DEGREE
             * float(gain_scale) * heading_factor)
    if not _finite(delta):
        result["reason"] = "invalid_far_geometry"
        return result
    candidate = _clip(delta, -maximum, maximum) * command_blend * strength
    result.update(valid=True, reason="connected_far_bend", candidate_duty=candidate,
                  sharp_blend=sharp_blend, sharp_supported=bool(quality),
                  sharp_support_quality=quality, maximum_extra_duty=maximum)
    return result


class SteeringPreview:
    """Observe a supported bend and return a small independent target delta.

    There is no fixed two-frame confirmation delay: two spatial tangent groups
    and far departure provide evidence within the same observation. A 60 ms
    first-order filter suppresses the abrupt addition of that evidence. Invalid
    observations and a straight path immediately clear the extra term; the
    caller's established LOST hold/recenter policy remains authoritative.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self._extra = 0.0
        self._direction = 0

    def update(self, points, confidence, perception_mode, requested_percent, dt,
               *, base_control_y=116.0, base_control_x=None,
               base_heading_step=None, center_gain=0.01, heading_gain=0.085,
               gain_scale=1.0, path_fit_blend=0.88, enabled=True, far_observation=None):
        result = dict(active=False, reason="disabled", extra_duty=0.0,
                      candidate_duty=0.0, unbounded_delta_duty=0.0,
                      command_blend=0.0, bend_blend=0.0, preview_y=None,
                      near_heading_deg=None, far_heading_deg=None,
                      signed_bend_deg=None, far_departure_px=None,
                      interior_departure_px=None,
                      fit_rms_px=None, curvature_signal_sigma=None, point_count=0,
                      source="none", far_preview=None)
        far_allowed = False

        def finish():
            return self._finish(result, far_observation if far_allowed else None,
                                dt, heading_gain, gain_scale)

        def reject(reason):
            result["reason"] = reason
            if far_allowed:
                return finish()
            self.reset()
            return result

        if not enabled:
            return reject("disabled")
        values = (confidence, requested_percent, base_control_y, center_gain,
                  heading_gain, gain_scale, path_fit_blend)
        if (not all(_finite(value) for value in values)
                or (base_control_x is not None and not _finite(base_control_x))
                or (base_heading_step is not None and not _finite(base_heading_step))
                or float(center_gain) < 0.0 or float(heading_gain) < 0.0
                or float(gain_scale) < 0.0
                or not 0.0 <= float(path_fit_blend) <= 1.0):
            return reject("invalid_input")
        if perception_mode != "TRACK" or float(confidence) < MIN_CONFIDENCE:
            return reject("unreliable_line")
        command_blend = _clip((float(requested_percent) - START_COMMAND_PERCENT)
                              / (FULL_COMMAND_PERCENT - START_COMMAND_PERCENT), 0.0, 1.0)
        result["command_blend"] = command_blend
        if command_blend <= 0.0:
            return reject("baseline_command")
        try:
            parsed = [(float(x), float(y)) for x, y in points]
        except (ValueError, TypeError, OverflowError):
            return reject("invalid_points")
        result["point_count"] = len(parsed)
        if (not all(_finite(x) and _finite(y) and 0.0 <= x <= 320.0
                    and 0.0 <= y <= 240.0 for x, y in parsed)
                or len({y for x, y in parsed}) != len(parsed)):
            return reject("invalid_points")
        parsed.sort(key=lambda point: -point[1])
        if len(parsed) < MIN_POINTS or parsed[0][1] - parsed[-1][1] < MIN_SPAN_Y:
            return reject("short_path")
        group = max(3, (len(parsed) + 1) // 2)
        near = _local_line(parsed[:group])
        far = _local_line(parsed[-group:])
        if near is None or far is None:
            return reject("invalid_tangents")
        near_heading = math.degrees(math.atan(near[0]))
        far_heading = math.degrees(math.atan(far[0]))
        bend = far_heading - near_heading
        departure = sum(x - (near[0] * -y + near[1])
                        for x, y in parsed[-group:]) / group
        interior_departure = parsed[-2][0] - (near[0] * -parsed[-2][1] + near[1])
        result.update(near_heading_deg=near_heading, far_heading_deg=far_heading,
                      signed_bend_deg=bend, far_departure_px=departure,
                      interior_departure_px=interior_departure)
        fit = _quadratic(parsed)
        result["fit_rms_px"] = fit[-1]
        if fit[-1] > MAX_FIT_RMS_PIXELS:
            return reject("poor_path_fit")
        far_allowed = True
        if (abs(bend) <= BEND_START_DEGREES
                or abs(departure) <= MIN_FAR_DEPARTURE_PIXELS
                or abs(interior_departure) <= MIN_FAR_DEPARTURE_PIXELS):
            return reject("straight_path")
        if bend * departure <= 0.0 or bend * interior_departure <= 0.0:
            return reject("conflicting_geometry")
        # Independent point noise can accidentally give two groups different
        # headings. Require curvature supported by the whole path fit, above
        # its residual uncertainty, before adding to an otherwise stable PD.
        ts = [index - fit[3] for index in range(len(parsed))]
        s2 = sum(t * t for t in ts)
        curvature_information = sum(t ** 4 for t in ts) - s2 * s2 / len(parsed)
        residual_sigma = fit[-1] * math.sqrt(len(parsed) / (len(parsed) - 3.0))
        curvature_sigma = residual_sigma / math.sqrt(curvature_information)
        signal_sigma = abs(fit[0]) / max(curvature_sigma, 1e-9)
        result["curvature_signal_sigma"] = signal_sigma
        if signal_sigma < MIN_CURVATURE_SIGNAL_SIGMA:
            return reject("noisy_curvature")
        preview_y = max(PREVIEW_Y_START + command_blend * (PREVIEW_Y_FULL - PREVIEW_Y_START),
                        parsed[-2][1])
        result["preview_y"] = preview_y
        if preview_y >= float(base_control_y) - 1e-9:
            return reject("no_additional_preview")
        base = _path_point(parsed, float(base_control_y), fit, float(path_fit_blend))
        preview = _path_point(parsed, preview_y, fit, float(path_fit_blend))
        if base is None or preview is None:
            return reject("base_outside_observation")
        base_x = base[0] if base_control_x is None else float(base_control_x)
        base_heading = base[1] if base_heading_step is None else float(base_heading_step)
        delta = ((preview[0] - base_x) * float(center_gain)
                 + (preview[1] - base_heading) * float(heading_gain)) * float(gain_scale)
        if not _finite(delta):
            return reject("invalid_input")
        result["unbounded_delta_duty"] = delta
        if delta * bend <= 0.0:
            return reject("conflicting_correction")
        bend_blend = _clip((abs(bend) - BEND_START_DEGREES)
                           / (BEND_FULL_DEGREES - BEND_START_DEGREES), 0.0, 1.0)
        candidate = _clip(delta, -MAX_EXTRA_DUTY, MAX_EXTRA_DUTY) * command_blend * bend_blend
        result.update(candidate_duty=candidate, bend_blend=bend_blend,
                      reason="observed_bend", source="local_observation")
        return finish()

    def _finish(self, result, observation, dt, heading_gain, gain_scale):
        """Choose one preview source and apply one shared bound/filter."""
        local = result["candidate_duty"]
        remote = _far_candidate(observation, result["command_blend"], heading_gain, gain_scale)
        result["far_preview"] = remote
        candidate = local
        far_extra = remote["candidate_duty"]
        local_bend = result.get("signed_bend_deg") or 0.0
        # A distant opposite bend in an S must wait while the near path is
        # still strongly turning. Do not add opposing or stacked feedforward.
        opposite_current_corner = (abs(local_bend) >= BEND_FULL_DEGREES
                                   and local_bend * far_extra < 0.0)
        if opposite_current_corner:
            remote["reason"] = "opposes_current_corner"
        elif far_extra and (not local or local * far_extra > 0.0) and abs(far_extra) > abs(local):
            candidate = far_extra
            result.update(reason="far_observed_bend", source="upper_observation",
                          preview_y=remote["preview_y"])
        elif far_extra and local * far_extra < 0.0:
            remote["reason"] = "opposes_local_preview"
        result["candidate_duty"] = candidate
        if not candidate:
            self.reset()
            return result
        direction = 1 if candidate > 0.0 else -1
        if direction != self._direction:
            self._extra = 0.0
        self._direction = direction
        interval = float(dt) if _finite(dt) and float(dt) > 0.0 else 1.0 / 30.0
        # A delayed camera frame is not proof of a long stable bend.
        interval = _clip(interval, 0.001, 0.10)
        # A still-observed sharp turn may lose a few far rows as it enters
        # the near field. Ease the stronger term down over 120 ms instead of
        # abruptly removing it. No carry is permitted on ordinary curves,
        # missing/weak observations, a direction conflict, or an S-turn veto.
        release = (abs(self._extra) > MAX_EXTRA_DUTY
                   and abs(candidate) < abs(self._extra)
                   and result["source"] == "upper_observation"
                   and remote["valid"] and remote["sharp_blend"] > 0.0
                   and remote["signed_bend_deg"] * self._extra > 0.0
                   and abs(remote["signed_bend_deg"]) > SHARP_BEND_START_DEGREES
                   and abs(remote["far_departure_px"]) > SHARP_DEPARTURE_START_PIXELS)
        response = SHARP_RELEASE_SECONDS if release else RESPONSE_SECONDS
        alpha = 1.0 - math.exp(-interval / response)
        self._extra += alpha * (candidate - self._extra)
        # Command reductions retain their absolute envelope during release;
        # every non-sharp exit keeps the previous immediate candidate clamp.
        bound = SHARP_MAX_EXTRA_DUTY * result["command_blend"] if release else abs(candidate)
        self._extra = math.copysign(min(abs(self._extra), bound), candidate)
        result.update(active=abs(self._extra) > 1e-9, extra_duty=self._extra,
                      sharp_release_active=release, response_time_seconds=response)
        return result
