"""Image-space forward-output limits for corners, independent of speed boost.

This is an open-loop PWM cap, not a calibrated road-speed or braking controller.
Inputs are the detector's bird-view points in its 320 x 240 coordinate system.
The caller applies the result only after mode selection, for AUTO / Drive.
No hardware access, mode selection, motor stop, or reverse command occurs here.
"""
import math


MIN_FORWARD_PERCENT = 13.0
RELEASE_HOLD_SECONDS = 0.30
RELEASE_PERCENT_PER_SECOND = 30.0
# Preserve the previously driven output range. The extra geometry restriction
# is introduced continuously only as the driver requests more straight output.
PACING_START_PERCENT = 28.0
PACING_FULL_PERCENT = 38.0


def _clip(value, low, high):
    return max(low, min(high, value))


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def _cap_at(value, knots):
    """Continuous output-percent profile, including its unrestricted end."""
    if value <= knots[0][0]:
        return float(knots[0][1])
    for (x0, y0), (x1, y1) in zip(knots, knots[1:]):
        if value <= x1:
            ratio = (value - x0) / (x1 - x0)
            return y0 + ratio * (y1 - y0)
    return float(knots[-1][1])


CURVATURE_PROFILE = ((8., 100.), (12., 55.), (18., 32.), (24., 22.), (32., 13.))
HEADING_PROFILE = ((12., 100.), (20., 55.), (30., 35.), (45., 22.), (60., 13.))
LATERAL_PROFILE = ((20., 100.), (30., 45.), (40., 28.), (60., 18.), (80., 13.))
STEERING_PROFILE = ((.70, 100.), (1., 55.), (1.50, 32.), (2.10, 20.), (2.70, 13.))

# The extra preview policy is opt-in. Absence preserves the driven legacy
# contract; a supplied but invalid observation is never evidence of far clear.
FAR_BEND_PROFILE = ((8., 100.), (15., 60.), (25., 42.), (40., 30.), (60., 22.))
FAR_BEND_DEPARTURE_PX = 8.0
FAR_DISTANT_ONSET_Y = -160.0
FAR_NEAR_ONSET_Y = 0.0
VERIFIED_CLEAR_SECONDS = .12
VERIFIED_RELEASE_PERCENT_PER_SECOND = 90.0
LATERAL_RECOVERY_MAX_PX = 40.0
LATERAL_RECOVERY_ALLOWANCE_PX = 12.0
LATERAL_RECOVERY_SECONDS = .15
LATERAL_RECOVERY_MIN_PX_PER_SECOND = 4.0


def _number(value):
    return None if isinstance(value, bool) or not _finite(value) else float(value)


def _near_reference(points):
    ordered = sorted([(float(x), float(y)) for x, y in points], key=lambda p: -p[1])
    near = [point for point in ordered if point[1] >= 70.]
    if len(near) < 2:
        return None
    ym, xm = sum(y for x, y in near) / len(near), sum(x for x, y in near) / len(near)
    denominator = sum((y - ym) ** 2 for x, y in near)
    if denominator <= 1e-9:
        return None
    slope = sum((y - ym) * (x - xm) for x, y in near) / denominator
    return ordered, slope, xm - slope * ym


def _far_geometry(observation, points, metrics, confidence):
    result = dict(valid=False, reason='far_observation_unknown', bend=False,
                  clear_geometry=False, bend_onset_y=None,
                  visible_straight_span_y_px=None, proximity=0., angle_cap_percent=100.,
                  approach_cap_percent=100., tangent_delta_deg=None,
                  far_departure_px=None, termination=None)
    if (not isinstance(observation, dict) or observation.get('valid') is not True
            or not metrics['valid'] or metrics['point_count'] < 6
            or metrics['span_y_px'] < 100. or _number(confidence) is None
            or float(confidence) < .90):
        return result
    names = ('near_heading_deg', 'far_heading_deg', 'tangent_delta_deg',
             'far_departure_px', 'far_top_y', 'extra_span_y_px',
             'anchor_count', 'point_count')
    values = {name: _number(observation.get(name)) for name in names}
    if any(value is None for value in values.values()):
        return result
    if (values['anchor_count'] < max(4, metrics['point_count'] - 2)
            or values['point_count'] < 6 or values['extra_span_y_px'] < 100.
            or values['far_top_y'] > 0.
            or abs(values['far_heading_deg'] - values['near_heading_deg']
                   - values['tangent_delta_deg']) > .25):
        return result
    try:
        far_points = [(float(x), float(y)) for x, y in observation.get('points', [])
                      if _number(x) is not None and _number(y) is not None]
    except (TypeError, ValueError):
        return result
    if (len(far_points) != values['point_count']
            or any(not -160. <= x < 480. or not -200. <= y <= 100.
                   for x, y in far_points)
            or any(first[1] <= second[1] for first, second in zip(far_points, far_points[1:]))
            or not far_points or abs(far_points[-1][1] - values['far_top_y']) > 1.):
        return result
    reference = _near_reference(points)
    if reference is None:
        result['reason'] = 'near_reference_short'
        return result
    ordered, slope, intercept = reference
    actual_near = math.degrees(math.atan(-slope))
    actual_far = _heading(far_points[-7:], ordered[0][1])
    if actual_far is None:
        result['reason'] = 'far_reference_short'
        return result
    actual_departure = sum(x - slope * y - intercept for x, y in far_points[-3:]) / 3.
    if (abs(actual_near - values['near_heading_deg']) > 1.
            or abs(actual_far - values['far_heading_deg']) > 1.
            or abs(actual_departure - values['far_departure_px']) > 1.):
        result['reason'] = 'inconsistent_far_geometry'
        return result
    delta, departure = values['tangent_delta_deg'], values['far_departure_px']
    termination = observation.get('termination')
    result.update(valid=True, tangent_delta_deg=delta, far_departure_px=departure,
                  termination=termination if isinstance(termination, str) else None,
                  far_top_y=values['far_top_y'], point_count=len(far_points))
    departures = [x - slope * y - intercept for x, y in far_points]
    window_headings = [_heading(far_points[index:index + 5], ordered[0][1])
                       for index in range(len(far_points) - 4)]
    if any(heading is None for heading in window_headings):
        result.update(valid=False, reason='far_section_short')
        return result
    window_bends = [heading - actual_near for heading in window_headings]
    max_internal_departure = max(abs(value) for value in departures)
    max_internal_bend = max(abs(value) for value in window_bends)
    result.update(max_internal_departure_px=max_internal_departure,
                  max_internal_bend_deg=max_internal_bend)
    result['clear_geometry'] = (
        termination == 'far_limit' and values['far_top_y'] <= -180.
        and abs(delta) <= 5. and abs(departure) <= 8.
        and max_internal_departure <= 8. and max_internal_bend <= 8.
        and max(abs(heading) for heading in window_headings) <= 12.
        and max(abs(values['near_heading_deg']), abs(values['far_heading_deg'])) <= 12.
    )
    if result['clear_geometry']:
        result['reason'] = 'far_horizon_straight'
        return result
    # A path can turn and then return to its near tangent (an S curve).
    # Find its first supported departure rather than relying on the final
    # tangent or final three points, which can both be zero in that case.
    onset = None
    bend_delta = None
    for index in range(len(far_points) - 2):
        sign = math.copysign(1., departures[index])
        signed_departures = [sign * value for value in departures[index:index + 3]]
        matching_bends = [value for value in window_bends if sign * value > 8.]
        if (all(value >= FAR_BEND_DEPARTURE_PX for value in signed_departures)
                and matching_bends):
            bend_delta = max(matching_bends, key=abs)
            onset = far_points[index][1]
            previous_departure = sign * departures[index - 1] if index else None
            if index and previous_departure < FAR_BEND_DEPARTURE_PX:
                previous_y = far_points[index - 1][1]
                fraction = ((FAR_BEND_DEPARTURE_PX - previous_departure)
                            / (sign * departures[index] - previous_departure))
                onset = previous_y + fraction * (onset - previous_y)
            break
    if onset is None:
        result['reason'] = 'far_bend_onset_unconfirmed'
        return result
    angle_cap = _cap_at(abs(bend_delta), FAR_BEND_PROFILE)
    proximity = _clip((onset - FAR_DISTANT_ONSET_Y)
                      / (FAR_NEAR_ONSET_Y - FAR_DISTANT_ONSET_Y), 0., 1.)
    result.update(bend=True, reason='connected_far_bend', bend_delta_deg=bend_delta,
                  bend_onset_y=onset,
                  visible_straight_span_y_px=ordered[0][1] - onset,
                  proximity=proximity, angle_cap_percent=angle_cap,
                  approach_cap_percent=100. + proximity * (angle_cap - 100.))
    return result


def _heading(points, origin_y):
    ts = [origin_y - y for x, y in points]
    xs = [x for x, y in points]
    tm, xm = sum(ts) / len(ts), sum(xs) / len(xs)
    denominator = sum((t - tm) ** 2 for t in ts)
    if denominator <= 1e-9:
        return None
    slope = sum((t - tm) * (x - xm) for t, x in zip(ts, xs)) / denominator
    return math.degrees(math.atan(slope))


def path_geometry(points, width=320.0, height=240.0):
    """Separate turning shape from a straight line seen diagonally.

    Two local least-squares headings suppress individual point noise. Their
    difference represents visible bending; their absolute angle is a separate
    alignment signal. A diagonal straight therefore has zero bending.
    """
    metrics = dict(valid=False, point_count=0, span_y_px=0.,
                   near_heading_deg=None, far_heading_deg=None,
                   tangent_delta_deg=None, near_lateral_px=None)
    try:
        raw = list(points)
    except TypeError:
        return metrics
    parsed = []
    for point in raw:
        try:
            x, y = point
        except (TypeError, ValueError):
            return metrics
        if not _finite(x) or not _finite(y):
            return metrics
        x, y = float(x), float(y)
        if not -1. <= x <= width + 1. or not -1. <= y <= height + 1.:
            return metrics
        parsed.append((x, y))
    # Detector samples have unique y values. Duplicate rows are not additional
    # evidence and would otherwise make a short path appear well supported.
    if len({y for x, y in parsed}) != len(parsed):
        return metrics
    parsed.sort(key=lambda point: -point[1])
    metrics['point_count'] = len(parsed)
    if len(parsed) < 5:
        return metrics
    span = parsed[0][1] - parsed[-1][1]
    metrics['span_y_px'] = span
    if span < 60.:
        return metrics
    group_size = max(3, (len(parsed) + 1) // 2)
    near = _heading(parsed[:group_size], parsed[0][1])
    far = _heading(parsed[-group_size:], parsed[0][1])
    if near is None or far is None:
        return metrics
    metrics.update(valid=True, near_heading_deg=near, far_heading_deg=far,
                   tangent_delta_deg=abs(far - near),
                   near_lateral_px=abs(parsed[0][0] - width / 2.))
    return metrics


class CurvePace:
    """Immediate corner entry cap with a stable, rate-limited corner exit.

    Geometric caps are independent of the driver's requested output. On a clear
    straight the driver's full setting remains available. Lower caps take
    effect on the first supported observation, before a steering limit is hit.
    A cap only relaxes after 0.30 seconds without another restrictive sample.
    """
    def __init__(self, minimum_percent=MIN_FORWARD_PERCENT,
                 release_hold=RELEASE_HOLD_SECONDS,
                 release_rate=RELEASE_PERCENT_PER_SECOND):
        if not all(_finite(x) for x in (minimum_percent, release_hold, release_rate)):
            raise ValueError('CurvePace configuration must be finite')
        if not 0. < float(minimum_percent) <= 100.:
            raise ValueError('minimum_percent must be in (0, 100]')
        if float(release_hold) < 0. or float(release_rate) <= 0.:
            raise ValueError('release hold/rate must be nonnegative/positive')
        self.minimum_percent = float(minimum_percent)
        self.release_hold = float(release_hold)
        self.release_rate = float(release_rate)
        self.reset()

    def reset(self):
        self._cap = 100.
        self._clear_time = 0.
        self._policy_kind = None
        self._preview_cap = None
        self._verified_time = 0.
        self._verified_samples = 0
        self._previous_lateral = None
        self._shrinking_time = 0.
        self._shrinking_samples = 0
        self._far_envelope = None
        self._far_sign = None

    def update(self, points, confidence, perception_mode, dt, requested_percent,
               steering_duty=10., neutral=10., far_observation=None):
        kind = 'legacy' if far_observation is None else 'far_preview'
        if self._policy_kind is not None and self._policy_kind != kind:
            self.reset()
        self._policy_kind = kind
        if far_observation is None:
            return self._update_legacy(points, confidence, perception_mode, dt,
                                       requested_percent, steering_duty, neutral)
        return self._update_preview(points, confidence, perception_mode, dt,
                                    requested_percent, steering_duty, neutral,
                                    far_observation)

    def _update_legacy(self, points, confidence, perception_mode, dt, requested_percent,
                       steering_duty=10., neutral=10.):
        requested_valid = _finite(requested_percent)
        requested = _clip(float(requested_percent), 0., 100.) if requested_valid else self.minimum_percent
        dt = float(dt) if _finite(dt) and float(dt) > 0. else 1. / 30.
        dt = min(dt, .30)  # One delayed frame cannot count as a stable long exit.
        metrics = path_geometry(points)
        reliable = (metrics['valid'] and perception_mode == 'TRACK'
                    and _finite(confidence) and float(confidence) >= .62
                    and _finite(steering_duty) and _finite(neutral)
                    and requested_valid)
        if not reliable:
            target = self.minimum_percent
            reason = 'unreliable_line'
            metrics['steering_offset_duty'] = None
        else:
            steering = abs(float(steering_duty) - float(neutral))
            metrics['steering_offset_duty'] = steering
            candidates = [
                (_cap_at(metrics['tangent_delta_deg'], CURVATURE_PROFILE), 'visible_bend'),
                (_cap_at(max(abs(metrics['near_heading_deg']), abs(metrics['far_heading_deg'])), HEADING_PROFILE), 'line_heading'),
                (_cap_at(metrics['near_lateral_px'], LATERAL_PROFILE), 'lateral_error'),
                (_cap_at(steering, STEERING_PROFILE), 'steering_margin'),
            ]
            target, reason = min(candidates, key=lambda item: item[0])
            target = max(self.minimum_percent, target)
            if target >= 100. - 1e-9:
                reason = 'clear_straight'
        if target <= self._cap + 1e-9:
            self._cap = target
            self._clear_time = 0.
        else:
            self._clear_time += dt
            # Count only the portion after the clear-observation hold. This
            # makes the recovery rate independent of camera/control cadence.
            release_dt = min(dt, max(0., self._clear_time - self.release_hold))
            self._cap = min(target, self._cap + self.release_rate * release_dt)
        if not reliable:
            phase = 'RECOVERY'
        elif target < 100. - 1e-9:
            phase = 'APPROACH' if (reason == 'visible_bend'
                                  and abs(metrics['near_heading_deg']) < 20.) else 'CORNER'
        elif self._cap < 100. - 1e-9:
            phase = 'EXIT'
            reason = 'corner_exit_hold' if self._clear_time <= self.release_hold else 'corner_exit_ramp'
        else:
            phase = 'STRAIGHT'
        policy_blend = (_clip((requested - PACING_START_PERCENT)
                             / (PACING_FULL_PERCENT - PACING_START_PERCENT), 0., 1.)
                        if requested_valid else 1.)
        restricted_cap = min(requested, self._cap)
        cap = requested + policy_blend * (restricted_cap - requested)
        if policy_blend == 0.:
            phase, reason = 'BASELINE', 'baseline_output_range'
        return dict(cap_percent=cap, policy_blend=policy_blend,
                    geometry_cap_percent=target, release_cap_percent=self._cap,
                    risk=_clip((100. - target) / (100. - self.minimum_percent), 0., 1.)
                         if self.minimum_percent < 100. else 0.,
                    phase=phase, reason=reason, requested_percent=requested,
                    clear_time_seconds=self._clear_time, metrics=metrics)

    def _update_preview(self, points, confidence, perception_mode, dt, requested_percent,
                        steering_duty, neutral, far_observation):
        requested_valid = _number(requested_percent) is not None
        requested = (_clip(float(requested_percent), 0., 100.) if requested_valid
                     else self.minimum_percent)
        dt_valid = _number(dt) is not None and float(dt) > 0.
        step = min(float(dt), .30) if dt_valid else 0.
        metrics = path_geometry(points)
        reliable = (metrics['valid'] and perception_mode == 'TRACK'
                    and _number(confidence) is not None and float(confidence) >= .62
                    and _number(steering_duty) is not None
                    and _number(neutral) is not None and requested_valid)
        far = _far_geometry(far_observation, points, metrics, confidence) if reliable else {
            'valid': False, 'reason': 'unreliable_line', 'bend': False,
            'clear_geometry': False, 'approach_cap_percent': 100.,
            'bend_onset_y': None, 'visible_straight_span_y_px': None,
        }
        steering = abs(float(steering_duty) - float(neutral)) if reliable else None
        metrics['steering_offset_duty'] = steering
        geometry_clear = (reliable and far['clear_geometry']
                          and metrics['tangent_delta_deg'] <= 5.
                          and max(abs(metrics['near_heading_deg']),
                                  abs(metrics['far_heading_deg'])) <= 12.
                          and steering <= .70)
        if geometry_clear and dt_valid:
            self._verified_time += step
            self._verified_samples += 1
        else:
            self._verified_time = 0.
            self._verified_samples = 0
        shrink_rate = None
        lateral = metrics['near_lateral_px'] if reliable else None
        if geometry_clear and dt_valid:
            if self._previous_lateral is not None:
                shrink_rate = (self._previous_lateral - lateral) / step
            if (20. < lateral <= LATERAL_RECOVERY_MAX_PX
                    and shrink_rate is not None
                    and shrink_rate >= LATERAL_RECOVERY_MIN_PX_PER_SECOND):
                self._shrinking_time += step
                self._shrinking_samples += 1
            else:
                self._shrinking_time = 0.
                self._shrinking_samples = 0
            self._previous_lateral = lateral
        else:
            self._previous_lateral = None
            self._shrinking_time = 0.
            self._shrinking_samples = 0
        lateral_recovery = (geometry_clear and self._shrinking_samples >= 2
                            and self._shrinking_time + 1e-9 >= LATERAL_RECOVERY_SECONDS)
        effective_lateral = (max(0., lateral - LATERAL_RECOVERY_ALLOWANCE_PX)
                             if lateral_recovery else lateral)
        metrics.update(effective_lateral_px=effective_lateral,
                       lateral_shrink_rate_px_per_second=shrink_rate)
        horizon_confirmed = (self._verified_samples >= 2
                             and self._verified_time + 1e-9 >= VERIFIED_CLEAR_SECONDS)
        verified_straight = (geometry_clear and horizon_confirmed
                             and (lateral <= 20. or lateral_recovery))
        envelope_held = False
        if far['bend']:
            sign = math.copysign(1., far['bend_delta_deg'])
            approach_cap = max(self.minimum_percent, far['approach_cap_percent'])
            if self._far_sign == sign and self._far_envelope is not None:
                envelope_held = approach_cap > self._far_envelope + 1e-9
                self._far_envelope = min(self._far_envelope, approach_cap)
            else:
                # A newly observed opposite bend starts its own envelope.
                # The final output rise limit still prevents an output jump.
                self._far_envelope = approach_cap
            self._far_sign = sign
        elif geometry_clear:
            if horizon_confirmed:
                self._far_envelope = None
                self._far_sign = None
        else:
            # Unknown far geometry is not clear evidence, and does not keep an
            # obsolete far-bend envelope indefinitely. Legacy release pacing
            # remains in force, based on the supported near-road observation.
            self._far_envelope = None
            self._far_sign = None
        if not reliable:
            target, reason = self.minimum_percent, 'unreliable_line'
        else:
            candidates = [
                (_cap_at(metrics['tangent_delta_deg'], CURVATURE_PROFILE), 'visible_bend'),
                (_cap_at(max(abs(metrics['near_heading_deg']),
                             abs(metrics['far_heading_deg'])), HEADING_PROFILE), 'line_heading'),
                (_cap_at(effective_lateral, LATERAL_PROFILE),
                 'recovering_lateral_error' if lateral_recovery else 'lateral_error'),
                (_cap_at(steering, STEERING_PROFILE), 'steering_margin'),
            ]
            if self._far_envelope is not None:
                candidates.append((self._far_envelope,
                                   'far_bend_envelope' if envelope_held else
                                   'far_bend_approach' if far['bend'] else
                                   'far_straight_confirmation'))
            target, reason = min(candidates, key=lambda item: item[0])
            target = max(self.minimum_percent, target)
            if target >= 100. - 1e-9:
                reason = 'verified_far_straight' if verified_straight else 'clear_near_road'
        policy_blend = (_clip((requested - PACING_START_PERCENT)
                             / (PACING_FULL_PERCENT - PACING_START_PERCENT), 0., 1.)
                        if requested_valid else 1.)
        desired = requested + policy_blend * (min(requested, target) - requested)
        startup = self._preview_cap is None
        rate = (VERIFIED_RELEASE_PERCENT_PER_SECOND if verified_straight
                else self.release_rate)
        release_state = 'steady'
        if policy_blend == 0.:
            # This range is deliberately identical to the previously driven
            # behavior. It also seeds a bounded later request increase.
            cap = requested
            self._clear_time = 0.
        elif startup:
            cap = min(desired, PACING_START_PERCENT)
            self._clear_time = 0.
            release_state = 'startup_limit' if cap + 1e-9 < desired else 'steady'
        elif desired <= self._preview_cap + 1e-9:
            cap = desired
            self._clear_time = 0.
        else:
            self._clear_time = self._clear_time + step if dt_valid else 0.
            if verified_straight:
                release_dt = min(step, max(0., self._verified_time - VERIFIED_CLEAR_SECONDS))
                release_state = 'verified_straight_ramp'
            else:
                release_dt = min(step, max(0., self._clear_time - self.release_hold))
                release_state = 'near_road_ramp' if release_dt else 'release_hold'
            cap = min(desired, self._preview_cap + rate * release_dt)
        self._preview_cap = cap
        rise_limited = cap + 1e-9 < desired
        if policy_blend == 0.:
            phase, reason = 'BASELINE', 'baseline_output_range'
        elif not reliable:
            phase = 'RECOVERY'
        elif target < 100. - 1e-9:
            phase = 'APPROACH' if reason.startswith('far_') or reason == 'visible_bend' else 'CORNER'
        elif rise_limited:
            phase = 'EXIT'
            reason = ('startup_rise_limit' if startup else
                      'verified_straight_release' if verified_straight else
                      'corner_exit_hold' if release_state == 'release_hold' else
                      'corner_exit_ramp')
        else:
            phase = 'STRAIGHT'
        return dict(cap_percent=cap, policy_blend=policy_blend,
                    geometry_cap_percent=target, release_cap_percent=cap,
                    risk=_clip((100. - target) / (100. - self.minimum_percent), 0., 1.)
                         if self.minimum_percent < 100. else 0.,
                    phase=phase, reason=reason, requested_percent=requested,
                    clear_time_seconds=self._clear_time, metrics=metrics,
                    preview_policy='far_horizon_release_v1', far_geometry=far,
                    far_envelope_cap_percent=self._far_envelope,
                    verified_straight=verified_straight,
                    verified_straight_time_seconds=self._verified_time,
                    lateral_recovery_active=lateral_recovery,
                    lateral_recovery_time_seconds=self._shrinking_time,
                    release_rate_percent_per_second=rate, release_state=release_state,
                    output_rise_limited=rise_limited, startup_limited=startup and rise_limited,
                    dt_valid=dt_valid)


def apply_forward_cap(duty, cap_percent, neutral, forward):
    """Reduce an existing forward command, preserving neutral and reverse.

    Only the caller knows the selected vehicle mode. It must call this after
    ControlState.apply_and_snapshot, only for AUTO / D, so neither speed boost
    nor the launch-output floor can overwrite the result. Never call this for
    MANUAL, STEER_AUTO, N gear, emergency stop, or STOP mode.
    """
    if not all(_finite(value) for value in (duty, cap_percent, neutral, forward)):
        raise ValueError('forward cap inputs must be finite')
    duty, neutral, forward = float(duty), float(neutral), float(forward)
    if forward >= neutral:
        raise ValueError('forward duty must be lower than neutral duty')
    if duty >= neutral:
        return duty
    cap = _clip(float(cap_percent), 0., 100.)
    return max(duty, neutral - (neutral - forward) * cap / 100.)
