"""Small opposing-D headroom only during a verified straight-line return.

The existing P, heading and derivative terms remain unchanged. This helper
adds only a bounded portion of derivative authority that the existing filter
headroom clamp removed. It never reverses the existing net steering target,
filters a target, controls speed, or accesses hardware. Image motion is not a
measurement of physical vehicle yaw or cross-track velocity.
"""
import math
from curve_pace import path_geometry, _far_geometry

MAX_EXTRA_DUTY = .10
MAX_POSITION_FRACTION = .25
MAX_NET_FRACTION = .50
FIXED_NEAR_Y = 190.
MAX_CONTROL_Y_CHANGE = .5
MIN_DT = .005
MAX_DT = .15
OFFSET_FULL_PX = 30.
OFFSET_ZERO_PX = 50.
MOTION_ZERO_PX_S = 15.
MOTION_FULL_PX_S = 45.
CROSS_TIME_FULL_S = .35
CROSS_TIME_ZERO_S = .80
MAX_PREVIEW_EXTRA = .02
HEADING_FULL_DUTY = .05


def _number(value):
    if isinstance(value, bool):return None
    try:result = float(value)
    except (TypeError, ValueError, OverflowError):return None
    return result if math.isfinite(result) else None


def _weight(value, low, high):
    """Continuous smoothstep, with zero slope at the authority boundaries."""
    u = max(0., min(1., (value - low) / (high - low)))
    return u * u * (3. - 2. * u)


def _fixed_near_error(points):
    ordered = sorted(((float(x), float(y)) for x, y in points), key=lambda p: -p[1])
    if not ordered or ordered[0][1] < FIXED_NEAR_Y or ordered[-1][1] > FIXED_NEAR_Y:
        return None
    for x, y in ordered:
        if y == FIXED_NEAR_Y:return x - 160.
    for (x0, y0), (x1, y1) in zip(ordered, ordered[1:]):
        if y0 > FIXED_NEAR_Y > y1:
            return x0 + (x1 - x0) * (y0 - FIXED_NEAR_Y) / (y0 - y1) - 160.
    return None


class ReturnDamping:
    """Track one current geometric observation; never retain extra authority.

    ``dt`` is elapsed time since the preceding observation, not a nominal
    camera period. A skipped/gapped or unreliable observation resets history.
    Optional geometry caches must describe the same current points/far trail.
    """
    def __init__(self):self.reset()

    def reset(self):self._previous = None

    def update(self, raw_error, position_term, heading_term, derivative_term,
               unbounded_derivative_term, confidence, perception_mode, dt, *,
               points, far_observation, control_y, preview_extra=0., enabled=True,
               path_metrics=None, far_geometry=None, frame_index=None):
        """All P/H/D contributions, including unbounded D, include gain_scale.

        The caller adds only ``delta_duty`` to its unchanged baseline target.
        Setting ``enabled=False`` is an exact bypass for manual/disabled paths.
        """
        result = dict(active=False, eligible=False, reason='disabled', delta_duty=0.,
                      position_term=position_term, heading_term=heading_term,
                      derivative_term=derivative_term, unbounded_derivative_term=unbounded_derivative_term,
                      near_error=None, control_error=raw_error, near_y=FIXED_NEAR_Y,
                      control_y=control_y, near_velocity_px_s=None, control_velocity_px_s=None,
                      near_time_to_zero_s=None, shortfall_duty=0., net_target_duty=None,
                      motion_weight=0., offset_weight=0., crossing_weight=0., heading_weight=0., strength=0.,
                      extra_bound_duty=0., geometry_reason=None)
        def bypass(reason, remember=None):
            self._previous = remember
            result['reason'] = reason
            return result
        if not enabled:return bypass('disabled')
        values = [_number(v) for v in (raw_error, position_term, heading_term, derivative_term,
                                       unbounded_derivative_term, confidence, dt, control_y, preview_extra)]
        if any(v is None for v in values):return bypass('invalid_input')
        raw, position, heading, derivative, unbounded, confidence, step, cy, preview = values
        index = _number(frame_index) if frame_index is not None else None
        if frame_index is not None and (index is None or index < 0 or index != int(index)):
            return bypass('invalid_frame_index')
        if not MIN_DT <= step <= MAX_DT:return bypass('history_gap')
        if perception_mode != 'TRACK' or confidence < .90:return bypass('unreliable_line')
        if abs(preview) > MAX_PREVIEW_EXTRA:return bypass('curve_preview_active')
        try:
            points = list(points)
            metrics = path_geometry(points) if path_metrics is None else path_metrics
            if (not isinstance(metrics, dict) or metrics.get('valid') is not True
                    or (_number(metrics.get('point_count')) or 0) < 6
                    or (_number(metrics.get('span_y_px')) or 0) < 100.):
                return bypass('local_support_short')
            angles = [_number(metrics.get(k)) for k in
                      ('tangent_delta_deg', 'near_heading_deg', 'far_heading_deg')]
            if any(v is None for v in angles):return bypass('invalid_geometry')
            if abs(angles[0]) > 5. or max(abs(angles[1]),abs(angles[2])) > 12.:
                return bypass('local_curve_or_alignment')
            if not isinstance(far_observation, dict) or far_observation.get('valid') is not True:
                return bypass('far_unavailable')
            support = [_number(far_observation.get(k)) for k in ('anchor_count','point_count','far_top_y')]
            if (any(v is None for v in support) or support[0] < 6 or support[1] < 20
                    or support[2] > -180. or far_observation.get('termination') != 'far_limit'):
                return bypass('far_support_short')
            far = (_far_geometry(far_observation, points, metrics, confidence)
                   if far_geometry is None else far_geometry)
            result['geometry_reason'] = far.get('reason') if isinstance(far,dict) else None
            if not isinstance(far,dict) or far.get('valid') is not True or far.get('clear_geometry') is not True:
                return bypass('far_curve_or_unconfirmed')
            near = _fixed_near_error(points)
            if near is None:return bypass('near_row_unsupported')
        except (TypeError,ValueError,KeyError,ZeroDivisionError,OverflowError):
            return bypass('invalid_geometry')
        result['near_error'] = near
        current = (raw, near, cy, index)
        previous = self._previous
        # Only valid current straight evidence can seed the next comparison.
        self._previous = current
        if previous is None:return bypass('seed_history',current)
        prior_raw, prior_near, prior_y, prior_index = previous
        if index is not None and (prior_index is None or index != prior_index + 1):
            return bypass('frame_gap',current)
        if abs(cy - prior_y) > MAX_CONTROL_Y_CHANGE:return bypass('control_point_changed',current)
        if not (raw * prior_raw > 0. and near * prior_near > 0. and raw * near > 0.
                and prior_raw * prior_near > 0. and raw * position > 0.):
            return bypass('crossing_or_position_sign_conflict',current)
        raw_velocity = (raw - prior_raw) / step
        near_velocity = (near - prior_near) / step
        result.update(near_velocity_px_s=near_velocity,control_velocity_px_s=raw_velocity)
        if raw_velocity * raw >= 0. or near_velocity * near >= 0.:
            return bypass('not_converging',current)
        net = position + heading + derivative + preview
        result['net_target_duty'] = net
        # Already countersteering targets are never strengthened by this helper.
        if net * position <= 0. or abs(net) <= 1e-12:return bypass('already_countersteering',current)
        shortfall = unbounded - derivative
        result['shortfall_duty'] = shortfall
        if (shortfall * position >= 0. or unbounded * position >= 0.
                or derivative * position > 0. or heading * position >= 0.
                or raw_velocity * shortfall <= 0. or near_velocity * shortfall <= 0.):
            return bypass('feedback_direction_conflict',current)
        offset_weight = 1. - _weight(max(abs(raw),abs(near)),OFFSET_FULL_PX,OFFSET_ZERO_PX)
        if offset_weight <= 0.:return bypass('offset_large',current)
        motion_weight = min(_weight(abs(raw_velocity),MOTION_ZERO_PX_S,MOTION_FULL_PX_S),
                            _weight(abs(near_velocity),MOTION_ZERO_PX_S,MOTION_FULL_PX_S))
        if motion_weight <= 0.:return bypass('motion_below_noise',current)
        crossing_time = abs(near / near_velocity)
        crossing_weight = 1. - _weight(crossing_time,CROSS_TIME_FULL_S,CROSS_TIME_ZERO_S)
        result.update(near_time_to_zero_s=crossing_time,motion_weight=motion_weight,
                      offset_weight=offset_weight,crossing_weight=crossing_weight)
        if crossing_weight <= 0.:return bypass('crossing_not_imminent',current)
        heading_weight = _weight(abs(heading),0.,HEADING_FULL_DUTY)
        strength = offset_weight * motion_weight * crossing_weight * heading_weight
        bound = min(MAX_EXTRA_DUTY, MAX_POSITION_FRACTION * abs(position), MAX_NET_FRACTION * abs(net))
        extra = math.copysign(min(abs(shortfall),bound) * strength,shortfall)
        result.update(active=abs(extra)>1e-12,eligible=True,reason='bounded_converging_return',
                      delta_duty=extra,strength=strength,extra_bound_duty=bound,heading_weight=heading_weight)
        return result
