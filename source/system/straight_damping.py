"""Bounded position/heading balance on a well-supported visible straight.

This modifies tracking contributions, not their timing. It does not filter the
steering command, change derivative feedback, control speed, or access hardware.
The caller retains the old baseline for pacing and adds ``delta_duty`` only to
the automatic steering target before its existing output limits/smoothing.
Image-space straight evidence is not a measurement of vehicle yaw or distance.
"""
import math

from curve_pace import path_geometry, _far_geometry


MIN_CONFIDENCE = .90
MIN_BASE_POINTS = 6
MIN_BASE_SPAN_Y = 100.
MIN_ANCHORS = 6
MIN_FAR_POINTS = 20
MAX_FAR_TOP_Y = -180.
MAX_LOCAL_BEND_DEG = 5.
MAX_LOCAL_HEADING_DEG = 12.
MAX_PREVIEW_EXTRA_DUTY = .02
AUTHORITY_SECONDS = .12
POSITION_SCALE = .85
OPPOSING_HEADING_FACTOR = 1.18
MAX_POSITION_REDUCTION_DUTY = .25
MAX_EXTRA_HEADING_DUTY = .12


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _clip(value, low, high):
    return max(low, min(high, value))


class StraightDamping:
    """Keep existing targets exact unless repeated straight evidence agrees.

    Two consecutive eligible observations confirm the gain adjustment. Authority
    grows over 120 ms of eligible observations. Loss of evidence immediately
    returns the existing controller unchanged; no stale straight adjustment can
    enter a corner, a genuine S bend, LOST, disabled/manual operation, or a new
    tracking session. No history of position/heading values is filtered here.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self._eligible_seconds = 0.
        self._eligible_samples = 0

    def update(self, position_term, heading_term, derivative_term, confidence,
               perception_mode, dt, *, points, far_observation,
               path_metrics=None, far_geometry=None, preview_extra=0., enabled=True):
        """Return adjusted P/H terms and a delta; every term is in duty points.

        Optional geometry dictionaries must describe the same current points
        and far observation; callers may reuse a current ``curve_pace`` result.
        Without them the existing pure validators compute the same evidence.
        Invalid/unconfirmed evidence is always an exact target bypass.
        """
        result = dict(active=False, eligible=False, reason='disabled',
                      strength=0., delta_duty=0., position_term=position_term,
                      heading_term=heading_term, derivative_term=derivative_term,
                      position_scale=1., position_reduction_duty=0.,
                      extra_heading_duty=0., opposing_heading=False,
                      eligible_seconds=0., eligible_samples=0,
                      geometry_reason=None)

        def bypass(reason):
            self.reset()
            result['reason'] = reason
            return result

        if not enabled:
            return bypass('disabled')
        parsed = [_number(v) for v in (position_term, heading_term, derivative_term,
                                       confidence, dt, preview_extra)]
        if any(v is None for v in parsed) or parsed[4] <= 0. or parsed[4] > .30:
            return bypass('invalid_input')
        position, heading, derivative, confidence_value, step, preview = parsed
        if perception_mode != 'TRACK' or confidence_value < MIN_CONFIDENCE:
            return bypass('unreliable_line')
        if abs(preview) > MAX_PREVIEW_EXTRA_DUTY:
            return bypass('curve_preview_active')
        try:
            points = list(points)
            metrics = path_geometry(points) if path_metrics is None else path_metrics
            if (not isinstance(metrics, dict) or metrics.get('valid') is not True
                    or _number(metrics.get('point_count')) is None
                    or metrics['point_count'] < MIN_BASE_POINTS
                    or _number(metrics.get('span_y_px')) is None
                    or metrics['span_y_px'] < MIN_BASE_SPAN_Y):
                return bypass('local_support_short')
            angles = [_number(metrics.get(key)) for key in
                      ('tangent_delta_deg', 'near_heading_deg', 'far_heading_deg')]
            if any(v is None for v in angles):
                return bypass('invalid_geometry')
            if abs(angles[0]) > MAX_LOCAL_BEND_DEG or max(abs(angles[1]), abs(angles[2])) > MAX_LOCAL_HEADING_DEG:
                return bypass('local_curve_or_alignment')
            if not isinstance(far_observation, dict) or far_observation.get('valid') is not True:
                return bypass('far_unavailable')
            support = [_number(far_observation.get(key)) for key in
                       ('anchor_count', 'point_count', 'far_top_y')]
            if (any(v is None for v in support) or support[0] < MIN_ANCHORS
                    or support[1] < MIN_FAR_POINTS or support[2] > MAX_FAR_TOP_Y
                    or far_observation.get('termination') != 'far_limit'):
                return bypass('far_support_short')
            far = (_far_geometry(far_observation, points, metrics, confidence_value)
                   if far_geometry is None else far_geometry)
            result['geometry_reason'] = far.get('reason') if isinstance(far, dict) else None
            if (not isinstance(far, dict) or far.get('valid') is not True
                    or far.get('clear_geometry') is not True
                    or far.get('termination') != 'far_limit'
                    or _number(far.get('far_top_y')) is None
                    or far['far_top_y'] > MAX_FAR_TOP_Y):
                return bypass('far_curve_or_unconfirmed')
        except (TypeError, ValueError, KeyError, ZeroDivisionError, OverflowError):
            return bypass('invalid_geometry')

        self._eligible_samples += 1
        self._eligible_seconds += min(step, .10)
        strength = (_clip(self._eligible_seconds / AUTHORITY_SECONDS, 0., 1.)
                    if self._eligible_samples >= 2 else 0.)
        reduction = _clip(position * (1. - POSITION_SCALE) * strength,
                          -MAX_POSITION_REDUCTION_DUTY * strength,
                          MAX_POSITION_REDUCTION_DUTY * strength)
        opposing = position * heading < 0.
        extra = (_clip(heading * (OPPOSING_HEADING_FACTOR - 1.) * strength,
                       -MAX_EXTRA_HEADING_DUTY * strength,
                       MAX_EXTRA_HEADING_DUTY * strength) if opposing else 0.)
        effective_position = position - reduction
        result.update(active=strength > 0., eligible=True,
                      reason=('confirming_straight' if strength == 0. else
                              'straight_gain_ramp' if strength < 1. else 'verified_straight_balance'),
                      strength=strength, delta_duty=-reduction + extra,
                      position_term=effective_position, heading_term=heading + extra,
                      derivative_term=derivative,
                      position_scale=effective_position / position if position else 1.,
                      position_reduction_duty=reduction, extra_heading_duty=extra,
                      opposing_heading=opposing, eligible_seconds=self._eligible_seconds,
                      eligible_samples=self._eligible_samples)
        return result
