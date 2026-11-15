"""Adaptive white evidence only; no hardware, mode changes, or tracker policy."""
import cv2
import numpy as np


def _white_colour_evidence(frame, hsv, lightness, roi_top, roi_bottom):
    """Recognise a cast white against a strongly coloured, measured floor.

    The course capture has white BGR=(255,232,130), HSV saturation=125,
    against cyan BGR=(199,158,1), saturation=254. An absolute S<=96 gate
    deletes that line. A cast white instead raises the floor's weakest colour
    channel and is substantially less saturated while retaining its cast hue.
    A coherent, strongly saturated floor is required before relaxing the gate;
    warm textured barriers must not establish a new arbitrary white colour.
    """
    allowed = hsv[:, :, 1] <= 96
    sample_top = roi_top + (roi_bottom - roi_top) // 2
    sample = hsv[sample_top:roi_bottom]
    good = (sample[:, :, 1] >= 160) & (sample[:, :, 2] >= 40)
    info = {'colour_floor_hue': None, 'colour_floor_saturation': None,
            'colour_floor_support': 0.0, 'cast_white_enabled': False}
    if np.count_nonzero(good) < sample.shape[0] * sample.shape[1] * .15:
        return allowed.astype(np.uint8) * 255, info
    hist = np.bincount(sample[:, :, 0][good], minlength=180)
    # Circular bins keep a red cast coherent across the 179/0 boundary.
    smoothed = sum(np.roll(hist, shift) for shift in range(-3, 4))
    hue = int(np.argmax(smoothed))
    distance = np.abs(sample[:, :, 0].astype(np.int16) - hue)
    near = np.minimum(distance, 180 - distance) <= 10
    support = float(np.mean(near))
    floor_sat = float(np.median(sample[:, :, 1][near]))
    info.update(colour_floor_hue=hue, colour_floor_saturation=floor_sat,
                colour_floor_support=support)
    if support < .35 or floor_sat < 192:
        return allowed.astype(np.uint8) * 255, info
    weakest = np.min(frame, axis=2)
    floor_weakest = float(np.median(weakest[sample_top:roi_bottom][near]))
    floor_light = float(np.median(lightness[sample_top:roi_bottom][near]))
    distance = np.abs(hsv[:, :, 0].astype(np.int16) - hue)
    hue_distance = np.minimum(distance, 180 - distance)
    cast_white = ((hsv[:, :, 1] <= floor_sat - 40.) &
                  (hue_distance <= 16) &
                  (weakest >= floor_weakest + 12.) &
                  (lightness >= floor_light + 12.))
    allowed |= cast_white
    info.update(cast_white_enabled=True, colour_floor_weakest=floor_weakest,
                colour_floor_lightness=floor_light)
    return allowed.astype(np.uint8) * 255, info


def _fill_interiors(lightness, seed, minimum, delta, roi_top, roi_bottom, max_width):
    occupied = seed[roi_top:roi_bottom] != 0
    padded = np.pad(occupied, ((0, 0), (1, 1)))
    ys, starts = np.nonzero(padded[:, 1:] & ~padded[:, :-1])
    _, ends = np.nonzero(padded[:, :-1] & ~padded[:, 1:])
    if len(starts) < 2:
        return seed
    left = np.flatnonzero((ys[:-1] == ys[1:]) &
                          (ends[1:] - starts[:-1] <= max_width * 2) &
                          (starts[1:] > ends[:-1]))
    if not len(left):
        return seed
    light = lightness[roi_top:roi_bottom]
    width = light.shape[1]
    begin = ys[left] * width + ends[left]
    finish = ys[left + 1] * width + starts[left + 1]
    gap_min = np.minimum.reduceat(light.ravel(), np.column_stack((begin, finish)).ravel())[::2]
    lengths = ends - starts
    ids = np.repeat(np.arange(len(starts)), lengths)
    values = light[occupied]
    ordered = values[np.lexsort((values, ids))].astype(np.float32)
    offsets = np.cumsum(lengths) - lengths
    medians = (ordered[offsets + (lengths - 1) // 2] + ordered[offsets + lengths // 2]) * .5
    edge = np.minimum(medians[left], medians[left + 1])
    valid = gap_min >= np.maximum(minimum, edge - delta)
    if not np.any(valid):
        return seed
    differences = np.zeros(occupied.size + 1, np.int16)
    np.add.at(differences, begin[valid], 1)
    np.add.at(differences, finish[valid], -1)
    fill = np.cumsum(differences[:-1]).reshape(occupied.shape) > 0
    result = seed.copy()
    result[roi_top:roi_bottom][fill] = 255
    return result


def adaptive_white_mask(frame, roi_top=35, roi_bottom=200, max_width=72,
                        white_reference=None, floor_reference=None):
    """Return mask and diagnostics using measured floor/white brightness.

    Optional references come from a geometrically confirmed line, not an
    unvalidated percentile update. The caller owns temporal reference policy.
    HLS lightness measures brightness; HSV saturation avoids rejecting clipped
    whites such as RGB(255,254,254), whose HLS saturation is misleadingly high.
    """
    lightness = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)[:, :, 1]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi = lightness[roi_top:roi_bottom]
    floor, upper = (float(v) for v in np.percentile(roi, (50, 99)))
    reference = upper
    if white_reference is not None and floor_reference is not None:
        # A disappearing line alone changes the upper tail, not the floor.
        reference = max(reference, float(white_reference) + floor - float(floor_reference))
    span = max(0., reference - floor)
    minimum = float(np.clip(floor + max(5., span * .45), 20., 245.))
    delta = float(np.clip(span * .08, 3., 10.))
    background = cv2.GaussianBlur(lightness, (31, 31), 0)
    local_delta = cv2.subtract(lightness, background)
    neutral, colour_info = _white_colour_evidence(
        frame, hsv, lightness, roi_top, roi_bottom)
    seed = cv2.bitwise_and(cv2.inRange(lightness, minimum, 255), neutral)
    seed = cv2.bitwise_and(seed, cv2.inRange(local_delta, delta, 255))
    mask = _fill_interiors(lightness, seed, minimum, delta,
                           roi_top, roi_bottom, max_width)
    # An orange strip between neutral seeds must not be filled into a white line.
    mask = cv2.bitwise_and(mask, neutral)
    drive_roi = np.zeros_like(mask)
    drive_roi[roi_top:roi_bottom] = 255
    mask = cv2.bitwise_and(mask, drive_roi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    return mask, {'floor': floor, 'white_measured': upper,
                  'white_reference': reference, 'minimum': minimum, 'delta': delta,
                  **colour_info}
