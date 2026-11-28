"""Plan short forward paths along lidar-visible corridors without a course map."""

import math
from collections import defaultdict
from dataclasses import dataclass


# Expand the 0.27 m by 0.145 m half-footprint for cell rounding and the
# translation and rotation between sampled poses.
HALF_LENGTH = 0.345
HALF_WIDTH = 0.205
CELL = 0.04
BUCKET = 0.25
STEP = 0.075
STEPS_PER_SECTION = 4
SECTIONS = 8
MINIMUM_SECTIONS = 4
BEAM_WIDTH = 16
CURVATURES = tuple(i * 0.175 for i in range(-8, 9))
MAX_BOUNDARY_DISTANCE = 1.3


@dataclass(slots=True)
class State:
    x: float
    y: float
    heading: float
    curvature: float
    cost: float
    path: tuple


def scan_buckets(points):
    occupied = set()
    wall_slices = (set(), set())
    for x, y in points:
        if not (math.isfinite(x) and math.isfinite(y) and -1.4 <= x <= 4.0 and abs(y) <= 3.8):
            continue
        occupied.add((math.floor(x / CELL), math.floor(y / CELL)))
        if -0.35 <= x <= 1.45 and HALF_WIDTH < abs(y) <= MAX_BOUNDARY_DISTANCE:
            wall_slices[y > 0].add(math.floor(x / 0.20))
    if min(map(len, wall_slices)) < 3:
        raise ValueError('Both corridor boundaries must be visible before moving')

    buckets = defaultdict(list)
    for col, row in sorted(occupied):
        x, y = (col + 0.5) * CELL, (row + 0.5) * CELL
        buckets[math.floor(x / BUCKET), math.floor(y / BUCKET)].append((x, y))
    return buckets


def evaluate(x, y, heading, hits):
    """Return the footprint clearance cost and cross-section side balance."""
    c, s = math.cos(heading), math.sin(heading)
    clearance = 0.13
    left = right = MAX_BOUNDARY_DISTANCE
    for px, py in hits:
        dx, dy = px - x, py - y
        longitudinal = abs(dx * c + dy * s)
        if longitudinal >= HALF_LENGTH + 0.13:
            continue
        lateral = -dx * s + dy * c
        side_gap = abs(lateral) - HALF_WIDTH
        if side_gap < clearance:
            gap = max(longitudinal - HALF_LENGTH, side_gap)
            if gap <= 0:
                return None
            if gap < clearance:
                clearance = gap
        if longitudinal < 0.23:
            if 0 < lateral < left:
                left = lateral
            elif 0 <= -lateral < right:
                right = -lateral
    if left == MAX_BOUNDARY_DISTANCE and right == MAX_BOUNDARY_DISTANCE:
        return None
    # Occlusion can hide a side temporarily. Penalize losing that side without
    # inventing a wall location and still check every scan hit for collision.
    balance = (left - right) ** 2 if max(left, right) < MAX_BOUNDARY_DISTANCE else 0.45
    return 2.2 * balance + 35.0 * (0.13 - clearance) ** 2


def advance(state, curvature, nearby, reference, reference_weight):
    x, y, heading = state.x, state.y, state.heading
    cost = state.cost + 0.035 * curvature ** 2 + 0.10 * (curvature - state.curvature) ** 2
    new_points = []
    for _ in range(STEPS_PER_SECTION):
        angle = heading + STEP * curvature
        if curvature:
            x += (math.sin(angle) - math.sin(heading)) / curvature
            y += (math.cos(heading) - math.cos(angle)) / curvature
        else:
            x += STEP * math.cos(heading)
            y += STEP * math.sin(heading)
        heading = angle
        if abs(heading) > 1.45:
            return None
        terrain_cost = evaluate(x, y, heading, nearby(x, y))
        if terrain_cost is None:
            return None
        cost += STEP * terrain_cost
        if reference:
            distance_squared = min((x - px) ** 2 + (y - py) ** 2 for px, py in reference)
            cost += STEP * reference_weight * distance_squared
        new_points.append((x, y))
    return State(x, y, heading, curvature, cost, state.path + tuple(new_points))


def plan(points, reference=None, *, learning=True, min_turn_radius=0.65,
         beam_width=BEAM_WIDTH):
    """Return a swept-footprint-checked path in local base axes, including the origin.

    Both sets of points use metres, with positive x forward and positive y to
    the left. The reference is optional and provides a preference on repeat laps.
    A missing corridor or a blockage raises ValueError.
    """
    if not math.isfinite(min_turn_radius) or min_turn_radius <= 0.0:
        raise ValueError('Minimum turning radius must be finite and positive')
    if not isinstance(beam_width, int) or beam_width < 1:
        raise ValueError('Beam width must be a positive integer')
    scale = min(1.0, 1.0 / (max(CURVATURES) * min_turn_radius))
    curvatures = tuple(value * scale for value in CURVATURES)
    buckets = scan_buckets(points)
    nearby_cache = {}
    radius = math.ceil(MAX_BOUNDARY_DISTANCE / BUCKET) + 1

    def nearby(x, y):
        col, row = math.floor(x / BUCKET), math.floor(y / BUCKET)
        key = col, row
        if key not in nearby_cache:
            nearby_cache[key] = [hit
                                 for i in range(col - radius, col + radius + 1)
                                 for j in range(row - radius, row + radius + 1)
                                 for hit in buckets.get((i, j), ())]
        return nearby_cache[key]

    if evaluate(0.0, 0.0, 0.0, nearby(0.0, 0.0)) is None:
        raise ValueError('Starting footprint is obstructed')
    preferred = tuple((x, y) for x, y in (() if reference is None else reference)
                      if math.isfinite(x) and math.isfinite(y) and -0.6 <= x <= 3.2 and abs(y) <= 3.8)
    reference_weight = 0.45 if learning else 5.0
    states = [State(0.0, 0.0, 0.0, 0.0, 0.0, ((0.0, 0.0),))]
    for section in range(SECTIONS):
        successors = {}
        for state in states:
            for curvature in curvatures:
                candidate = advance(state, curvature, nearby, preferred, reference_weight)
                if candidate is None:
                    continue
                key = (round(candidate.y / 0.075), round(candidate.heading / 0.13))
                prior = successors.get(key)
                if prior is None or candidate.cost < prior.cost:
                    successors[key] = candidate
        if not successors:
            if section < MINIMUM_SECTIONS:
                raise ValueError('No forward path clears the obstacles and corridor boundaries')
            break
        states = sorted(successors.values(), key=lambda state: state.cost)[:beam_width]
    return list(states[0].path)
