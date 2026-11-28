"""Lap confirmation, optimized path extraction and portable map artifacts."""

from bisect import bisect_right
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory


Point = tuple[float, float]
Pose = tuple[float, float, float]


def _wrap(angle):
    return math.remainder(angle, 2.0 * math.pi)


@dataclass
class LapGate:
    """Use odometry for travel evidence and a directed map-frame start gate."""

    min_distance: float = 8.0
    min_turn: float = 4.7
    departure_radius: float = 1.5
    gate_half_width: float = 0.60
    heading_tolerance: float = 0.70
    max_gate_step: float = 0.45
    start: Pose | None = field(init=False, default=None)
    distance: float = field(init=False, default=0.0)
    turn: float = field(init=False, default=0.0)
    departed: bool = field(init=False, default=False)
    crossed: bool = field(init=False, default=False)
    _odom_start: Pose | None = field(init=False, default=None)
    _prev_odom: Pose | None = field(init=False, default=None)
    _prev_map: Pose | None = field(init=False, default=None)

    def __post_init__(self):
        for key in ('min_distance', 'min_turn', 'departure_radius', 'gate_half_width',
                    'heading_tolerance', 'max_gate_step'):
            value = getattr(self, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')

    def update(self, odom_pose: Pose, map_pose: Pose) -> bool:
        """Report a qualified crossing, retaining it while the graph catches up."""
        if not all(math.isfinite(value) for value in (*odom_pose, *map_pose)):
            raise ValueError('Lap poses must be finite x, y and yaw')
        if self.start is None:
            self.start = self._prev_map = map_pose
            self._odom_start = self._prev_odom = odom_pose
            return False
        if self.crossed:
            return True

        ox, oy, oyaw = self._prev_odom
        dx, dy = odom_pose[0] - ox, odom_pose[1] - oy
        travel = math.hypot(dx, dy)
        yaw_step = _wrap(odom_pose[2] - oyaw)
        if travel > 0.75 or abs(yaw_step) > 1.0:
            raise ValueError('Discontinuous odometry while timing the learning lap')
        self.distance += travel
        self.turn += yaw_step
        self.departed |= math.dist(odom_pose[:2], self._odom_start[:2]) >= self.departure_radius

        sx, sy, syaw = self.start
        cos_start, sin_start = math.cos(syaw), math.sin(syaw)
        prev_x, prev_y = self._prev_map[:2]
        before = (prev_x - sx) * cos_start + (prev_y - sy) * sin_start
        mx, my, myaw = map_pose
        after = (mx - sx) * cos_start + (my - sy) * sin_start
        forward = dx * math.cos(oyaw) + dy * math.sin(oyaw)
        if (self.departed and self.distance >= self.min_distance
                and abs(self.turn) >= self.min_turn
                and before < 0.0 <= after and 0.0 < after - before <= self.max_gate_step
                and forward > 0.001 and abs(_wrap(myaw - syaw)) <= self.heading_tolerance):
            # Check the lateral position at the plane, rather than at either sample.
            fraction = -before / (after - before)
            cross_x = prev_x + fraction * (mx - prev_x)
            cross_y = prev_y + fraction * (my - prev_y)
            lateral = -(cross_x - sx) * sin_start + (cross_y - sy) * cos_start
            self.crossed = abs(lateral) <= self.gate_half_width

        self._prev_odom, self._prev_map = odom_pose, map_pose
        return self.crossed


@dataclass(frozen=True)
class SlamGraph:
    nodes: dict[int, Point]
    edges: tuple[tuple[int, int], ...]


def parse_slam_graph(markers, frame_id='map') -> SlamGraph:
    """Decode Jazzy slam_toolbox MarkerArray nodes and mapping graph edges."""
    # Jazzy uses ADD/SPHERE for optimized keyframes and ADD/LINE_LIST for
    # edges. The mapping edge marker is ID 0. Its coordinates come from the
    # same corrected poses as the nodes, so each endpoint matches exactly.
    nodes = {}
    edge_points = []
    for marker in markers:
        if marker.action != 0 or marker.header.frame_id != frame_id:
            continue
        if marker.ns == 'slam_toolbox' and marker.type == 2:
            position = marker.pose.position
            if math.isfinite(position.x) and math.isfinite(position.y):
                nodes[marker.id] = (position.x, position.y)
        elif marker.ns == 'slam_toolbox_edges' and marker.type == 5 and marker.id == 0:
            edge_points.extend((point.x, point.y) for point in marker.points)

    ids_by_point = {}
    for node_id, point in nodes.items():
        ids_by_point[point] = node_id if point not in ids_by_point else None
    edges = set()
    for point_a, point_b in zip(edge_points[::2], edge_points[1::2]):
        a, b = ids_by_point.get(point_a), ids_by_point.get(point_b)
        if a is not None and b is not None and a != b:
            edges.add((min(a, b), max(a, b)))
    return SlamGraph(dict(sorted(nodes.items())), tuple(sorted(edges)))


def _stations(points):
    travel = [0.0]
    for a, b in zip(points, points[1:]):
        travel.append(travel[-1] + math.dist(a, b))
    return travel


def find_loop_edge(graph: SlamGraph, start_pose: Pose, min_distance=8.0,
                   *, start_neighborhood=1.2) -> tuple[int, int] | None:
    """Find a nonlocal mapping edge connecting departure and return keyframes."""
    ordered = tuple(sorted(graph.nodes))
    if not ordered:
        return None
    station = dict(zip(ordered, _stations([graph.nodes[node] for node in ordered])))
    start_xy = start_pose[:2]
    near = {node for node in ordered if math.dist(graph.nodes[node], start_xy) <= start_neighborhood}
    candidates = [
        (a, b) for a, b in graph.edges
        if a in near and b in near
        and station[a] <= 2.0 * start_neighborhood
        and station[b] - station[a] >= min_distance
        and station[ordered[-1]] - station[b] <= 2.0 * start_neighborhood
    ]
    return min(candidates, key=lambda edge: (edge[1], edge[0]), default=None)


def keyframes_for_loop(graph: SlamGraph, start_pose: Pose, edge: tuple[int, int] | None,
                       *, min_distance=8.0) -> tuple[Point, ...]:
    """Trim optimized keyframes around an edge or a confirmed geometric return."""
    keys = sorted(graph.nodes)
    points = [graph.nodes[key] for key in keys]
    if len(points) < 8:
        raise ValueError('Too few optimized keyframes for a learning lap')
    station = _stations(points)
    start_xy = start_pose[:2]
    if edge is None:
        early = [i for i, distance in enumerate(station) if distance <= 1.2]
        begin = min(early, key=lambda i: math.dist(points[i], start_xy))
        if math.dist(points[begin], start_xy) > 0.45:
            raise ValueError('No optimized departure keyframe near the start gate')
        candidates = []
        for i in range(begin + 1, len(points)):
            if not (station[i] - station[begin] >= min_distance
                    and station[-1] - station[i] <= 0.75):
                continue
            close = math.dist(points[i], points[begin])
            x0, y0 = points[i - 1]
            x1, y1 = points[i]
            heading = math.atan2(y1 - y0, x1 - x0)
            if close <= 0.45 and abs(_wrap(heading - start_pose[2])) <= 0.7:
                candidates.append((close, i))
        if not candidates:
            raise ValueError('No recent optimized keyframe closes the route near the start')
        _, finish = min(candidates)
        return tuple(points[begin:finish + 1])

    early, late = keys.index(edge[0]), keys.index(edge[1])
    begin = min(range(early + 1), key=lambda i: math.dist(points[i], start_xy))
    # An edge can be added slightly before or after the rover crosses the plane.
    # Use the near-gate point in the surrounding late samples as the route seam.
    finish_candidates = [
        i for i in range(begin + 1, len(points))
        if abs(station[i] - station[late]) <= 1.2 and station[i] >= station[late] * 0.8
    ]
    finish = min(finish_candidates, key=lambda i: math.dist(points[i], points[begin]))
    return tuple(points[begin:finish + 1])


def _resample_ring(points, spacing):
    ring = (*points, points[0])
    stations = _stations(ring)
    length = stations[-1]
    count = max(8, math.ceil(length / spacing))
    result = []
    for i in range(count):
        station = i * length / count
        index = bisect_right(stations, station) - 1
        fraction = (station - stations[index]) / (stations[index + 1] - stations[index])
        x0, y0 = ring[index]
        x1, y1 = ring[index + 1]
        result.append((x0 + fraction * (x1 - x0), y0 + fraction * (y1 - y0)))
    return result


def _clean_keyframes(points):
    filtered = []
    for point in points:
        if len(point) != 2 or not all(map(math.isfinite, point)):
            raise ValueError('Optimized keyframes must be finite x, y points')
        if not filtered or math.dist(filtered[-1], point) >= 0.02:
            filtered.append(tuple(point))
    if len(filtered) > 1 and math.dist(filtered[0], filtered[-1]) < 0.02:
        filtered.pop()
    cleaned = []
    for index, point in enumerate(filtered):
        before = filtered[index - 1]
        after = filtered[(index + 1) % len(filtered)]
        back, ahead = math.dist(before, point), math.dist(point, after)
        chord = math.dist(before, after)
        # An isolated bad fix doubles back to the previous neighborhood.
        if min(back, ahead) > 0.14 and chord < 0.70 * min(back, ahead):
            continue
        cleaned.append(point)
    return cleaned


def _smooth_ring(points, sample_spacing, sigma):
    count = len(points)
    radius = min((count - 1) // 2, math.ceil(3.0 * sigma / sample_spacing))
    weights = [math.exp(-0.5 * (offset * sample_spacing / sigma) ** 2)
               for offset in range(-radius, radius + 1)]
    scale = sum(weights)
    smoothed = []
    for i in range(count):
        window = [points[(i + offset) % count] for offset in range(-radius, radius + 1)]
        x = sum(point[0] * weight for point, weight in zip(window, weights)) / scale
        y = sum(point[1] * weight for point, weight in zip(window, weights)) / scale
        smoothed.append((x, y))
    return smoothed


def _curvature(a, b, c):
    ab, bc, ac = math.dist(a, b), math.dist(b, c), math.dist(a, c)
    cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return math.inf if min(ab, bc, ac) < 1e-6 else 2.0 * abs(cross) / (ab * bc * ac)


def _path_curvature(points):
    return max(_curvature(points[i - 1], point, points[(i + 1) % len(points)])
               for i, point in enumerate(points))


def _route_poses(points):
    result = []
    for i, (x, y) in enumerate(points):
        previous, following = points[i - 1], points[(i + 1) % len(points)]
        yaw = math.atan2(following[1] - previous[1], following[0] - previous[0])
        result.append((x, y, yaw))
    return tuple(result)


def build_closed_route(points, *, spacing=0.075, min_turn_radius=0.65,
                       max_close_gap=0.90, max_smoothing_shift=0.22) -> tuple[Pose, ...]:
    """Smooth and sample one closed Ackermann route from optimized positions."""
    for value in (spacing, min_turn_radius, max_close_gap, max_smoothing_shift):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('Route limits must be finite and positive')
    points = _clean_keyframes(points)
    if len(points) < 8:
        raise ValueError('Too few optimized keyframes for a closed route')
    if math.dist(points[-1], points[0]) > max_close_gap:
        raise ValueError('Optimized keyframes do not return close enough to the start')
    if any(math.dist(a, b) > max_close_gap for a, b in zip(points, points[1:])):
        raise ValueError('Optimized keyframes have a gap along the route')

    closed_length = _stations((*points, points[0]))[-1]
    samples = _resample_ring(points, spacing)
    sample_spacing = closed_length / len(samples)
    for sigma in (0.13, 0.19, 0.26, 0.35, 0.45, 0.55):
        smoothed = _smooth_ring(samples, sample_spacing, sigma)
        if max(map(math.dist, samples, smoothed)) > max_smoothing_shift:
            continue
        route = _resample_ring(smoothed, spacing)
        if _path_curvature(route) <= 1.0 / min_turn_radius:
            return _route_poses(route)
    raise ValueError('Learned path cannot meet the minimum turning radius within the smoothing limit')


def _validate_grid(width, height, resolution, origin_pose, grid_data):
    if width <= 0 or height <= 0 or len(grid_data) != width * height:
        raise ValueError('Occupancy dimensions do not match the grid data')
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('Occupancy resolution must be finite and positive')
    if len(origin_pose) != 3 or not all(map(math.isfinite, origin_pose)):
        raise ValueError('Occupancy origin must be finite x, y and yaw')
    if any(value < -1 or value > 100 for value in grid_data):
        raise ValueError('Occupancy cells must be between -1 and 100')


_OUTSIDE_MAP = -2
_HALF_LENGTH = 0.27
_HALF_WIDTH = 0.145
_FOOTPRINT_PADDING = 0.02


def _footprint_check(width, height, resolution, origin_pose, grid_data, half_length, half_width):
    origin_x, origin_y, origin_yaw = origin_pose
    origin_cos, origin_sin = math.cos(origin_yaw), math.sin(origin_yaw)

    def blocked(map_x, map_y, yaw):
        dx, dy = map_x - origin_x, map_y - origin_y
        grid_x = dx * origin_cos + dy * origin_sin
        grid_y = -dx * origin_sin + dy * origin_cos
        angle = yaw - origin_yaw
        cosine, sine = math.cos(angle), math.sin(angle)
        span_x = abs(cosine) * half_length + abs(sine) * half_width
        span_y = abs(sine) * half_length + abs(cosine) * half_width
        first_x = math.floor((grid_x - span_x) / resolution)
        last_x = math.floor((grid_x + span_x) / resolution)
        first_y = math.floor((grid_y - span_y) / resolution)
        last_y = math.floor((grid_y + span_y) / resolution)
        if first_x < 0 or last_x >= width or first_y < 0 or last_y >= height:
            return _OUTSIDE_MAP
        cell_projection = 0.5 * resolution * (abs(cosine) + abs(sine))
        for row in range(first_y, last_y + 1):
            dy = (row + 0.5) * resolution - grid_y
            for column in range(first_x, last_x + 1):
                cell = grid_data[row * width + column]
                if 0 <= cell <= 19:
                    continue
                dx = (column + 0.5) * resolution - grid_x
                along = dx * cosine + dy * sine
                lateral = -dx * sine + dy * cosine
                if (abs(along) <= half_length + cell_projection
                        and abs(lateral) <= half_width + cell_projection):
                    return cell
        return None

    return blocked


def _route_obstructions(route, resolution, corner_radius, check, order=None):
    for i in range(len(route)) if order is None else order:
        x, y, yaw = route[i]
        x_next, y_next, yaw_next = route[(i + 1) % len(route)]
        yaw_delta = _wrap(yaw_next - yaw)
        translation = math.hypot(x_next - x, y_next - y)
        corner_travel = abs(yaw_delta) * corner_radius
        steps = max(1, math.ceil(2 * (translation + corner_travel) / resolution))
        for j in range(steps):
            fraction = j / steps
            map_x = x + fraction * (x_next - x)
            map_y = y + fraction * (y_next - y)
            cell = check(map_x, map_y, yaw + fraction * yaw_delta)
            if cell is not None:
                yield i, map_x, map_y, cell
                break


def _obstruction_reason(obstruction):
    _, x, y, cell = obstruction
    if cell == _OUTSIDE_MAP:
        return f'Learned route footprint leaves the map at ({x:.2f}, {y:.2f})'
    return (f'Learned route footprint intersects non-free map space at '
            f'({x:.2f}, {y:.2f}), cell occupancy {cell}')


def validate_route_in_grid(route, width, height, resolution, origin_pose, grid_data,
                           *, half_length=_HALF_LENGTH, half_width=_HALF_WIDTH,
                           padding=_FOOTPRINT_PADDING):
    """Check the swept rectangular rover footprint against known map free space."""
    _validate_grid(width, height, resolution, origin_pose, grid_data)
    if len(route) < 8:
        raise ValueError('Too few route samples for an occupancy check')
    if (not all(map(math.isfinite, (half_length, half_width, padding)))
            or half_length <= 0 or half_width <= 0 or padding < 0):
        raise ValueError('Invalid route validation footprint')
    half_length += padding
    half_width += padding
    check = _footprint_check(width, height, resolution, origin_pose, grid_data,
                             half_length, half_width)
    corner_radius = math.hypot(half_length, half_width)
    first = next(_route_obstructions(route, resolution, corner_radius, check), None)
    if first is not None:
        raise ValueError(_obstruction_reason(first))


def repair_route_in_grid(route, width, height, resolution, origin_pose, grid_data,
                         *, min_turn_radius=0.65):
    """Find a bounded smooth detour around one blocked interval of a closed route."""
    _validate_grid(width, height, resolution, origin_pose, grid_data)
    if len(route) < 8 or any(len(pose) != 3 or not all(map(math.isfinite, pose)) for pose in route):
        raise ValueError('Repair requires a closed route with finite x, y and yaw')
    if not math.isfinite(min_turn_radius) or min_turn_radius <= 0.0:
        raise ValueError('Minimum turning radius must be finite and positive')
    points = [pose[:2] for pose in route]
    if _path_curvature(points) > 1.0 / min_turn_radius:
        raise ValueError('Input route does not meet the minimum turning radius')
    half_length = _HALF_LENGTH + _FOOTPRINT_PADDING
    half_width = _HALF_WIDTH + _FOOTPRINT_PADDING
    corner_radius = math.hypot(half_length, half_width)
    check = _footprint_check(width, height, resolution, origin_pose, grid_data,
                             half_length, half_width)
    blocked = tuple(_route_obstructions(route, resolution, corner_radius, check))
    if not blocked:
        return route

    stations = _stations((*points, points[0]))
    length = stations[-1]
    blocked_stations = [stations[index] for index, *_ in blocked]
    # Put the seam of the search outside the obstructed interval, even if the
    # obstruction straddles the route's actual start.
    gaps = [b - a for a, b in zip(blocked_stations, blocked_stations[1:])]
    gaps.append(length + blocked_stations[0] - blocked_stations[-1])
    largest_gap = max(range(len(gaps)), key=gaps.__getitem__)
    blocked_length = length - gaps[largest_gap]
    if blocked_length > min(3.0, 0.25 * length):
        raise ValueError(f'{_obstruction_reason(blocked[0])}. Blocked interval is too long to repair')
    interval_start = blocked_stations[(largest_gap + 1) % len(blocked)]
    midpoint = round((interval_start + blocked_length / 2) * 10.0) / 10.0
    centers = [midpoint]
    for step in range(1, 10):
        centers.extend((midpoint - 0.1 * step, midpoint + 0.1 * step))
    normals = [(-math.sin(yaw), math.cos(yaw)) for _, _, yaw in route]
    indices = sorted({index for index, *_ in blocked})
    count = len(route)
    sweep_order = sorted(range(count),
                         key=lambda index: abs(math.remainder(stations[index] - midpoint, length)))
    station_scale = length / math.pi
    shapes = []
    for center in centers:
        deltas = [station_scale * math.sin((station - center) / station_scale)
                  for station in stations[:-1]]
        for sigma in (0.70, 0.85, 1.0, 1.15, 1.30, 1.45, 1.60, 1.75):
            weights = [math.exp(-0.5 * (delta / sigma) ** 2) for delta in deltas]
            shapes.append([(weight * nx, weight * ny) for weight, (nx, ny) in zip(weights, normals)])

    for magnitude in range(6, 32):
        for shape in shapes:
            for direction in (1, -1):
                shift = direction * magnitude * 0.02

                def point(index):
                    x, y = points[index]
                    dx, dy = shape[index]
                    return x + shift * dx, y + shift * dy

                # Test likely obstruction poses before rebuilding the full route.
                for index in indices:
                    before, after = point(index - 1), point((index + 1) % count)
                    heading = math.atan2(after[1] - before[1], after[0] - before[0])
                    if check(*point(index), heading) is not None:
                        break
                else:
                    candidate = tuple(point(index) for index in range(count))
                    if _path_curvature(candidate) > 1.0 / min_turn_radius:
                        continue
                    poses = _route_poses(candidate)
                    if next(_route_obstructions(
                            poses, resolution, corner_radius, check, sweep_order), None) is None:
                        return poses
    raise ValueError(f'{_obstruction_reason(blocked[0])}. No bounded curvature-safe detour fits the map')


def write_learning_artifacts(output_dir, route, width, height, resolution, origin_pose,
                             grid_data, *, metadata=None) -> tuple[Path, Path]:
    """Write one immutable map/route result readable by Nav2 and the oval driver."""
    output_dir = Path(output_dir)
    if output_dir.exists():
        raise FileExistsError(f'Use a new output directory for this lap: {output_dir}')
    _validate_grid(width, height, resolution, origin_pose, grid_data)
    if len(route) < 8 or any(len(pose) != 3 or not all(map(math.isfinite, pose)) for pose in route):
        raise ValueError('A finished learned route must have finite x, y and yaw samples')

    # OccupancyGrid begins at its lower-left corner. PGM starts at its top left.
    # Gray 205 lies between the matching trinary map thresholds below.
    pixels = bytes(
        254 if 0 <= value <= 19 else 0 if value >= 65 else 205
        for row in range(height - 1, -1, -1)
        for value in grid_data[row * width:(row + 1) * width]
    )
    yaml_text = (
        'image: map.pgm\nmode: trinary\n'
        f'resolution: {resolution!r}\norigin: {list(origin_pose)!r}\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
    )
    result = {
        'version': 1,
        'frame_id': 'map',
        'closed': True,
        'points': [dict(x=round(x, 6), y=round(y, 6), yaw=round(yaw, 6)) for x, y, yaw in route],
    }
    if metadata is not None:
        result['metadata'] = metadata
    json_text = json.dumps(result, indent=2, allow_nan=False, sort_keys=True) + '\n'
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix='.learned-lap-', dir=output_dir.parent) as staging:
        artifacts = Path(staging) / 'artifacts'
        artifacts.mkdir()
        (artifacts / 'map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode('ascii') + pixels)
        (artifacts / 'map.yaml').write_text(yaml_text, encoding='utf-8', newline='\n')
        (artifacts / 'learned_route.json').write_text(json_text, encoding='utf-8', newline='\n')
        artifacts.rename(output_dir)
    return output_dir / 'map.yaml', output_dir / 'learned_route.json'
