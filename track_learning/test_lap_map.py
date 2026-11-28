"""Focused tests for lap evidence and the on-disk route/map contract."""

import json
import math
from types import SimpleNamespace
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from track_learning.lap_map import (
    LapGate,
    SlamGraph,
    build_closed_route,
    find_loop_edge,
    keyframes_for_loop,
    parse_slam_graph,
    repair_route_in_grid,
    validate_route_in_grid,
    write_learning_artifacts,
)


def example_pose(index, *, radius=2.0):
    theta = index * 2.0 * math.pi / 120
    return (radius * math.cos(theta), radius * math.sin(theta),
            math.remainder(theta + math.pi / 2, 2.0 * math.pi))


def point_message(point):
    return SimpleNamespace(x=point[0], y=point[1])


def example_graph():
    # The last keyframe has crossed the gate. Its optimized position differs
    # slightly from the departure keyframe.
    positions = {index: example_pose(index)[:2] for index in range(122)}
    return SlamGraph(positions, tuple((i, i + 1) for i in range(121)) + ((0, 121),))


class LapMapTest(unittest.TestCase):
    def test_requires_departure_turn_and_forward_start_crossing(self):
        gate = LapGate()
        for index in range(121):
            pose = example_pose(index)
            self.assertFalse(gate.update(pose, pose))
        self.assertGreater(gate.distance, 12.0)
        self.assertGreater(gate.turn, 6.0)
        self.assertTrue(gate.update(example_pose(121), example_pose(121)))
        self.assertTrue(gate.update(example_pose(122), example_pose(122)))

    def test_stationary_map_jump_cannot_count_as_crossing(self):
        gate = LapGate()
        for index in range(121):
            pose = example_pose(index)
            self.assertFalse(gate.update(pose, pose))
        self.assertFalse(gate.update(example_pose(120), example_pose(121)))

    def test_reads_only_jazzy_mapping_graph_markers(self):
        graph = example_graph()
        header = SimpleNamespace(frame_id='map')
        markers = [
            SimpleNamespace(action=0, header=header, ns='slam_toolbox', type=2,
                            id=node, pose=SimpleNamespace(position=point_message(point)))
            for node, point in graph.nodes.items()
        ]
        markers.append(SimpleNamespace(action=0, header=header, ns='slam_toolbox_edges',
                                       type=5, id=0, points=[
                                           point_message(graph.nodes[node])
                                           for edge in graph.edges for node in edge]))
        markers.append(SimpleNamespace(action=0, header=header, ns='slam_toolbox_edges',
                                       type=5, id=1, points=[
                                           point_message(graph.nodes[node]) for node in (1, 121)]))
        parsed = parse_slam_graph(markers)
        self.assertEqual(parsed.nodes, graph.nodes)
        self.assertEqual(len(parsed.edges), 122)
        self.assertEqual(find_loop_edge(parsed, example_pose(0)), (0, 121))
        without_loop = SlamGraph(parsed.nodes, tuple(edge for edge in parsed.edges if edge != (0, 121)))
        self.assertIsNone(find_loop_edge(without_loop, example_pose(0)))

    def test_resamples_a_closed_route_and_constrains_fallback(self):
        graph = example_graph()
        points = list(keyframes_for_loop(graph, example_pose(0), (0, 121)))
        points[35] = (points[35][0] + 0.48, points[35][1])
        route = build_closed_route(points)
        self.assertGreater(len(route), 150)
        self.assertLess(math.dist(route[0][:2], route[-1][:2]), 0.076)
        self.assertLess(abs(math.remainder(route[0][2] - math.pi / 2, 2 * math.pi)), 0.1)
        self.assertTrue(keyframes_for_loop(graph, example_pose(0), None))
        not_closed = SlamGraph({node: point for node, point in graph.nodes.items() if node < 110}, ())
        with self.assertRaisesRegex(ValueError, 'No recent'):
            keyframes_for_loop(not_closed, example_pose(0), None)
        with self.assertRaisesRegex(ValueError, 'minimum turning radius'):
            build_closed_route(example_pose(i, radius=0.40)[:2] for i in range(121))

    def test_enforces_known_free_map_footprint_and_writes_consistent_pair(self):
        graph = example_graph()
        route = build_closed_route(keyframes_for_loop(graph, example_pose(0), (0, 121)))
        width = height = 120
        resolution = 0.05
        origin = (-3.0, -3.0, 0.0)
        grid_data = [0] * (width * height)
        validate_route_in_grid(route, width, height, resolution, origin, grid_data)
        # Side of the box intersects this cell even though the centerline misses it.
        grid_data[60 * width + 97] = -1
        with self.assertRaisesRegex(ValueError, 'cell occupancy -1'):
            validate_route_in_grid(route, width, height, resolution, origin, grid_data)
        with TemporaryDirectory() as directory:
            output = Path(directory) / 'lap'
            map_yaml, route_json = write_learning_artifacts(
                output, route, 2, 2, resolution, origin, [100, 0, -1, 50],
                metadata={'qualification': 'pose_graph_loop'})
            self.assertTrue(map_yaml.read_text().startswith('image: map.pgm\n'))
            self.assertEqual((output / 'map.pgm').read_bytes(), b'P5\n2 2\n255\n' + bytes([205, 205, 0, 254]))
            saved = json.loads(route_json.read_text())
            self.assertEqual(saved['metadata']['qualification'], 'pose_graph_loop')
            self.assertEqual(len(saved['points']), len(route))
            with self.assertRaises(FileExistsError):
                write_learning_artifacts(output, route, 2, 2, resolution, origin, [0] * 4)

    def test_repairs_a_blocked_seam_without_changing_occupancy(self):
        graph = example_graph()
        route = build_closed_route(keyframes_for_loop(graph, example_pose(0), (0, 121)))
        width = height = 120
        resolution = 0.05
        origin = (-3.0, -3.0, 0.0)
        grid = [0] * (width * height)
        grid_args = (width, height, resolution, origin, grid)
        self.assertIs(repair_route_in_grid(route, *grid_args), route)
        grid[60 * width + 100] = 100
        before = grid.copy()
        with self.assertRaisesRegex(ValueError, 'cell occupancy 100'):
            validate_route_in_grid(route, *grid_args)
        repaired = repair_route_in_grid(route, *grid_args)
        self.assertIsNot(repaired, route)
        self.assertEqual(grid, before)
        validate_route_in_grid(repaired, *grid_args)
        self.assertGreater(math.dist(route[0][:2], repaired[0][:2]), 0.1)
        with self.assertRaisesRegex(ValueError, 'too long to repair'):
            repair_route_in_grid(route, width, height, resolution, origin, [100] * len(grid))


if __name__ == '__main__':
    unittest.main()
