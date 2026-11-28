import math
import unittest

from track_learning.corridor import plan


def straight_walls():
    return [(0.04 * index, side * 0.64) for side in (-1, 1) for index in range(-20, 101)]


class CorridorTest(unittest.TestCase):
    def test_center_between_straight_walls(self):
        path = plan(straight_walls())
        self.assertEqual(path[0], (0.0, 0.0))
        self.assertGreater(path[-1][0], 2.2)
        self.assertLess(max(abs(y) for _, y in path), 0.06)

    def test_replay_detours_around_a_cone(self):
        walls = straight_walls()
        center = [(index * 0.075, 0.0) for index in range(33)]
        cone = [(1.3 + 0.04 * math.cos(angle), 0.04 * math.sin(angle))
                for angle in (index * math.pi / 8 for index in range(16))]
        min_turn_radius = 1.3
        path = plan(walls + cone, center, learning=False, min_turn_radius=min_turn_radius,
                    beam_width=8)
        crossing = min(path, key=lambda point: abs(point[0] - 1.3))
        self.assertGreater(path[-1][0], 1.8)
        self.assertGreater(abs(crossing[1]), 0.22)
        for a, b, c in zip(path, path[1:], path[2:]):
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            curvature = 2 * abs(cross) / (math.dist(a, b) * math.dist(b, c) * math.dist(a, c))
            self.assertLessEqual(curvature, 1 / min_turn_radius + 1e-6)

    def test_stops_when_the_corridor_is_blocked(self):
        obstruction = [(0.7, row * 0.04) for row in range(-16, 17)]
        with self.assertRaisesRegex(ValueError, 'No forward path'):
            plan(straight_walls() + obstruction)

    def test_rejects_missing_boundary(self):
        with self.assertRaisesRegex(ValueError, 'Both corridor boundaries'):
            plan((x, y) for x, y in straight_walls() if y > 0)


if __name__ == '__main__':
    unittest.main()
