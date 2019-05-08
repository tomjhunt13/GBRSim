import unittest
import numpy as np

from src.Track import Line, Circle, BezierSpline

class TestArcLengthIntegration(unittest.TestCase):
    """
    Testing the arc length integration function in segment.

    Problem: s = integral of |df/dlambda| wrt lambda between alpha_1 and alpha_2
    """

    def setUp(self):
        # Line setup
        self.line = Line.Line([[0, 0, 0], [2, 0, 0]])

        # Circle set up
        self.circle_radius = 2
        self.circle = Circle.Circle(self.circle_radius)

        # Bezier setup
        knots = [[0, 0, 0], [12, 0, 0]]
        control_points = [[4, 0, 0], [8, 0, 0]]

        self.bezier = BezierSpline.CubicBezier(knots, control_points)

    def test_calculate_map_line(self):

        s_map = self.line.calculate_arc_length_map()

        # Check start, mid, end
        self.assertAlmostEqual(s_map[0], 0)
        self.assertAlmostEqual(s_map[10], 1)
        self.assertAlmostEqual(s_map[-1], 2)

    def test_calculate_map_circle(self):

        s_map = self.circle.calculate_arc_length_map()

        # Check start, mid, end
        self.assertAlmostEqual(s_map[0], 0)
        self.assertAlmostEqual(s_map[10], np.pi * self.circle_radius)
        self.assertAlmostEqual(s_map[-1], 2 * np.pi * self.circle_radius)

    def test_calculate_map_bezier(self):

        s_map = self.bezier.calculate_arc_length_map()

        # Check start, mid, end
        self.assertAlmostEqual(s_map[0], 0)
        self.assertAlmostEqual(s_map[10], 6)
        self.assertAlmostEqual(s_map[-1], 12)


if __name__ == '__main__':
    unittest.main()