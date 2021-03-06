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
        self.circle = Circle.HorizontalCircle(self.circle_radius)

        # Bezier setup
        knots = [[0, 0, 0], [12, 0, 0]]
        control_points = [[4, 0, 0], [8, 0, 0]]

        self.bezier = BezierSpline.CubicBezier(knots, control_points)

    def test_Line_0_1(self):

        arc_length = self.line.arc_length_integral(0, 1)

        self.assertAlmostEqual(arc_length, 2)

    def test_Line_0_0p5(self):
        arc_length = self.line.arc_length_integral(0, 0.5)

        self.assertAlmostEqual(arc_length, 1)

    def test_Line_0p5_1(self):
        arc_length = self.line.arc_length_integral(0.5, 1)

        self.assertAlmostEqual(arc_length, 1)

    def test_Line_0p25_0p75_longer(self):
        self.line.coordinates[1] = [0, 10, 0]

        arc_length = self.line.arc_length_integral(0.25, 0.75)

        self.assertAlmostEqual(arc_length, 5)

    def test_Circle_0_1(self):

        arc_length = self.circle.arc_length_integral(0, 1)

        self.assertAlmostEqual(arc_length, 2 * np.pi * self.circle_radius)

    def test_Circle_0_0p5(self):

        arc_length = self.circle.arc_length_integral(0, 0.5)

        self.assertAlmostEqual(arc_length, np.pi * self.circle_radius)

    def test_Circle_0p5_1(self):

        arc_length = self.circle.arc_length_integral(0.5, 1)

        self.assertAlmostEqual(arc_length, np.pi * self.circle_radius)

    def test_Bezier_0_1(self):

        arc_length = self.bezier.arc_length_integral(0, 1)

        self.assertAlmostEqual(arc_length, 12)

    def test_skewed_Bezier_0_1(self):

        # Move control point
        self.bezier.control_points[0][0] = 2

        arc_length = self.bezier.arc_length_integral(0, 1)

        self.assertAlmostEqual(arc_length, 12)


if __name__ == '__main__':
    unittest.main()