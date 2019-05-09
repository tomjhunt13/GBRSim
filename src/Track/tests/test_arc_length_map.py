import unittest
import numpy as np

from src.Track import Line, Circle, BezierSpline

class TestArcLengthMap(unittest.TestCase):
    """

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

    def test_calculate_map_line(self):

        self.line.coordinates[1][0] = 2

        s_map = self.line.calculate_arc_length_map()

        # Check start, mid, end
        self.assertAlmostEqual(s_map[1][0], 0)
        self.assertAlmostEqual(s_map[1][10], 1)
        self.assertAlmostEqual(s_map[1][-1], 2)

    def test_calculate_map_circle(self):

        s_map = self.circle.calculate_arc_length_map()

        # Check start, mid, end
        self.assertAlmostEqual(s_map[1][0], 0)
        self.assertAlmostEqual(s_map[1][10], np.pi * self.circle_radius)
        self.assertAlmostEqual(s_map[1][-1], 2 * np.pi * self.circle_radius)

    def test_calculate_map_bezier(self):

        s_map = self.bezier.calculate_arc_length_map()

        # Check start, mid, end
        self.assertAlmostEqual(s_map[1][0], 0)
        self.assertAlmostEqual(s_map[1][10], 6)
        self.assertAlmostEqual(s_map[1][-1], 12)

    def test_map_line_0p5(self):
        self.line.coordinates[1][0] = 2

        lambda_parameter = self.line.lambda_from_arc_length(1)
        self.assertAlmostEqual(lambda_parameter, 0.5)

    def test_map_line_0p51(self):
        self.line.coordinates[1][0] = 2

        lambda_parameter = self.line.lambda_from_arc_length(1.02)
        self.assertAlmostEqual(lambda_parameter, 0.51)

    def test_map_long_line_0p51(self):

        self.line.coordinates[1][0] = 1000
        self.line.calculate_arc_length_map()

        lambda_parameter = self.line.lambda_from_arc_length(510)
        self.assertAlmostEqual(lambda_parameter, 0.51)

    def test_map_circle_0p5(self):

        lambda_parameter = self.circle.lambda_from_arc_length(np.pi * self.circle_radius)
        self.assertAlmostEqual(lambda_parameter, 0.5)

    def test_map_circle_0p51(self):

        lambda_parameter = self.circle.lambda_from_arc_length(np.pi * self.circle_radius * 1.02)
        self.assertAlmostEqual(lambda_parameter, 0.51)

    def test_map_bezier_0p508(self):

        lambda_parameter = self.bezier.lambda_from_arc_length(6.1)
        self.assertAlmostEqual(lambda_parameter, (6.1 / 12.))

    def test_map_upper_limit(self):

        upper_lambda_parameter = self.bezier.lambda_from_arc_length(100)
        self.assertAlmostEqual(upper_lambda_parameter, 1)

    def test_map_lower_limit(self):

        lower_lambda_parameter = self.bezier.lambda_from_arc_length(-100)
        self.assertAlmostEqual(lower_lambda_parameter, 0)


if __name__ == '__main__':
    unittest.main()