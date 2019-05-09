import unittest

from src.Track import Line, BezierSpline
from src.Track import Track

class TestTrackArcLengthToLambda(unittest.TestCase):
    """
    Testing map from arc length to segment and lambda parameter in Track
    """

    def setUp(self):
        # Three line track setup
        self.line_1 = Line.Line([[0, 0, 0], [2, 0, 0]])
        self.line_2 = Line.Line([[2, 0, 0], [2, 2, 0]])
        self.line_3 = Line.Line([[2, 2, 0], [0, 2, 0]])

        self.line_track = Track.Track([self.line_1, self.line_2, self.line_3])

        # Two Bezier lines track setup
        knots_1 = [[0, 0, 0], [12, 0, 0]]
        control_points_1 = [[4, 0, 0], [8, 0, 0]]
        self.bezier_1 = BezierSpline.CubicBezier(knots_1, control_points_1)

        knots_2 = [[12, 0, 0], [12, 12, 0]]
        control_points_2 = [[12, 4, 0], [12, 8, 0]]
        self.bezier_2 = BezierSpline.CubicBezier(knots_2, control_points_2)

        self.bezier_track = Track.Track([self.bezier_1, self.bezier_2])

    def test_three_lines_arc_length_1(self):

        segment, lambda_parameter = self.line_track.segment_lambda_from_arc_length(1)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 0.5)

        # Test correct segment
        truth = (self.line_1.coordinates == segment.coordinates)
        self.assertEqual(truth, True)

    def test_three_lines_arc_length_1p5(self):

        segment, lambda_parameter = self.line_track.segment_lambda_from_arc_length(1.5)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 0.75)

        # Test correct segment
        truth = (self.line_1.coordinates == segment.coordinates)
        self.assertEqual(truth, True)

    def test_three_lines_arc_length_2p5(self):

        segment, lambda_parameter = self.line_track.segment_lambda_from_arc_length(2.5)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 0.25)

        # Test correct segment
        truth = (self.line_2.coordinates == segment.coordinates)
        self.assertEqual(truth, True)

    def test_three_lines_arc_length_5(self):

        segment, lambda_parameter = self.line_track.segment_lambda_from_arc_length(5)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 0.5)

        # Test correct segment
        truth = (self.line_3.coordinates == segment.coordinates)
        self.assertEqual(truth, True)

    def test_two_bezier_arc_length_6(self):

        segment, lambda_parameter = self.bezier_track.segment_lambda_from_arc_length(6)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 0.5)

        # Test correct segment
        truth = (self.bezier_1.knots == segment.knots)
        self.assertEqual(truth, True)

    def test_two_bezier_arc_length_11(self):

        segment, lambda_parameter = self.bezier_track.segment_lambda_from_arc_length(11)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 11./12.)

        # Test correct segment
        truth = (self.bezier_1.knots == segment.knots)
        self.assertEqual(truth, True)

    def test_two_bezier_arc_length_13(self):

        segment, lambda_parameter = self.bezier_track.segment_lambda_from_arc_length(13)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 1./12.)

        # Test correct segment
        truth = (self.bezier_2.knots == segment.knots)
        self.assertEqual(truth, True)


if __name__ == '__main__':
    unittest.main()