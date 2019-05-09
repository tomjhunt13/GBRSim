import unittest
import numpy as np

from src.Track import Line, Circle, BezierSpline
from src.Track import Track

class TestTrackArcLengthToLambda(unittest.TestCase):
    """

    """

    def setUp(self):
        # Three line track setup
        self.line_1 = Line.Line([[0, 0, 0], [2, 0, 0]])
        self.line_2 = Line.Line([[2, 0, 0], [2, 2, 0]])
        self.line_3 = Line.Line([[2, 2, 0], [0, 2, 0]])

        self.track = Track.Track([self.line_1, self.line_2, self.line_3])

        # Two Bezier lines track setup

    def test_three_lines_arc_length_1(self):

        segment, lambda_parameter = self.track.segment_lambda_from_arc_length(1)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 0.5)

        # Test correct segment
        truth = (self.line_1.coordinates == segment.coordinates)
        self.assertEqual(truth, True)

    def test_three_lines_arc_length_1p5(self):

        segment, lambda_parameter = self.track.segment_lambda_from_arc_length(1.5)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 0.75)

        # Test correct segment
        truth = (self.line_1.coordinates == segment.coordinates)
        self.assertEqual(truth, True)

    def test_three_lines_arc_length_2p5(self):

        segment, lambda_parameter = self.track.segment_lambda_from_arc_length(2.5)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 0.25)

        # Test correct segment
        truth = (self.line_2.coordinates == segment.coordinates)
        self.assertEqual(truth, True)

    def test_three_lines_arc_length_5(self):

        segment, lambda_parameter = self.track.segment_lambda_from_arc_length(5)

        # Test lambda
        self.assertAlmostEqual(lambda_parameter, 0.5)

        # Test correct segment
        truth = (self.line_3.coordinates == segment.coordinates)
        self.assertEqual(truth, True)





if __name__ == '__main__':
    unittest.main()