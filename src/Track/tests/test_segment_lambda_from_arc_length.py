import unittest
import numpy as np

from src.Track import Line, Circle, BezierSpline
from src.Track import Track

class TestTrackArcLengthToLambda(unittest.TestCase):
    """

    """

    def setUp(self):
        # Three line track setup
        line_1 = Line.Line([[0, 0, 0], [2, 0, 0]])
        line_2 = Line.Line([[2, 0, 0], [2, 2, 0]])
        line_3 = Line.Line([[2, 2, 0], [0, 2, 0]])

        self.track = Track.Track([line_1, line_2, line_3])

    def test_three_lines_arc_length_1(self):

        segment, lambda_parameter = self.track.segment_lambda_from_arc_length(1)

        self.assertAlmostEqual(lambda_parameter, 0.5)



if __name__ == '__main__':
    unittest.main()