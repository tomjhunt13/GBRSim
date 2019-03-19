import unittest

# from math import sqrt, pi, sin, cos
from src.Track.BezierSpline import *

class TestBezierSpline(unittest.TestCase):

    def test_EndsMatchControlPoints(self):

        # Flat test
        knots = [[0, 0, 0], [0, 1, 0]]
        control_points = [[1, 0, 0], [1, 1, 0]]

        b = CubicBezier(knots, control_points)

        for t in range(2):
            P = b.position(t)

            for i in range(3):
                self.assertEqual(P[i], knots[t][i])

        # Raised end
        b.knots = [[0, 0, -1], [1, 0, 1]]
        for t in range(2):
            P = b.position(t)

            for i in range(3):
                self.assertEqual(P[i], b.knots[t][i])






if __name__ == '__main__':
    unittest.main()