import unittest

from math import sqrt, pi, sin, cos
from src.Track_original import *

class TestTrack(unittest.TestCase):

    def test_triNormal(self):

        # Flat test
        A = [0, 0, 0]
        B = [1, 0, 0]
        C = [0, 1, 0]

        N = [0, 0, 1]

        self.assertEqual(N, triNormal(A, B, C))

        # Angled test
        A = [0, 0, 0]
        B = [1, 0, 0]
        C = [0, 0, 1]

        N = [0, -1, 0]

        self.assertEqual(N, triNormal(A, B, C))


    def test_pointOnArc(self):
        # Flat test
        A = [1, 0, 0]
        B = [0, 1, 0]
        C = [0, 0, 0]

        alpha = pi / 4

        P_calculated = pointOnArc(A, B, C, alpha)

        P_expected = [1 / sqrt(2), 1 / sqrt(2), 0]

        for i in range(3):
            self.assertAlmostEqual(P_calculated[i], P_expected[i])

        # Flat test - Smaller angle
        A = [1, 0, 0]
        B = [1 / sqrt(2), 1 / sqrt(2), 0]
        C = [0, 0, 0]

        alpha = pi / 8

        P_calculated = pointOnArc(A, B, C, alpha)

        P_expected = [cos(alpha), sin(alpha), 0]

        for i in range(3):
            self.assertAlmostEqual(P_calculated[i], P_expected[i])

        # 90 deg test
        A = [0, 1, 0]
        B = [0, 0, 1]
        C = [0, 0, 0]

        alpha = pi / 4

        P_calculated = pointOnArc(A, B, C, alpha)

        P_expected = [0, cos(alpha), sin(alpha)]

        for i in range(3):
            self.assertAlmostEqual(P_calculated[i], P_expected[i])

        # Offset, 90 deg, flat
        A = [1, 0, 0]
        B = [2, 1, 0]
        C = [1, 1, 0]

        alpha = pi / 2

        P_calculated = pointOnArc(A, B, C, alpha)
        P_expected = [2, 1, 0]

        for i in range(3):
            self.assertAlmostEqual(P_calculated[i], P_expected[i])


    def test_arc(self):

        # Valid flat arc
        arc_test = {'start': [0, 0, 0], 'end': [50, 50, 0], 'centre': [0, 50, 0]}
        valid = arc(arc_test)

        self.assertEqual(valid, True)
        self.assertAlmostEqual(arc_test['radius'], 50)
        self.assertAlmostEqual(arc_test['angle'], pi / 2)
        self.assertAlmostEqual(arc_test['length'], 50 * (pi / 2))

        # Invalid flat arc
        arc_test = {'start': [0, 0, 0], 'end': [60, 50, 0], 'centre': [0, 50, 0]}
        valid = arc(arc_test)

        self.assertEqual(valid, False)


if __name__ == '__main__':
    unittest.main()