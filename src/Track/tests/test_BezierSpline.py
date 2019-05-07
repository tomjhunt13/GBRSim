import unittest

from src.Track.BezierSpline import *

class TestBezierSpline(unittest.TestCase):

    def test_EndsMatchKnots(self):

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

    def test_Derivatives(self):

        # Flat test
        knots = [[0, 0, 0], [3, 0, 0]]
        control_points = [[1, 0, 0], [2, 0, 0]]

        b = CubicBezier(knots, control_points)

        computed_derivative = b.dx_dlambda(0.5)
        expected_derivative = [3, 0, 0]


        for i in range(3):
            self.assertEqual(computed_derivative[i], expected_derivative[i])

        # Angled 1
        knots = [[0, 0, 0], [3, 0, 3]]
        control_points = [[1, 0, 1], [2, 0, 2]]

        b = CubicBezier(knots, control_points)

        computed_derivative = b.dx_dlambda(0.5)
        expected_derivative = [3, 0, 3]

        for i in range(3):
            self.assertEqual(computed_derivative[i], expected_derivative[i])

        # Angled 2
        knots = [[0, 0, 0], [3, 3, 3]]
        control_points = [[1, 1, 1], [2, 2, 2]]

        b = CubicBezier(knots, control_points)

        computed_derivative = b.dx_dlambda(0.5)
        expected_derivative = [3, 3, 3]

        for i in range(3):
            self.assertEqual(computed_derivative[i], expected_derivative[i])


    def test_gradient(self):

        # Flat
        knots = [[0, 0, 0], [3, 0, 0]]
        control_points = [[1, 0, 0], [2, 0, 0]]

        b = CubicBezier(knots, control_points)

        calculated_gradient = b.gradient(0)
        expected_gradient = 0

        self.assertAlmostEqual(calculated_gradient, expected_gradient)


        # Inclined
        knots = [[0, 0, 0], [3, 0, 3]]
        control_points = [[1, 0, 1], [2, 0, 2]]

        b = CubicBezier(knots, control_points)

        calculated_gradient = b.gradient(0)
        expected_gradient = np.pi/4

        self.assertAlmostEqual(calculated_gradient, expected_gradient)


if __name__ == '__main__':
    unittest.main()