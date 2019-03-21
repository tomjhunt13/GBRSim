import unittest

from numpy import pi, cos, sin
from src.Track.Segment import *


class Circle:
    def __init__(self, radius):
        self.radius = radius

    def postition(self, t):
        theta = 2 * pi * t
        return [self.radius * cos(theta), self.radius * sin(theta)]


class TestBezierSpline(unittest.TestCase):

    def test_distance_between_coordinates(self):

        # Two points 3D
        points = [[0, 0, 0], [1, 0, 0]]
        self.assertAlmostEqual(distance_between_coordinates(points), 1)

        # Three points 3D
        points += [[2, 0, 0]]
        self.assertAlmostEqual(distance_between_coordinates(points), 2)

        # Four points 3D
        points += [[2, 0, 2]]
        self.assertAlmostEqual(distance_between_coordinates(points), 4)


    def test_length_of_segment(self):

        # Test circle radius 1
        c = Circle(1)
        length = length_of_parametric_curve(c.postition, tolerance=1e-5)
        self.assertAlmostEqual(length, np.pi * 2, places=5)

        # Radius 2
        c.radius = 2
        length = length_of_parametric_curve(c.postition, tolerance=1e-5)
        self.assertAlmostEqual(length, np.pi * 4, places=5)




if __name__ == '__main__':
    unittest.main()