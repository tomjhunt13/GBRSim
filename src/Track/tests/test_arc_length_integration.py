import unittest
import numpy  as np

from src.Track import Line, Circle

class TestArcLengthIntegration(unittest.TestCase):

    def setUp(self):
        # Line setup
        self.line = Line.Line([[0, 0, 0], [2, 0, 0]])

        # Circle set up
        self.circle_radius = 2
        self.circle = Circle.Circle(self.circle_radius)

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



if __name__ == '__main__':
    unittest.main()