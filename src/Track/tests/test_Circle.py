import unittest

from src.Track import Circle


class TestBezierSpline(unittest.TestCase):

    def test_vertical_circle_coordinates(self):

        radius = 1
        vertical_circle = Circle.VerticalCircle(radius)

        bottom = vertical_circle.position(0)
        self.assertAlmostEqual(bottom[0], radius)
        self.assertAlmostEqual(bottom[2], 0)

        quarter = vertical_circle.position(0.25)
        self.assertAlmostEqual(quarter[0], 0)
        self.assertAlmostEqual(quarter[2], radius)

        three_quarter = vertical_circle.position(0.75)
        self.assertAlmostEqual(three_quarter[0], 0)
        self.assertAlmostEqual(three_quarter[2], -radius)


if __name__ == '__main__':
    unittest.main()