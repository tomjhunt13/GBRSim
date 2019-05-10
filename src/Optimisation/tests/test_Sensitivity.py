import unittest

from src.Optimisation.tests import example_functions
from src.Optimisation import Sensitivity


class TestOptimiser(unittest.TestCase):

    def setUp(self):

        self.dx = 1e-5

    def test_quadratic(self):

        # Test y = x^2, dy/dx = 2 x
        print('Quadratic')
        quadratic = example_functions.Polynomial()
        sensitivity = Sensitivity.Sensitivity(dx=self.dx)
        sensitivity.add_variable('x', quadratic.x)

        # x: 1
        quadratic.x[0] = 1
        gradient = sensitivity.sensitivity(quadratic.cost)
        self.assertAlmostEqual(gradient['x'], 2, places=3)

        # x: 2
        quadratic.x[0] = 2
        gradient = sensitivity.sensitivity(quadratic.cost)
        self.assertAlmostEqual(gradient['x'], 4, places=3)

        # x: -2
        quadratic.x[0] = -2
        gradient = sensitivity.sensitivity(quadratic.cost)
        self.assertAlmostEqual(gradient['x'], -4, places=3)

    def test_quartic(self):

        print('Quartic')
        # Test y = x^4 + 1, dy/dx = 4 x^3
        quadratic = example_functions.Polynomial([1, 0, 0, 0, 1])
        sensitivity = Sensitivity.Sensitivity(dx=self.dx)
        sensitivity.add_variable('x', quadratic.x)

        # x: 1
        quadratic.x[0] = 1
        gradient = sensitivity.sensitivity(quadratic.cost)
        self.assertAlmostEqual(gradient['x'], 4, places=3)

        # x: 2
        quadratic.x[0] = 2
        gradient = sensitivity.sensitivity(quadratic.cost)
        self.assertAlmostEqual(gradient['x'], 32, places=3)

        # x: -2
        quadratic.x[0] = -2
        gradient = sensitivity.sensitivity(quadratic.cost)
        self.assertAlmostEqual(gradient['x'], -32, places=3)

    def test_xy(self):

        # z = x^2 + y^2,  dz/dx = 2x, dz/dy = 2y
        print('XY')
        xy = example_functions.XY()
        sensitivity = Sensitivity.Sensitivity(dx=self.dx)
        sensitivity.add_variable('x', xy.x)
        sensitivity.add_variable('y', xy.y)

        # x: 0, y: 0
        xy.x[0] = 0
        xy.y[0] = 0
        gradient = sensitivity.sensitivity(xy.cost)
        self.assertAlmostEqual(gradient['x'], 0, places=3)
        self.assertAlmostEqual(gradient['y'], 0, places=3)

        # x: 1, y: 2
        xy.x[0] = 1
        xy.y[0] = 2
        gradient = sensitivity.sensitivity(xy.cost)
        self.assertAlmostEqual(gradient['x'], 2, places=3)
        self.assertAlmostEqual(gradient['y'], 4, places=3)


if __name__ == '__main__':
    unittest.main()