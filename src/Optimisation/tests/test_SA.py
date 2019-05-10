import unittest

from src.Optimisation.tests import example_functions
from src.Optimisation import SA

"""
look up Rosenbrock function
"""


class TestOptimiser(unittest.TestCase):

    def test_quadratic(self):
        print('Quadratic')
        quadratic = example_functions.Polynomial()
        optimiser = SA.SA()

        optimiser.add_variable('x', quadratic.x, -2, 2)

        result = optimiser.optimise(quadratic.cost, max_iterations=1000)
        self.assertAlmostEqual(result['x'], 0, places=3)

    def test_quartic(self):
        print('Quartic')
        quartic = example_functions.Polynomial([1, 0, 0, 0, 1])
        optimiser = SA.SA()

        optimiser.add_variable('x', quartic.x, -2, 2)
        result = optimiser.optimise(quartic.cost, max_iterations=1000)
        self.assertAlmostEqual(result['x'], 0, places=3)

    def test_xy(self):
        print('XY')
        xy = example_functions.XY()
        optimiser = SA.SA()

        optimiser.add_variable('x', xy.x, -2, 2)
        optimiser.add_variable('y', xy.y, -2, 2)
        result = optimiser.optimise(xy.cost, max_iterations=1000)
        self.assertAlmostEqual(result['x'], 0)
        self.assertAlmostEqual(result['y'], 0, places=3)


if __name__ == '__main__':
    unittest.main()