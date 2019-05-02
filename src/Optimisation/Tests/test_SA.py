import unittest

from src.Optimisation.tests import example_functions
from src.Optimisation import SimulatedAnnealing

"""
look up Rosenbrock function
"""


class TestOptimiser(unittest.TestCase):

    def test_quadratic(self):
        print('Quadratic')
        quadratic = example_functions.Polynomial()
        optimiser = SimulatedAnnealing.SA()

        optimiser.AddVariable('x', quadratic.x, -2, 2)

        result = optimiser.Optimise(quadratic.cost)
        self.assertAlmostEqual(result['x'], 0)

    def test_quartic(self):
        print('Quartic')
        quartic = example_functions.Polynomial([1, 0, 0, 0, 1])
        optimiser = SimulatedAnnealing.SA()

        optimiser.AddVariable('x', quartic.x, -2, 2)
        result = optimiser.Optimise(quartic.cost)
        self.assertAlmostEqual(result['x'], 0)

    def test_xy(self):
        print('XY')
        xy = example_functions.XY()
        optimiser = SimulatedAnnealing.SA()

        optimiser.AddVariable('x', xy.x, -2, 2)
        optimiser.AddVariable('y', xy.y, -2, 2)
        result = optimiser.Optimise(xy.cost)
        self.assertAlmostEqual(result['x'], 0)
        self.assertAlmostEqual(result['y'], 0)


if __name__ == '__main__':
    unittest.main()