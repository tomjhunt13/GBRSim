import unittest
import numpy as np

from src.Optimisation.Optimiser import *

class Polynomial:
    def __init__(self, coefficients=[0, 0, 1]):
        self.coefficients = coefficients

        self.x = [0]

    def cost(self):
        x = self.x[0]
        y = 0
        for index, value in enumerate(self.coefficients):

            y += np.power(x, index) * value

        return y

class XY:
    def __init__(self, coefficients=[0, 0, 1]):
        self.coefficients = coefficients

        self.x = [0]
        self.y = [0]

    def cost(self):

        return self.x[0] * self.x[0] + self.y[0] * self.y[0]



class TestOptimiser(unittest.TestCase):

    def test_quadratic(self):
        print('Quadratic')
        quadratic = Polynomial()
        optimiser = Optimiser()

        optimiser.AddVariable('x', quadratic.x, -2, 2)

        result = optimiser.Optimise(quadratic.cost)
        self.assertAlmostEqual(result['x'], 0)

    def test_quartic(self):
        print('Quartic')
        quartic = Polynomial([1, 0, 0, 0, 1])
        optimiser = Optimiser()

        optimiser.AddVariable('x', quartic.x, -2, 2)
        result = optimiser.Optimise(quartic.cost)
        self.assertAlmostEqual(result['x'], 0)

    def test_xy(self):
        print('XY')
        xy = XY()
        optimiser = Optimiser()

        optimiser.AddVariable('x', xy.x, -2, 2)
        optimiser.AddVariable('y', xy.y, -2, 2)
        result = optimiser.Optimise(xy.cost)
        self.assertAlmostEqual(result['x'], 0)
        self.assertAlmostEqual(result['y'], 0)


if __name__ == '__main__':
    unittest.main()