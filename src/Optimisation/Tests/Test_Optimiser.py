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






class TestOptimiser(unittest.TestCase):

    def test_base_optimiser(self):

        """
        Test quadratic x^2 returns x = 0
        """


        quadratic = Polynomial()
        optimiser = Optimiser()

        optimiser.AddVariable('x', quadratic.x, -2, 1)

        result = optimiser.Optimise(quadratic.cost)


        self.assertAlmostEqual(result['x'], 0)



if __name__ == '__main__':
    unittest.main()