import numpy as np

from scipy.optimize import minimize
from src.Optimisation import Optimiser

class GradOptimiser(Optimiser.Optimiser):
    def __init__(self):
        super(GradOptimiser, self).__init__()

    def Optimise(self, cost_function, tolerance=1e-7):

        self.cost_function = cost_function

        input = self._assemble_input_vector()

        optimisation_result = minimize(self._cost, input, method='Nelder-Mead', tol=tolerance)['x']

        result = {}
        for index, value in enumerate(optimisation_result):
            result[self.variables[index]['name']] = value

        return result

    def _assemble_input_vector(self):

        """
        for each variable:

        """

        input = [None] * len(self.variables)

        for index, variable in enumerate(self.variables):
            input[index] = variable['min'] + np.random.rand(1)[0] * (variable['max'] - variable['min'])

        return input