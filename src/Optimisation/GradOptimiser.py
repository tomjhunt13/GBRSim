import numpy as np

from scipy.optimize import minimize
from src.Optimisation import Optimiser

class GradOptimiser(Optimiser.Optimiser):
    def __init__(self):

        super(GradOptimiser, self).__init__()

        self.default_parameters = {'tolerance': 1e-7}

    def _optimise(self, **kwargs):

        minimisation_input = self._assemble_input_vector()

        return minimize(self._cost, minimisation_input, method='Nelder-Mead', tol=kwargs['tolerance'])['x']

    def _assemble_input_vector(self):

        """
        for each variable:

        """

        minimisation_input = [None] * len(self.variables)

        for index, variable in enumerate(self.variables):
            minimisation_input[index] = variable['min'] + np.random.rand(1)[0] * (variable['max'] - variable['min'])

        return minimisation_input