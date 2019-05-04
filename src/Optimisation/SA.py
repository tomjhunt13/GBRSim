from scipy.optimize import dual_annealing
from src.Optimisation import Optimiser

class SA(Optimiser.Optimiser):

    def __init__(self):
        super(SA, self).__init__()

    def Optimise(self, cost_function, max_iterations=1, initial_temp=5230):

        self.cost_function = cost_function

        bounds = self._assemble_bounds_vector()

        optimisation_result = dual_annealing(self._cost, bounds, maxiter=max_iterations, initial_temp=initial_temp)['x']

        result = {}
        for index, value in enumerate(optimisation_result):
            result[self.variables[index]['name']] = value

        return result

    def _assemble_bounds_vector(self):

        bounds = [None] * len(self.variables)
        for index, var in enumerate(self.variables):
            bounds[index] = (var['min'], var['max'])

        return bounds