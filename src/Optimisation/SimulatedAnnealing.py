import numpy as np

from scipy.optimize import dual_annealing
from src.Optimisation import Optimiser

class SA(Optimiser.Optimiser):

    def __init__(self, k_max=500, sample_generator=1):
        super(SA, self).__init__()

    def Optimise(self, cost_function, max_iterations=1000, initial_temp=5230):

        self.cost_function = cost_function

        bounds = self._assemble_bounds_vector()

        optimisation_result = dual_annealing(self._cost, bounds, maxiter=max_iterations, initial_temp=initial_temp)['x']

        result = {}
        for index, value in enumerate(optimisation_result):
            result[self.variables[index]['name']] = value

        return result

    def _assemble_input_vector(self):

        bounds = [None] * len(self.variables)
        for index, var in self.variables:
            bounds[index] = (var['min'], var['max'])

        return bounds


    # def Optimise(self, cost_function, initial_temperature=1000):
    #
    #     # Generate initial sample - s
    #     s = self.GenerateSamples()
    #
    #     # Evaluate inital sample
    #     self.EvaluateSample()
    #
    #     # Initialse temperature
    #     self.temperature = initial_temperature
    #
    #     # Main loop - For step k in range 0 : k_max
    #     for k in range(self.k_max):
    #
    #         # Update temperature
    #         self.UpdateTemperature()
    #
    #         # Generate neighbour sample - s'
    #         self.NeighourSample()
    #
    #         # Decide whether or not to accept neighbour
    #
    #
    #         print(1)

    # def AcceptanceProbability(self, s, s_prime, T):
    #
    #     # return np.exp( (solutionEnergy - neighbourEnergy) / temperature
    #     pass
    #
    # def EvaluateSample(self):
    #     pass
    #
    # def NeighbourSample(self):
    #     pass
    #
    # def UpdateTemperature(self):
    #     pass
    #
    # def GenerateSamples(self):
    #     pass