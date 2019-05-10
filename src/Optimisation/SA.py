from scipy.optimize import dual_annealing
from src.Optimisation import Optimiser

class SA(Optimiser.Optimiser):

    def __init__(self, verbose=True):

        super(SA, self).__init__(verbose=verbose)

        self.default_parameters = {'max_iterations': 1, 'initial_temperature': 5230}

    def _optimise(self, **kwargs):

        bounds = self._assemble_bounds_vector()
        optimisation_result = dual_annealing(self.evaluate_cost, bounds, maxiter=kwargs['max_iterations'], initial_temp=kwargs['initial_temperature'])

        return optimisation_result['x']