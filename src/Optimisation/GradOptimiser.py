from scipy.optimize import minimize

from src.Optimisation import Optimiser

class GradOptimiser(Optimiser.Optimiser):
    def __init__(self, verbose=True):

        super(GradOptimiser, self).__init__(verbose=verbose)

        self.default_parameters = {'tolerance': 1e-7}

    def _optimise(self, **kwargs):

        minimisation_input = self._assemble_input_vector()
        bounds = self._assemble_bounds_vector()
        optimisation_result = minimize(self.evaluate_cost,
                                       minimisation_input,
                                       tol=kwargs['tolerance'],
                                       bounds=bounds)

        return optimisation_result['x']