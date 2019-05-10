from scipy.optimize import approx_fprime
from src.Optimisation import VariableManager

class Sensitivity(VariableManager.VariableManager):
    def __init__(self, verbose=True):

        super(Sensitivity, self).__init__(verbose=verbose)

    def sensitivity(self, cost_function, **kwargs):

        self.cost_function = cost_function

        for key in self.default_parameters.keys():
            if key not in kwargs.keys():
                kwargs[key] = self.default_parameters[key]

        gradient = approx_fprime(self._assemble_input_vector(), self.evaluate_cost, 0.001)

        a = 6

    # def _assemble_input_vector(self):
    #
    #     sensitivity_input = [None] * len(self.variables)
    #
    #     for index, variable in enumerate(self.variables):
    #         sensitivity_input[index] = variable['var'][0]
    #
    #     return sensitivity_input


