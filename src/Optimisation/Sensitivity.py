from scipy.optimize import approx_fprime
from src.Optimisation import VariableManager

class Sensitivity(VariableManager.VariableManager):
    def __init__(self, verbose=True):

        super(Sensitivity, self).__init__(verbose=verbose)

    def add_variable(self, name, variable_ref):

        self.variables.append({'name': name, 'var': variable_ref})

    def sensitivity(self, cost_function, **kwargs):

        self.cost_function = cost_function

        for key in self.default_parameters.keys():
            if key not in kwargs.keys():
                kwargs[key] = self.default_parameters[key]

        input_vector = self._assemble_input_vector()

        gradient = approx_fprime(input_vector, self.evaluate_cost, 0.001)

        a = 6

