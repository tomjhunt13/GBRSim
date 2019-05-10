from scipy.optimize import approx_fprime
from src.Optimisation import VariableManager

class Sensitivity(VariableManager.VariableManager):
    def __init__(self, verbose=True, dx=1e-3):

        super(Sensitivity, self).__init__(verbose=verbose)

        self.dx = dx

    def add_variable(self, name, variable_ref, dx=None):

        if not dx:
            dx = self.dx

        self.variables.append({'name': name, 'var': variable_ref, 'dx': [dx]})

    def sensitivity(self, cost_function, **kwargs):

        self.cost_function = cost_function

        for key in self.default_parameters.keys():
            if key not in kwargs.keys():
                kwargs[key] = self.default_parameters[key]

        input_vector = self._assemble_input_vector()
        dx_vector = self._assemble_input_vector(key='dx')

        gradient = approx_fprime(input_vector, self.evaluate_cost, dx_vector)

        sensitivity = self._unpack_vector(gradient)

        if self.verbose:
            print('Gradient: ' + str(sensitivity))

        return sensitivity


