import numpy as np

from scipy.optimize import minimize


class Optimiser:
    def __init__(self):

        """
        Attributes:

        self.variables - List of dictionaries of form {'name': Name, 'var': Reference to , 'min': Minimum Value, 'max': Max Value}
        self.variables_name_hash -
        """
        #  is a list so it maintains order
        self.variables = []
        # self.variable_name_hash = {}

    def AddVariable(self, name, variable_ref, min, max):

        self.variables.append({'name': name, 'var': variable_ref, 'min': min, 'max': max})
        # self.variable_name_hash['name'] = len(self.variables) - 1


    def Optimise(self, cost_function, tolerance=1e-7):

        self.cost_function = cost_function

        input = self._assemble_input_vector()

        optimisation_result = minimize(self._cost, input, method='Nelder-Mead', tol=tolerance)['x']

        result = {}
        for index, value in enumerate(optimisation_result):
            result[self.variables[index]['name']] = value

        return result

    def _cost(self, input_vector):

        self._update_parameters(input_vector)

        return self.cost_function()

    def _assemble_input_vector(self):

        """
        for each variable:

        """

        input = [None] * len(self.variables)

        for index, variable in enumerate(self.variables):
            input[index] = variable['min'] + np.random.rand(1)[0] * (variable['max'] - variable['min'])

        return input


    def _update_parameters(self, input_vector):
        """
        For each variable, update
        :return:
        """

        for index, variable in enumerate(self.variables):

            print(variable['name'] + ': ' + str(input_vector[index]))
            variable['var'][0] = input_vector[index]

            a = 56


