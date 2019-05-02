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

    def Optimise(self, cost_function):

        pass

    def _cost(self, input_vector):

        self._update_parameters(input_vector)

        return self.cost_function()

    def _update_parameters(self, input_vector):
        """
        For each variable, update
        :return:
        """

        for index, variable in enumerate(self.variables):

            print(variable['name'] + ': ' + str(input_vector[index]))
            variable['var'][0] = input_vector[index]


