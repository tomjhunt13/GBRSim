class VariableManager:
    def __init__(self, verbose=True):

        """
        Attributes:

        self.variables - List of dictionaries of form {'name': Name, 'var': Reference to , 'min': Minimum Value, 'max': Max Value}
        self.variables_name_hash -
        """
        #  is a list so it maintains order
        self.variables = []
        self.default_parameters = {}
        self.verbose = verbose

    def add_variable(self, name, variable_ref, min, max):

        self.variables.append({'name': name, 'var': variable_ref, 'min': min, 'max': max})

    def evaluate_cost(self, input_vector):

        self._update_parameters(input_vector)

        return self.cost_function()

    def _update_parameters(self, input_vector):
        """
        For each variable, update
        :return:
        """

        for index, variable in enumerate(self.variables):

            if self.verbose:
                print(variable['name'] + ': ' + str(input_vector[index]))
            variable['var'][0] = input_vector[index]

    def _assemble_input_vector(self):

        sensitivity_input = [None] * len(self.variables)

        for index, variable in enumerate(self.variables):
            sensitivity_input[index] = variable['var'][0]

        return sensitivity_input


