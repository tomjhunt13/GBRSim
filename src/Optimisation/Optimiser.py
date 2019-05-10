from src.Optimisation import VariableManager

class Optimiser(VariableManager.VariableManager):
    def __init__(self, verbose=True):

        super(Optimiser, self).__init__(verbose=verbose)

    def AddVariable(self, name, variable_ref, min, max):

        self.variables.append({'name': name, 'var': variable_ref, 'min': min, 'max': max})

    def Optimise(self, cost_function, **kwargs):

        self.cost_function = cost_function

        for key in self.default_parameters.keys():
            if key not in kwargs.keys():
                kwargs[key] = self.default_parameters[key]

        optimisation_result = self._optimise(**kwargs)

        result = {}
        for index, value in enumerate(optimisation_result):
            result[self.variables[index]['name']] = value

        if self.verbose:
            print('Optimised Values: ' + str(result))

        return result

    def _optimise(self, **kwargs):

        pass