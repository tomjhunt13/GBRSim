from src.Optimisation import VariableManager

class Optimiser(VariableManager.VariableManager):
    def __init__(self, verbose=True):

        super(Optimiser, self).__init__(verbose=verbose)

        self.optimum = None

    def optimise(self, cost_function, **kwargs):

        self.cost_function = cost_function

        for key in self.default_parameters.keys():
            if key not in kwargs.keys():
                kwargs[key] = self.default_parameters[key]

        optimisation_result = self._optimise(**kwargs)

        result = self._unpack_vector(optimisation_result)

        if self.verbose:
            print('Optimised Values: ' + str(result))

        self.optimum = result

        return result

    def _optimise(self, **kwargs):

        pass

    def _assemble_bounds_vector(self):

        bounds = [None] * len(self.variables)
        for index, var in enumerate(self.variables):
            bounds[index] = (var['min'], var['max'])

        return bounds