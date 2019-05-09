from src.Integration import RKF45, RK4, DP45, Butcher, Euler

class Model:

    def __init__(self):

        pass

    def simulate(self, initial_conditions, dt=0.25, t_start=0, t_end=1, solver=RK4.RK4, verbose=True, **model_arguments):

        solver_instance = solver()
        solution = solver_instance.solve(self,
                                         self.update_equation,
                                         initial_conditions,
                                         model_arguments,
                                         dt=dt,
                                         t_start=t_start,
                                         t_end=t_end,
                                         verbose=verbose)

        return solution

    def update_equation(self, t, state, information_dictionary, **kwargs):

        pass

    def end_condition(self):

        return True

    def pre_step(self, t_n, y_n):

        pass

    def post_step(self, t_np1, y_np1, information_dictionary):

        pass

    def initialise(self, initial_conditions, information_dictionary, **kwargs):

        pass
