import numpy as np

from src.Model import Model

class IVP(Model.Model):

    def initialise(self, a, b, **model_kwargs):
        pass

    def post_step(self, t_np1, y_np1, dictionary_t_np1):
        dictionary_t_np1['t'] = t_np1
        dictionary_t_np1['y'] = y_np1

    def end_condition(self):
        return True

    def wikipedia_f(self, t, y, information_dictionary, **kwargs):
        "Example from: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods"
        return [np.tan(y) + 1]


    def rk4_test_func(self, t,  y, information_dictionary, **kwargs):
        "Example from: https://www.intmath.com/differential-equations/12-runge-kutta-rk4-des.php"
        return [(5 * t * t - y[0]) / np.exp(t + y[0])]

    def separation_of_variables_grad(self, t, y, information_dictionary, **kwargs):
        """
        dy / dt = y
        """

        return y

    def separation_of_variables_analytical(self, t, y_0):
        """
        dy / dt = y
        """

        return float(np.exp(t) * y_0)