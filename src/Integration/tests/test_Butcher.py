import numpy as np
import unittest

from src.Integration import Butcher


class IVP:

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
        return (5 * t * t - y[0]) / np.exp(t + y[0])


def mass_spring(t, y, info_dict, **kwargs):

    m = 1
    g = 9.81
    k = 20

    return [y[1], (-1/m) * k * y[0] + g]

def internet_IVP(t, y, info_dict, **kwargs):
    return [1 + y[0] * y[0]]


class TestButcher(unittest.TestCase):

    def test_wikipedia_IVP(self):

        ivp = IVP()

        s = Butcher.Butcher(A=[[], [2. / 3.]],  b=[1. / 4., 3. / 4.], h=[0, 2. / 3.])

        results = s.solve(ivp, ivp.wikipedia_f, {}, [1], dt=0.025, t_start=1, t_end=1.1)

        self.assertAlmostEqual(results[1]['y'][0], 1.066869388)

    def test_RK4_func_IVP(self):
        ivp = IVP()

        s = Butcher.Butcher()

        results = s.solve(ivp, ivp.rk4_test_func, {}, [1], dt=0.1, t_start=0, t_end=1)

        self.assertAlmostEqual(results[1]['y'][0], 0.9655827899)






if __name__ == '__main__':
    unittest.main()