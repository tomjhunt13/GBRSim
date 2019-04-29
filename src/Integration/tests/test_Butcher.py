import numpy as np
import unittest

from src.Integration import Butcher, RK4

def wikipedia_f(t, y, information_dictionary, **kwargs):
    "Example from: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods"

    return [np.tan(y) + 1]


def mass_spring(t, y, info_dict, **kwargs):

    m = 1
    g = 9.81
    k = 20

    return [y[1], (-1/m) * k * y[0] + g]

def internet_IVP(t, y, info_dict, **kwargs):
    return [1 + y[0] * y[0]]


class TestButcher(unittest.TestCase):

    def test_wikipedia_IVP(self):

        s = Butcher.Butcher(A=[[], [2. / 3.]],  b=[1. / 4., 3. / 4.], h=[0, 2. / 3.])
        s.f = wikipedia_f
        s.dt = 0.025

        y_np1 = s._step(0, [0], {})




        k_1 = 1



if __name__ == '__main__':
    unittest.main()