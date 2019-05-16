import numpy as np


import numpy as np

from src.Integration.Integrator import Integrator

class Euler(Integrator):

    def __init__(self):

        super(Euler, self).__init__()


    def update(self, t_n, y_n, **kwargs):

        info_dictionary = {}

        return self._step(t_n, y_n, info_dictionary, **kwargs), info_dictionary


    def _step(self, t_n, y_n, info_total, **kwargs):

        info_1 = {}
        dy = np.multiply(self.dt, self.f(t_n, y_n, info_1, **kwargs))
        y_np1 = np.add(y_n, dy)

        for info in info_1.keys():
            info_total[info] = info_1[info]

        return y_np1