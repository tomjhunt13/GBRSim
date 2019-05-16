import numpy as np

from src.Integration.Integrator import Integrator

class RK4(Integrator):

    def __init__(self):

        super(RK4, self).__init__()


    def update(self, t_n, y_n, **kwargs):

        info_dictionary = {}

        return self._step(t_n, y_n, info_dictionary, **kwargs), info_dictionary


    def _step(self, t_n, y_n, info_total, **kwargs):

        info_1 = {}
        info_2 = {}
        info_3 = {}
        info_4 = {}

        # Runge Kutta Step
        k_1 = np.multiply(self.dt, self.f(t_n, y_n, info_1, **kwargs))
        k_2 = np.multiply(self.dt, self.f(t_n + (self.dt / 2.0), np.add(y_n, np.multiply((0.5), k_1)), info_2, kwargs=kwargs))
        k_3 = np.multiply(self.dt, self.f(t_n + (self.dt / 2.0), np.add(y_n, np.multiply((0.5), k_2)), info_3, kwargs=kwargs))
        k_4 = np.multiply(self.dt, self.f(t_n + self.dt, np.add(y_n, k_3), info_4, kwargs=kwargs))

        dy_4 = RK4_weighting(k_1, k_2, k_3, k_4)
        y_np1 = np.add(y_n, dy_4)

        for info in info_1.keys():
            info_total[info] = \
                RK4_weighting(info_1[info], info_2[info], info_3[info], info_4[info])

        return y_np1


def RK4_weighting(k_1, k_2, k_3, k_4):

    # Apply weighting 4th order
    k_1_w = np.multiply(k_1, 1. / 6.)
    k_2_w = np.multiply(k_2, 2. / 6.)
    k_3_w = np.multiply(k_3, 2. / 6.)
    k_4_w = np.multiply(k_4, 1. / 6.)

    dy_4 = np.add(k_1_w, np.add(k_2_w, np.add(k_3_w, k_4_w)))

    return dy_4


def RK4_step(f, t_n, y_n, dt, info_total, **kwargs):

    info_1 = {}
    info_2 = {}
    info_3 = {}
    info_4 = {}

    # Runge Kutta Step
    k_1 = np.multiply(dt, f(t_n, y_n, info_1, kwargs=kwargs))
    k_2 = np.multiply(dt, f(t_n + (dt / 2.0), np.add(y_n, np.multiply((0.5), k_1)), info_2, kwargs=kwargs))
    k_3 = np.multiply(dt, f(t_n + (dt / 2.0), np.add(y_n, np.multiply((0.5), k_2)), info_3, kwargs=kwargs))
    k_4 = np.multiply(dt, f(t_n + dt, np.add(y_n, k_3), info_4, kwargs=kwargs))

    # Apply weighting
    k_1_w = np.multiply(k_1, 1 / 6)
    k_2_w = np.multiply(k_2, 1 / 3)
    k_3_w = np.multiply(k_3, 1 / 3)
    k_4_w = np.multiply(k_4, 1 / 6)

    y_np1 = np.add(y_n, np.add(k_1_w, np.add(k_2_w, np.add(k_3_w, k_4_w))))

    for info in info_1.keys():
        info_total[info] = RK_weighting(info_1[info], info_2[info], info_3[info], info_4[info])

    return y_np1

def RK_weighting(k_1, k_2, k_3, k_4):

    return (1 / 6) * (k_1 + k_4 + 2 * (k_2 + k_3))
