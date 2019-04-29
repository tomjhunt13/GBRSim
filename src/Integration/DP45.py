import numpy as np

from src.Integration.Integrator import Integrator

class DP45(Integrator):

    """
    Using:  http://depa.fquim.unam.mx/amyd/archivero/DormandPrince_19856.pdf
    """

    def __init__(self, dt_min=1e-4, dt_max=1, error_tolerance=0.1):

        self.dt_min = dt_min
        self.dt_max = dt_max
        self.error_tolerance = error_tolerance

        super(DP45, self).__init__()


    def update(self, t_n, y_n, **kwargs):

        info_dictionary = {}

        y_np1, scale = self._step(t_n, y_n, info_dictionary, **kwargs)

        # Update dt
        if not scale:
            self.dt = self.dt_max

        elif scale > 1:

            self.dt = min(self.dt_max, scale * self.dt)

        else:
            self.dt = max(self.dt_min, scale * self.dt)


        return y_np1, info_dictionary


    def _step(self, t_n, y_n, info_total, **kwargs):

        info_1 = {}
        info_2 = {}
        info_3 = {}
        info_4 = {}
        info_5 = {}
        info_6 = {}
        info_7 = {}


        # k_1 = h * f(t_n, y_n)
        k_1 = np.multiply(self.dt, self.f(t_n, y_n, info_1, **kwargs))

        # k_2 = h * f(t_n + dt * 1/5, y_n + k_1 * 1/5)
        k_2 = np.multiply(self.dt, self.f(t_n + self.dt * (1. / 5.), np.add(y_n, np.multiply(1. / 5., k_1)), info_2, **kwargs))

        # k_3 = h * f(t_n + dt * 3/10, y_n + k_1 * 3/40 + k_2 * 9/40)
        k_3 = np.multiply(self.dt, self.f(t_n + self.dt * (3. / 10.),
                                np.add(
                                    y_n,
                                    np.add(
                                        np.multiply(3. / 40., k_1),
                                        np.multiply(9. / 40., k_2))),
                                info_3, **kwargs))

        # k_4 = h * f(t_n + dt * 4/5, y_n + k_1 * 44/45 - k_2 * 56/15 + k_3 * 32/9)
        k_4 = np.multiply(self.dt, self.f(t_n + self.dt * (4. / 5.),
                                np.add(
                                    y_n,
                                    np.add(
                                        np.multiply(44. / 45., k_1),
                                        np.add(
                                            np.multiply(-56. / 15., k_2),
                                            np.multiply(32. / 9., k_3)))),
                                info_4, **kwargs))

        # k_5 = h * f(t_n + dt * 8/9, y_n + k_1 * 19372/6561 - k_2 * 25360 / 2187 + k_3 * 64448/6561 - k_4 * 212/729)
        k_5 = np.multiply(self.dt, self.f(t_n + self.dt * (8. / 9.),
                                np.add(y_n,
                                       np.add(np.multiply(19372. / 6561., k_1),
                                              np.add(np.multiply(-25360. / 2187., k_2),
                                                     np.add(np.multiply(64448. / 6561., k_3),
                                                            np.multiply(-212. / 729., k_4))))),
                                info_5, **kwargs))

        # k_6 = h * f(t_n + dt, y_n + k_1 * 9017/3168 - k_2 * 355/33. - k_3 * 46732/5247 + k_4 * 49/176 - k_5 * 5103/18656)
        k_6 = np.multiply(self.dt, self.f(t_n + self.dt,
                                np.add(y_n,
                                       np.add(np.multiply(9017. / 3168., k_1),
                                              np.add(np.multiply(-355. / 33., k_2),
                                                     np.add(np.multiply(-46732. / 5247., k_3),
                                                            np.add(np.multiply(49. / 176., k_4),
                                                                   np.multiply(-5103. / 18656., k_5)))))),
                                info_6, **kwargs))

        # k_7 = h * f(t_n + dt, y_n + k_1 * 35/384 + k_3 * 500/1113 + k_4 * 125/192 - k_5 * 2187/6784 + k_6 * 11/84 )
        k_7 = np.multiply(self.dt, self.f(t_n + self.dt,
                                          np.add(y_n,
                                                 np.add(np.multiply(35. / 384., k_1),
                                                        np.add(np.multiply(500. / 1113., k_3),
                                                               np.add(np.multiply(125. / 192., k_4),
                                                                      np.add(np.multiply(-2187. / 6784., k_5),
                                                                             np.multiply(11. / 84., k_6)))))),
                                          info_7, **kwargs))

        dy_4, dy_5 = DP45_weighting(k_1, k_2, k_3, k_4, k_5, k_6, k_7)

        y_np1 = np.add(y_n, dy_4)
        z_np1 = np.add(y_n, dy_5)

        error = np.linalg.norm(np.subtract(z_np1, y_np1))

        scale = np.power(((self.error_tolerance * self.dt) / (2 * error)), 1 / 5)

        if self.verbose:
            print('dt: ' + str(self.dt) + ', error: ' + str(error) + ', scale: ' + str(scale))

        for info in info_1.keys():
            info_total[info] = \
                DP45_weighting(info_1[info], info_2[info], info_3[info], info_4[info], info_5[info], info_6[info], info_7[info])[0]

        return y_np1, scale


def DP45_weighting(k_1, k_2, k_3, k_4, k_5, k_6, k_7):

    # Apply weighting 4th order
    k_1_w = np.multiply(k_1, 35. / 384.)
    k_2_w = np.multiply(k_3, 500. / 1113.)
    k_3_w = np.multiply(k_4, 125. / 192.)
    k_4_w = np.multiply(k_5, -2187. / 6784.)
    k_5_w = np.multiply(k_6, 11. / 84.)

    dy_4 = np.add(k_1_w, np.add(k_2_w, np.add(k_3_w, np.add(k_4_w, k_5_w))))

    # Apply weighting 5th order
    k_1_w = np.multiply(k_1, 5179. / 57600.)
    k_2_w = np.multiply(k_3, 7571. / 16695.)
    k_3_w = np.multiply(k_4, 393. / 640.)
    k_4_w = np.multiply(k_5, -92097. / 339200.)
    k_5_w = np.multiply(k_6, 187. / 2100.)
    k_6_w = np.multiply(k_7, 1. / 40.)

    dy_5 = np.add(k_1_w, np.add(k_2_w, np.add(k_3_w, np.add(k_4_w, np.add(k_5_w, k_6_w)))))

    return dy_4, dy_5
