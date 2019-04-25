import numpy as np

class RKF45:

    def __init__(self, f, h_initial=0.0001):

        self.h_initial = h_initial
        self.h_min = 1e-6
        self.h_max = 1

        self.error_tolerance = 0.1

        self.f = f
        self.reset()

    def reset(self):
        self.t_latest = 0
        self.h = self.h_initial

    def update(self, t_n, y_n, t_np1, info_total, **kwargs):

        # Update f to time t_np1
        dt = t_np1 - t_n

        if self.h == None:
            self.h = min(dt, self.h_min)

        if self.h > dt:
            self.h = dt

        # # t_latest = t_n
        # t_remaining = dt
        y_latest = y_n

        # Step
        while self.t_latest < t_np1:

            print(self.t_latest)

            if self.t_latest + self.h >= t_np1:
                self.h = t_np1 - self.t_latest

            # Step f
            y_latest = self._inner_update(y_latest, info_total)

        print(self.h)

        return y_latest

    def _inner_update(self, y_latest, info_total, **kwargs):


        y_next, scale = self._step(self.t_latest, y_latest, info_total, kwargs=kwargs)

        self.t_latest += self.h

        # Update h
        if not scale:
            self.h = self.h_max

        elif scale > 1:

            self.h = min(self.h_max, scale * self.h)

        else:
            self.h = max(self.h_min, scale * self.h)

        return y_next



    def _step(self, t_n, y_n, info_total, **kwargs):

        kwargs_unwrapped = kwargs['kwargs']

        info_1 = {}
        info_2 = {}
        info_3 = {}
        info_4 = {}
        info_5 = {}
        info_6 = {}

        # Runge Kutta Step

        # k_1 = h * f(t_n, y_n)
        k_1 = np.multiply(self.h, self.f(t_n, y_n, info_1, kwargs=kwargs_unwrapped))

        # k_2 = h * f(t_n + dt * 1/4, y_n + k_1 * 1/4)
        k_2 = np.multiply(self.h, self.f(t_n + self.h * (1. / 4.), np.add(y_n, np.multiply(1. / 4., k_1)), info_2, kwargs=kwargs_unwrapped))

        # k_3 = h * f(t_n + dt * 3/8, y_n + k_1 * 3/32 + k_2 * 9/32)
        k_3 = np.multiply(self.h, self.f(t_n + self.h * (3. / 8.),
                                np.add(
                                    y_n,
                                    np.add(
                                        np.multiply(3. / 32., k_1),
                                        np.multiply(9. / 32., k_2))),
                                info_3, kwargs=kwargs_unwrapped))

        # k_4 = h * f(t_n + dt * 12/13, y_n + k_1 * 1932/2197 - k_2 * 7200/2197 + k_3 * 7296/2197)
        k_4 = np.multiply(self.h, self.f(t_n + self.h * (12. / 13.),
                                np.add(
                                    y_n,
                                    np.add(
                                        np.multiply(1932. / 2197., k_1),
                                        np.add(
                                            np.multiply(-7200. / 2197., k_2),
                                            np.multiply(7296. / 2197., k_3)))),
                                info_4, kwargs=kwargs_unwrapped))

        # k_5 = h * f(t_n + dt, y_n + k_1 * 439/216 - k_2 * 8 + k_3 * 3680/513 - k_4 * 845/4104)
        k_5 = np.multiply(self.h, self.f(t_n + self.h,
                                np.add(y_n,
                                       np.add(np.multiply(439. / 216., k_1),
                                              np.add(np.multiply(-8., k_2),
                                                     np.add(np.multiply(3680. / 513., k_3),
                                                            np.multiply(-845. / 4104., k_4))))),
                                info_5, kwargs=kwargs_unwrapped))

        # k_6 = h * f(t_n + dt * 1/2, y_n - k_1 * 8/27 + k_2 * 2 - k_3 * 3544/2565 + k_4 * 1859/4104 - k_5 * 11/40)
        k_6 = np.multiply(self.h, self.f(t_n + self.h * (1. / 2.),
                                np.add(y_n,
                                       np.add(np.multiply(-8. / 27., k_1),
                                              np.add(np.multiply(2., k_2),
                                                     np.add(np.multiply(-3544. / 2565., k_3),
                                                            np.add(np.multiply(1859. / 4104., k_4),
                                                                   np.multiply(-11. / 40., k_5)))))),
                                info_6, kwargs=kwargs_unwrapped))

        dy_4, dy_5 = RKF45_weighting(k_1, k_2, k_3, k_4, k_5, k_6)

        y_np1 = np.add(y_n, dy_4)
        z_np1 = np.add(y_n, dy_5)

        error = np.linalg.norm(np.subtract(z_np1, y_np1))

        scale = np.power(((self.error_tolerance * self.h) / (2 * error)), 1 / 4)

        # print(scale)

        for info in info_1.keys():
            info_total[info] = \
                RKF45_weighting(info_1[info], info_2[info], info_3[info], info_4[info], info_5[info], info_6[info])[0]

        return y_np1, scale




def RKF45_step(f, t_n, y_n, dt, info_total, **kwargs):
    """
    http://maths.cnam.fr/IMG/pdf/RungeKuttaFehlbergProof.pdf
    :param f:
    :param t_n:
    :param y_n:
    :param dt:
    :param info_total:
    :param kwargs:
    :return:
    """

    info_1 = {}
    info_2 = {}
    info_3 = {}
    info_4 = {}
    info_5 = {}
    info_6 = {}

    # Runge Kutta Step

    # k_1 = h * f(t_n, y_n)
    k_1 = np.multiply(dt, f(t_n, y_n, info_1, kwargs=kwargs))

    # k_2 = h * f(t_n + dt * 1/4, y_n + k_1 * 1/4)
    k_2 = np.multiply(dt, f(t_n + dt * (1. / 4.), np.add(y_n, np.multiply(1. / 4., k_1)), info_2, kwargs=kwargs))

    # k_3 = h * f(t_n + dt * 3/8, y_n + k_1 * 3/32 + k_2 * 9/32)
    k_3 = np.multiply(dt, f(t_n + dt * (3. / 8.),
                            np.add(
                                y_n,
                                np.add(
                                    np.multiply(3. / 32., k_1),
                                    np.multiply(9. /  32., k_2))),
                            info_3, kwargs=kwargs))

    # k_4 = h * f(t_n + dt * 12/13, y_n + k_1 * 1932/2197 - k_2 * 7200/2197 + k_3 * 7296/2197)
    k_4 = np.multiply(dt, f(t_n + dt * (12. / 13.),
                            np.add(
                                y_n,
                                np.add(
                                    np.multiply(1932. / 2197., k_1),
                                    np.add(
                                        np.multiply(-7200. / 2197., k_2),
                                        np.multiply(7296. / 2197., k_3)))),
                            info_4, kwargs=kwargs))

    # k_5 = h * f(t_n + dt, y_n + k_1 * 439/216 - k_2 * 8 + k_3 * 3680/513 - k_4 * 845/4104)
    k_5 = np.multiply(dt, f(t_n + dt,
                            np.add(y_n,
                                   np.add(np.multiply(439. / 216., k_1),
                                          np.add(np.multiply(-8., k_2),
                                                 np.add(np.multiply(3680. / 513., k_3),
                                                        np.multiply(-845. / 4104., k_4))))),
                            info_5, kwargs=kwargs))

    # k_6 = h * f(t_n + dt * 1/2, y_n - k_1 * 8/27 + k_2 * 2 - k_3 * 3544/2565 + k_4 * 1859/4104 - k_5 * 11/40)
    k_6 = np.multiply(dt, f(t_n + dt * (1. / 2.),
                            np.add(y_n,
                                   np.add(np.multiply(-8. / 27., k_1),
                                          np.add(np.multiply(2., k_2),
                                                 np.add(np.multiply(-3544. / 2565., k_3),
                                                        np.add(np.multiply(1859. / 4104., k_4),
                                                               np.multiply(-11. / 40., k_5)))))),
                            info_6, kwargs=kwargs))

    dy_4, dy_5 = RKF45_weighting(k_1, k_2, k_3, k_4, k_5, k_6)

    y_np1 = np.add(y_n, dy_4)
    z_np1 = np.add(y_n, dy_5)


    error = np.linalg.norm(np.subtract(z_np1, y_np1))

    tolerance = 0.01
    scale = np.power(((tolerance * dt) / (2 * error)), 1/4)

    print(scale)




    for info in info_1.keys():
        info_total[info] = RKF45_weighting(info_1[info], info_2[info], info_3[info], info_4[info], info_5[info], info_6[info])[0]

    return y_np1

def RKF45_weighting(k_1, k_2, k_3, k_4, k_5, k_6):

    # Apply weighting 4th order
    k_1_w = np.multiply(k_1, 25. / 216.)
    k_2_w = np.multiply(k_3, 1408. / 2565.)
    k_3_w = np.multiply(k_4, 2197. / 4101.)
    k_4_w = np.multiply(k_5, -1. / 5.)

    dy_4 = np.add(k_1_w, np.add(k_2_w, np.add(k_3_w, k_4_w)))

    # Apply weighting 5th order
    k_1_w = np.multiply(k_1, 16. / 135.)
    k_2_w = np.multiply(k_3, 6656. / 12825.)
    k_3_w = np.multiply(k_4, 28561. / 56430.)
    k_4_w = np.multiply(k_5, -9. / 50.)
    k_5_w = np.multiply(k_5, 2. / 55.)

    dy_5 = np.add(k_1_w, np.add(k_2_w, np.add(k_3_w, np.add(k_4_w, k_5_w))))

    return dy_4, dy_5

