import numpy as np
"""
Problem:

y' = f(y, t)


k_1 = h * f(t_n, y_n)
k_2 = h * f(t_n + h/2, y_n + k_1/2)
k_3 = h * f(t_n + h/2, y_n + k_2/2)
k_4 = h * f(t_n + h, y_n + k_3)

y_n+1 = y_n + 1/6 * (k_1 + 2 * k_2 + 2 * k_3 + k_4)
t_n+1 = t + h


"""


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
    k_6 = np.multiply(dt, f(t_n + dt (1. / 2.),
                            np.add(y_n,
                                   np.add(np.multiply(-8. / 27., k_1),
                                          np.add(np.multiply(2., k_2),
                                                 np.add(np.multiply(-3544. / 2565., k_3),
                                                        np.add(np.multiply(1859. / 4104., k_4),
                                                               np.multiply(-11. / 40., k_5)))))),
                            info_6, kwargs=kwargs))

    # Apply weighting
    k_1_w = np.multiply(k_1, 1 / 6)
    k_2_w = np.multiply(k_2, 1 / 3)
    k_3_w = np.multiply(k_3, 1 / 3)
    k_4_w = np.multiply(k_4, 1 / 6)

    y_np1 = np.add(y_n, np.add(k_1_w, np.add(k_2_w, np.add(k_3_w, k_4_w))))

    for info in info_1.keys():
        info_total[info] = RK_weighting(info_1[info], info_2[info], info_3[info], info_4[info])

    return y_np1

def RKF45_weighting(k_1, k_2, k_3, k_4, k_5, k_6):

    # Apply weighting 4th order
    k_1_w = np.multiply(k_1, 25. / 216.)
    k_2_w = np.multiply(k_3, 1408. / 2565.)
    k_3_w = np.multiply(k_4, 2197. / 4101.)
    k_4_w = np.multiply(k_5, -1. / 5.)

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
