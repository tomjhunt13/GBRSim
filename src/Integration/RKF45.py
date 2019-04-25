import numpy as np

class RKF45:

    def __init__(self, dt_min=1e-4, dt_max=1, error_tolerance=0.1):

        self.dt_min = dt_min
        self.dt_max = dt_max
        self.error_tolerance = error_tolerance

    def solve(self, model, model_update_function, model_kwargs, initial_conditions, dt=0.01, t_end=500):

        # Initialise state space
        self.f = model_update_function
        self.y = [initial_conditions]
        self.t = [0]
        self.dt = dt
        self.t_end = t_end
        self.information_dictionary = [{}]

        # Initialise model
        self.model = model
        self.model.initialise(initial_conditions, self.information_dictionary[0], kwargs=model_kwargs)

        # Simulation loop
        while self.end_condition():

            # Pre-step processing
            self.pre_step()

            # Step
            y_np1, t_np1 = self.update()
            t_n = self.t[-1]
            y_n = self.y[-1]

            t_np1 = t_n + self.dt
            y_np1, dictionary_t_np1 = self.update(t_n, y_n)

            # Post-step processing
            self.model.post_step(t_np1, y_np1, dictionary_t_np1)

            # Update state
            self.t.append(t_np1)
            self.y.append(y_np1)
            self.information_dictionary.append(dictionary_t_np1)

        # Update info dict
        update_dictionary_keys(self.info_dict[1], self.info_dict[0])

        return self.info_dict

    def end_condition(self):

        if self.t[-1] >= self.t_end:
            return False

        return self.model.end_conition()

    def pre_step(self):
        pass


    def update(self, t_n, y_n, t_np1, **kwargs):

        # t_n = self.t[-1]
        # self._step_y_n = y_n

        info_dictionary = {}

        y_np1, scale = self._step(t_n, y_n, info_dictionary)

        # Update dt
        if not scale:
            self.dt = self.h_max

        elif scale > 1:

            self.dt = min(self.h_max, scale * self.dt)

        else:
            self.dt = max(self.h_min, scale * self.dt)


        return y_np1, info_dictionary


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


def update_dictionary_keys(full_dictionary, destination_dictionary):
    """
    Updates destination dictionary with keys its missing from full dictionary
    :param full_dictionary:
    :param destination_dictionary:
    :return:
    """

    for key in full_dictionary.keys():
        if key not in destination_dictionary.keys():

            destination_dictionary[key] = 0

