import numpy as np

from src.Integration import Integrator

class Butcher(Integrator.Integrator):

    def __init__(self, h=None, A=None, b=None):

        if h and A and b:
            self.h = h
            self.A = A
            self.b = b

        else:

            self.h = [0, 1./2., 1./2., 1]
            self.A = [[], [1./2.], [0, 1./2.], [0, 0, 1]]
            self.b = [1. / 6., 2. / 6., 2. / 6., 1. / 6.]

        super(Butcher, self).__init__()

    def update(self, t_n, y_n, **kwargs):
        info_dictionary = {}

        return self._step(t_n, y_n, info_dictionary, **kwargs), info_dictionary

    def _step(self, t_n, y_n, info_total, **kwargs):

        number_of_stages = len(self.h)
        info = [None] * number_of_stages
        k = [None] * number_of_stages

        for index in range(number_of_stages):
            info[index] = {}


            t_n_stage = t_n + self.dt * self.h[index]
            y_n_stage = y_n
            for coefficient_index, coefficient in enumerate(self.A[index]):
                y_n_stage = np.add(y_n_stage, np.multiply(coefficient, k[coefficient_index]))

            k[index] = np.multiply(self.dt, self.f(t_n_stage, y_n_stage, info[index], **kwargs))

        dy = self.apply_weighting(k)
        y_np1 = np.add(y_n, dy)

        for info_key in info[0].keys():
            info_list = [info[index][info_key] for index in range(number_of_stages)]
            info_total[info_key] = self.apply_weighting([info[index][info_key] for index in range(number_of_stages)])

        return y_np1


    def apply_weighting(self, input_vector):

        # dy = [0] * len(input_vector[0])

        k_weighted = [None] * len(self.b)
        # For each element of input vector apply weighting

        for coefficient_index, coefficient in enumerate(self.b):

            k_weighted[coefficient_index] = np.multiply(coefficient, input_vector[coefficient_index])

        dy = np.sum(k_weighted)

            # dy = np.add(dy, np.multiply(coefficient, input_vector[coefficient_index]))

        return dy