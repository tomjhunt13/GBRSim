import numpy as np

from src.Integration import Integrator

class Butcher(Integrator.Integrator):

    def __init__(self, h=None, A=None, b=None):

        self.h = h
        self.A = A
        self.b = b

        if not self.h and not self.A and not self.b:

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

                if coefficient != 0.:
                    y_n_stage = np.add(y_n_stage, np.multiply(coefficient, k[coefficient_index]))

            k[index] = np.multiply(self.dt, self.f(t_n_stage, y_n_stage, info[index], **kwargs))

        dy = self.apply_weighting(k)
        y_np1 = np.add(y_n, dy)

        for info_key in info[0].keys():
            info_total[info_key] = self.apply_weighting([info[index][info_key] for index in range(number_of_stages)])

        return y_np1


    def apply_weighting(self, input_vector):

        k_weighted = [None] * len(self.b)

        # For each element of input vector apply weighting
        for coefficient_index, coefficient in enumerate(self.b):

            k_weighted[coefficient_index] = np.multiply(coefficient, input_vector[coefficient_index])

        dy = k_weighted[0]
        for k in k_weighted:
            dy = np.add(dy, k)
        return dy


class RK4(Butcher):

    def __init__(self):
        super(RK4, self).__init__()


class RK8(Butcher):

    def __init__(self):

        """
        Reference: https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19760017203.pdf
        """

        h = [0,
             4. / 27.,
             2. / 9.,
             1. / 3.,
             1. / 2.,
             2. / 3.,
             1. / 6.,
             1,
             5. / 6.,
             1]

        A = [[],
             [4. / 27.],
             [1. / 18., 3. / 18.],
             [1. / 12., 0, 3. / 12.],
             [1. / 8., 0, 0, 3. / 8.],
             [13. / 54, 0, -27. / 54., 42. / 54., 8. / 54.],
             [389. / 4320., 0, -54. / 4320., 966. / 4320., -824. / 4320., 243. / 4320.],
             [-231. / 20., 0, 81. / 20., -1164. / 20., 656. / 20., -122. / 20., 800. / 20.],
             [-127. / 288., 0, 18. / 288., -678. / 288., 456. / 288., -9. / 288., 576. / 288., 4. / 288.],
             [1481. / 820., 0, -81. / 820., 7104. / 820., -3376. / 820., 72. / 820., -5040. / 820., -60. / 820., 720. / 820.]]

        b = np.multiply(1. / 840., [41., 0, 0, 27., 272., 27., 216., 0, 216., 41.])

        super(RK8, self).__init__(h=h, A=A, b=b)
