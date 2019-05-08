import numpy as np

from src.Track.Segment import *

class Sinusoidal(Segment):
    def __init__(self, x_scale, frequency, amplitude):
        self.x_scale = x_scale
        self.frequency = frequency
        self.amplitude = amplitude

        super(Sinusoidal, self).__init__()

        self.length = self._length()

    def direction(self, t):
        """

        :param t:
        :return:
        """

        return list(np.divide(self._AB(), self._length()))

    def position(self, t):
        """

        :param t:
        :return:
        """
        return list(np.add(self.coordinates[0], np.multiply(t, self._AB())))

    def radius_of_curvature(self, t):
        return 1e8

    def df_dlambda(self, lambda_parameter):

        df_dlambda = [self.x_scale, self.frequency * self.amplitude * np.cos(self.frequency * lambda_parameter)]

        return df_dlambda

    def d_dx_dlambda_dt(self, state):
        return 0

    def _length(self):
        return np.linalg.norm(self._AB())

    def _AB(self):
        return list(np.subtract(self.coordinates[1], self.coordinates[0]))
