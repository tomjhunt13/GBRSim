import numpy as np

from src.Track.Segment import *


class HorizontalCircle(Segment):
    def __init__(self, radius, **segment_options):
        self.radius = radius

        super(HorizontalCircle, self).__init__(**segment_options)

        self.length = 2 * np.pi * radius

    def direction(self, lambda_parameter):
        """

        :param lambda_parameter:
        :return:
        """

        # Get first derivative
        d = self.df_dlambda(lambda_parameter)

        return np.multiply((1 / np.linalg.norm(d)), d)

    def gradient(self, lambda_parameter):
        return 0

    def position(self, lambda_parameter):
        """

        :param t:
        :return:
        """

        P = [self.radius * np.cos(lambda_parameter * 2 * np.pi),
             self.radius * np.sin(lambda_parameter * 2 * np.pi),
             0]

        return P

    def radius_of_curvature(self, lambda_parameter):
        return self.radius

    def df_dlambda(self, lambda_parameter):

        dx_dlambda = [-2 * np.pi * self.radius * np.sin(lambda_parameter * 2 * np.pi),
                      2 * np.pi * self.radius * np.cos(lambda_parameter * 2 * np.pi),
                      0]

        return dx_dlambda

    def d_dx_dlambda_dt(self, state):

        d_dx_dlambda_dt = [-4 * np.pi * np.pi * self.radius * state[1] * np.cos(state[0] * 2 * np.pi),
                           -4 * np.pi * np.pi * self.radius * state[1] * np.sin(state[0] * 2 * np.pi),
                           0]

        return d_dx_dlambda_dt

    def _length(self):
        return 2 * np.pi * self.radius


class VerticalCircle(Segment):
    def __init__(self, radius, **segment_options):
        self.radius = radius

        super(VerticalCircle, self).__init__(**segment_options)

        self.length = 2 * np.pi * radius

    def direction(self, lambda_parameter):
        """

        :param lambda_parameter:
        :return:
        """

        # Get first derivative
        d = self.df_dlambda(lambda_parameter)

        return np.multiply((1 / np.linalg.norm(d)), d)

    def position(self, lambda_parameter):
        """

        :param t:
        :return:
        """

        P = [self.radius * np.cos(lambda_parameter * 2 * np.pi),
             0,
             self.radius * np.sin(lambda_parameter * 2 * np.pi)]

        return P

    def radius_of_curvature(self, lambda_parameter):
        return self.radius

    def df_dlambda(self, lambda_parameter):

        dx_dlambda = [-2 * np.pi * self.radius * np.sin(lambda_parameter * 2 * np.pi),
                      0,
                      2 * np.pi * self.radius * np.cos(lambda_parameter * 2 * np.pi)]

        return dx_dlambda

    def d_dx_dlambda_dt(self, state):

        d_dx_dlambda_dt = [-4 * np.pi * np.pi * self.radius * state[1] * np.cos(state[0] * 2 * np.pi),
                           0,
                           -4 * np.pi * np.pi * self.radius * state[1] * np.sin(state[0] * 2 * np.pi)]

        return d_dx_dlambda_dt

    def _length(self):
        return 2 * np.pi * self.radius
