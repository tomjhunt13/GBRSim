from src.Track.Segment import *


class Line(Segment):
    def __init__(self, coordinates):
        self.coordinates = coordinates

        super(Line, self).__init__()

        self.length = self._length()

    def draw_coordinates(self, **kwargs):
        """
        Calculate 3D coordinates to draw segment
        :return: tuple: x_coordinates, y_coordinates, z_coordinates
        """

        coords = [[self.coordinates[0][i], self.coordinates[1][i]] for i in range(3)]
        return coords[0], coords[1], coords[2]

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

    def dx_dt(self, t):
        return self._AB()

    def d_dx_dlambda_dt(self, lambda_param):
        return 0

    def _length(self):
        return np.linalg.norm(self._AB())

    def _AB(self):
        return list(np.subtract(self.coordinates[1], self.coordinates[0]))
