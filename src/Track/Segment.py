import numpy as np

class Segment:

    def draw_coordinates(self, num_segments=20):
        """
        Calculate 3D coordinates to draw segment
        :param num_segments: Number of points used to discretise curve
        :return: tuple: x_coordinates, y_coordinates, z_coordinates
        """

        x = [None] * (num_segments + 1)
        y = [None] * (num_segments + 1)
        z = [None] * (num_segments + 1)

        for i in range(num_segments + 1):

            t = float(i) / num_segments
            P = self.position(t)

            x[i] = P[0]
            y[i] = P[1]
            z[i] = P[2]

        return x, y, z

    def gradient(self, t):
        """
        For a given point on the segment, get the gradient in radians
        :param lambda_param: float between 0 and 1 - value of parameter t to get gradient of
        :return: float - Gradient of track in radians
        """

        # Get directional vector
        direction = self.direction(t)

        # If flat
        if np.isclose(direction[2], 0):
            return 0

        # Get angle
        dot = np.dot(direction, [0, 0, 1])
        angle_to_vertical = np.arccos(dot)

        return (np.pi / 2) - angle_to_vertical

    def direction(self, lambda_param):
        """
        For a given point on the track, get the direction of travel a unit vector
        :param segment: integer - Index of segment of track to use
        :param lambda_param: float between 0 and 1 - value of parameter lambda to get direction of
        :return: 3 element list - Direction vector of track
        """

        pass
