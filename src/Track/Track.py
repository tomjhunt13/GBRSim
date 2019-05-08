import numpy as np

class Track:
    def __init__(self, segment_list):
        """
        Track object
        :param segment_list: List with elements of Segment type
        """

        # Environment attributes
        self.rho = 1.225    # Air density (g / cm^3)
        self.g = 9.81

        # Segments
        # self.current_segment = 0
        self.segments = segment_list


    def draw_coordinates(self, num_segments=5):
        """
        Calculate 3D coordinates to draw track
        :param num_arc_points: Number of points used to discretise curve
        :return: tuple: x_coordinates, y_coordinates, z_coordinates
        """

        x = []
        y = []
        z = []

        for index, segment in enumerate(self.segments):

            x_i, y_i, z_i = segment.draw_coordinates(num_segments=num_segments)

            x += x_i
            y += y_i
            z += z_i

        return x, y, z

    def gradient(self, segment_index, t):
        """
        For a given point on the track, get the gradient in radians
        :param segment_index: integer - Index of segment of track to use
        :param t: float between 0 and 1 - scaled  distance along segment
        :return: float - Gradient of track in radians
        """

        return self.segments[segment_index].gradient(t)

    def direction(self, segment_index, t):
        """
        For a given point on the track, get the direction of travel a unit vector
        :param segment_index: integer - Index of segment of track to use
        :param t: float between 0 and 1 - scaled  distance along segment
        :return: 3 element list - Direction vector of track
        """

        return self.segments[segment_index].direction(t)

    def position(self, segment_index, t):
        """
        For a given point on the track, get the position of the vehicle in 3D space
        :param segment_index: integer - Index of segment of track to use
        :param t: float between 0 and 1 - scaled  distance along segment
        :return: 3 element list - Position vector of vehicle
        """

        return self.segments[segment_index].position(t)

    def total_length(self):

        length = 0
        for segment in self.segments:
            length += segment.length

        return length

    def dx_dlambda(self, segment_index, t):

        dx_dt = self.segments[segment_index].df_dlambda(t)

        return np.linalg.norm(dx_dt)

    def d_dx_dlambda_dt(self, segment_index, state):

        d_dx_dlambda_dt = self.segments[segment_index].d_dx_dlambda_dt(state)

        return np.linalg.norm(d_dx_dlambda_dt)
