import numpy as np

class Track:
    def __init__(self, segment_list):
        """
        Track object
        :param segment_list: List with elements of Segment type
        """

        # Segments
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

    def segment_lambda_from_arc_length(self, arc_length):
        """
        Find segment and lambda from the arc length
        :param arc_length:
        :return:
        """

        remaining_arc_length = arc_length
        for segment_index, segment in enumerate(self.segments):

            remaining_arc_length -= segment.length
            if remaining_arc_length <= 0:
                break

        segment_arc_length = remaining_arc_length + segment.length
        lambda_parameter = segment.lambda_from_arc_length(segment_arc_length)

        return segment, lambda_parameter