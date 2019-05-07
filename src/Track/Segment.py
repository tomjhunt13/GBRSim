import numpy as np


class Segment:

    def __init__(self):
        self.length = 0

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
        :param t: float between 0 and 1 - value of parameter t to get gradient of
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

    def direction(self, t):
        pass

    def position(self, t):
        pass

    def dx_dt(self, t):
        pass

    def d_dx_dlambda_dt(self, state):
        pass

    def horizontal_radius_of_curvature(self, t):
        return self.radius_of_curvature(t)

    def radius_of_curvature(self, t):
        return 0

    def _length(self):
        return length_of_parametric_curve(self.position)


def length_of_parametric_curve(position_function, tolerance=1e-5):
    """
    Approximates arc length of a parametric curve numerically
    :param position_function: function pointer to parametric position function
    :param tolerance: integral convergence tolerance
    :return: length
    """
    # Iteratively double the number of points until integral converges

    # Initially
    difference = tolerance + 1
    level = 0
    t = [0, 1]
    old_points = [position_function(t[0]), position_function(t[1])]
    old_length = distance_between_coordinates(old_points)

    # Start iteration
    offset = 1
    while difference > tolerance:
        level += 1

        # Get new points
        offset = offset / 2
        num_new_points = 2**(level-1)
        new_points_t = [offset * (1 + 2 * i) for i in range(num_new_points)]
        new_points = [position_function(t) for t in new_points_t]

        # Mix new points into points list
        total_points = len(old_points) + num_new_points
        all_points = [None] * total_points

        switch = False
        old_counter = 0
        new_counter = 0
        for i in range(total_points):

            if switch:
                all_points[i] = new_points[new_counter]
                new_counter += 1

            else:
                all_points[i] = old_points[old_counter]
                old_counter += 1

            switch = not switch

        # Calculate convergence
        new_length = distance_between_coordinates(all_points)
        difference = new_length - old_length

        # Update old
        old_points = all_points
        old_length = new_length

    return new_length


def distance_between_coordinates(coordinates):
    """
    Sums the linear distance between a series of coordinates
    :param coordinates:  list of coordinates
    :return: length
    """

    length = 0
    for i in range(len(coordinates) - 1):

        vec = np.subtract(coordinates[i+1], coordinates[i])
        length += np.linalg.norm(vec)

    return length