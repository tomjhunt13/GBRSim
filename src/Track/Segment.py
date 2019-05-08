import numpy as np

from scipy.integrate import solve_ivp

class Segment:

    def __init__(self, **segment_options):

        # Default segment options
        default_segment_options = {
            'arc_length_integral_step': 1e-4,
            'arc_length_map_resolution': 20
        }

        for option in default_segment_options.keys():
            if option not in segment_options:
                segment_options[option] = default_segment_options[option]

        # Pre-calculate values
        self.length = self._length()
        self.arc_length_integral_step = segment_options['arc_length_integral_step']
        self.calculate_arc_length_map(number_of_sections=segment_options['arc_length_map_resolution'])

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

    def gradient(self, lambda_parameter):
        """
        For a given point on the segment, get the gradient in radians
        :param t: float between 0 and 1 - value of parameter t to get gradient of
        :return: float - Gradient of track in radians
        """

        # Get directional vector
        direction = self.direction(lambda_parameter)

        # If flat
        if np.isclose(direction[2], 0):
            return 0

        # Get angle
        dot = np.dot(direction, [0, 0, 1])
        angle_to_vertical = np.arccos(dot)

        return (np.pi / 2) - angle_to_vertical

    def direction(self, lambda_parameter):
        """
        For a given point on the segment, get the direction of travel as a unit vector
        :param lambda_parameter: float between 0 and 1 - value of lambda to get direction of
        :return: 3 element list - Direction vector of track
        """

        df_dlambda = self.df_dlambda(lambda_parameter)
        df_dlambda_mag = np.linalg.norm(df_dlambda)

        return np.divide(df_dlambda, df_dlambda_mag)

    def position(self, lambda_parameter):
        pass

    def df_dlambda(self, lambda_parameter):
        pass

    def df_dlambda_integrand(self, lambda_parameter, y):

        return np.linalg.norm(self.df_dlambda(lambda_parameter))

    def horizontal_radius_of_curvature(self, lambda_parameter):
        return self.radius_of_curvature(lambda_parameter)

    def radius_of_curvature(self, lambda_parameter):
        return 0

    def arc_length_integral(self, alpha_1, alpha_2):
        """
        Finds the arc length along segment between coordinates: alpha_1 and alpha_2
        :param alpha_1: Start coordinate
        :param alpha_2: End coordinate
        :return:  Arc length
        """

        integration_interval = (alpha_1, alpha_2)

        integration_result = solve_ivp(self.df_dlambda_integrand, integration_interval, [0])

        arc_length = integration_result['y'][0][-1]

        return arc_length

    def calculate_arc_length_map(self, number_of_sections=20):
        """
        Populates map relating arc length to lambda
        :param number_of_sections: Resolution of map
        """

        # Calculate node positions
        number_of_nodes = number_of_sections + 1
        nodes = np.linspace(0, 1, number_of_nodes)

        # Calculate arc lengths
        arc_length_map = [None] * number_of_nodes

        total_arc_length = 0
        for index in range(number_of_nodes - 1):

            arc_length_map[index] = total_arc_length
            total_arc_length += self.arc_length_integral(nodes[index], nodes[index + 1])

        arc_length_map[-1] = total_arc_length

        self.arc_length_map = [nodes, arc_length_map]

        return self.arc_length_map

    def lambda_from_arc_length(self, arc_length):

        return np.interp(arc_length, self.arc_length_map[1], self.arc_length_map[0])


    def _length(self):
        return self.arc_length_integral(0, 1)


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