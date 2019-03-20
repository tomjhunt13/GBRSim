import numpy as np
from src.Track.Segment import *

class CubicBezier(Segment):
    def __init__(self, knots, control_points):

        self.knots = knots
        self.control_points = control_points
        
        super(CubicBezier, self).__init__()

    def direction(self, t):
        """
        For a given point on the track, get the direction of travel a unit vector
        :param lambda_param: float between 0 and 1 - value of parameter lambda to get direction of
        :return: 3 element list - Direction vector of track
        """

        # Get first derivative
        d = self._first_derivative(t)

        return np.multiply((1 / np.linalg.norm(d)), d)

    def position(self, t):
        """
        For a value of parameter t, get the position
        :param t: float between 0 and 1 - value of t to get position of
        :return: 3 element list - Position vector
        """

        # Unpack knot vectors
        P0 = self.knots[0]
        P1 = self.control_points[0]
        P2 = self.control_points[1]
        P3 = self.knots[1]


        # Calculate t parameters
        t_squared = t * t
        t_cubed = t_squared * t
        one_minus_t = (1 - t)
        one_minus_t_squared = one_minus_t * one_minus_t
        one_minus_t_cubed = one_minus_t_squared * one_minus_t


        # Get contributions
        P0_contribution = np.multiply(one_minus_t_cubed, P0)
        P1_contribution = np.multiply(3 * one_minus_t_squared * t, P1)
        P2_contribution = np.multiply(3 * one_minus_t * t_squared, P2)
        P3_contribution = np.multiply(t_cubed, P3)

        return list(np.add(np.add(P0_contribution, P1_contribution), np.add(P2_contribution, P3_contribution)))

    def radius_of_curvature(self, t):
        """

        :param t:
        :return:

        https://www.khanacademy.org/math/multivariable-calculus/multivariable-derivatives/curvature/v/curvature-formula-part-5
        """

        curvature = self._curvature(t)
        if np.isclose(curvature, 0, atol=1e-8):
            return 1e8

        return 1 / curvature

    def _curvature(self, t):
        """

        :param t:
        :return:

        https://www.khanacademy.org/math/multivariable-calculus/multivariable-derivatives/curvature/v/curvature-formula-part-5
        """

        # Derivatives
        dS_dt = self._first_derivative(t)
        d2S_dt2 = self._second_derivative(t)
        dS_dt_mag = np.linalg.norm(dS_dt)


        dS_dt_cross_dS2_dt2 = np.cross(dS_dt, d2S_dt2)
        dS_dt_cross_dS2_dt2_normalised = np.divide(dS_dt_cross_dS2_dt2, dS_dt_mag * dS_dt_mag * dS_dt_mag)

        return np.linalg.norm(dS_dt_cross_dS2_dt2_normalised)

    def _first_derivative(self, t):
        """
        For a value of parameter t, compute the first derivative wrt t
        :param t: float between 0 and 1 - value of t to get first derivative at
        :return: 3 element list - Derivative vector
        """

        # Unpack knot vectors
        P0 = self.knots[0]
        P1 = self.control_points[0]
        P2 = self.control_points[1]
        P3 = self.knots[1]

        # Calculate t parameters
        t_squared = t * t
        one_minus_t = (1 - t)
        one_minus_t_squared = one_minus_t * one_minus_t


        # Get contributions
        P0_contribution = np.multiply(-3 * one_minus_t_squared, P0)
        P1_contribution = np.multiply(3 * (1 - 4 * t + 3 * t_squared), P1)
        P2_contribution = np.multiply(3 * (2 * t - 3 * t_squared), P2)
        P3_contribution = np.multiply(3 * t_squared, P3)

        return list(np.add(np.add(P0_contribution, P1_contribution), np.add(P2_contribution, P3_contribution)))

    def _second_derivative(self, t):
        """
        For a value of parameter t, compute the second derivative wrt t
        :param t: float between 0 and 1 - value of t to get second derivative at
        :return: 3 element list - Derivative vector
        """

        # Unpack knot vectors
        P0 = self.knots[0]
        P1 = self.control_points[0]
        P2 = self.control_points[1]
        P3 = self.knots[1]

        # Get contributions
        P0_contribution = np.multiply(6 * (1 - t), P0)
        P1_contribution = np.multiply(3 * (- 4 + 6 * t), P1)
        P2_contribution = np.multiply(6 * (1 - 3 * t), P2)
        P3_contribution = np.multiply(6 * t, P3)

        return list(np.add(np.add(P0_contribution, P1_contribution), np.add(P2_contribution, P3_contribution)))






def fit_bezier(points):
    """

    :param points:
    :return:
    """

    # Check number of points
    if len(points) < 3:
        raise Exception('Not enough points')


    """
    
    At left boundary:
    
        dS(0)/dt_i =  dS(1)/dt_i-1
    
    """

    num_splines = len(points) - 1


    # Solve for P1
    P1 = [None, None, None]
    for i in range(3):

        # Initialise system
        M = [None] * num_splines
        d = [None] * num_splines

        # Boundary conditions
        d[0] = points[0][i] + 2 * points[1][i]
        d[-1] = 8 * points[-2][i] + points[-1][i]

        M[0] = [0] * num_splines
        M[0][0] = 2
        M[0][1] = 1

        M[-1] = [0] * num_splines
        M[-1][0] = 2
        M[-1][1] = 7


        # Construct rest of system
        j = 1
        while j < (num_splines - 1):
            M[j] = [0] * num_splines
            M[j][j - 1] = 1
            M[j][j] = 4
            M[j][j + 1] = 1

            d[j] = 4 * points[j][i] + 2 * points[j + 1][i]
            j += 1

        P1[i] = np.linalg.solve(M, d)

    # Unpack P1 and calculate P2
    P1_control_points = [[P1[i][p] for i in range(3)] for p in range(num_splines)]

    P2_control_points = [[None, None, None]] * num_splines
    # P1_control_points[i] =


    for i in range(3):

        for p in range(num_splines - 1):
            P2_control_points[p][i] = 2 * points[p + 1][i] - P1_control_points[p + 1][i]

        P2_control_points[num_splines - 1][i] = 0.5 * (P1_control_points[num_splines - 1][i] + points[-1][i])




    # P2_control_points = [[2 * points[p][i] - P1_control_points[p][i] for i in range(3)] for p in range(num_splines)]

    # P2_control_points[-1] = [0.5 * points[-2][i] + P1_control_points[-2][i] for i in range(3)]

    # Construct splines
    splines = [None] * num_splines

    for spline_index in range(num_splines):
        knots = [points[spline_index], points[spline_index + 1]]
        P1_j = P1_control_points[spline_index]
        P2_j = P2_control_points[spline_index]

        splines[spline_index] = CubicBezier(knots, [P1_j, P2_j])

    return splines

    # P2 = [[2 * points[j][i] - P1[i][j] for j in range(num_splines)] for i in range(3)]

    # # Construct splines
    # splines = [None] * num_splines
    #
    # for spline_index in range(num_splines):
    #     knots = [points[spline_index], points[spline_index + 1]]
    #     P1_j = [P1[spline_index][i] for i in range(3)]
    #     P2_j = [P2[spline_index][i] for i in range(3)]
    #
    #     splines[spline_index] = CubicBezier(knots, [P1_j, P2_j])
    #
    # return splines