import numpy as np
from src.Track.Segment import *

class CubicBezier(Segment):
    def __init__(self, knots, control_points, **segment_options):

        self.knots = knots
        self.control_points = control_points
        
        super(CubicBezier, self).__init__(**segment_options)

    def position(self, lambda_parameter):
        """
        For a value of parameter lambda, get the position
        :param lambda_parameter: float between 0 and 1 - value of lambda to get position of
        :return: 3 element list - Position vector
        """

        # Unpack knot vectors
        P0 = self.knots[0]
        P1 = self.control_points[0]
        P2 = self.control_points[1]
        P3 = self.knots[1]


        # Calculate t parameters
        t_squared = lambda_parameter * lambda_parameter
        t_cubed = t_squared * lambda_parameter
        one_minus_t = (1 - lambda_parameter)
        one_minus_t_squared = one_minus_t * one_minus_t
        one_minus_t_cubed = one_minus_t_squared * one_minus_t


        # Get contributions
        P0_contribution = np.multiply(one_minus_t_cubed, P0)
        P1_contribution = np.multiply(3 * one_minus_t_squared * lambda_parameter, P1)
        P2_contribution = np.multiply(3 * one_minus_t * t_squared, P2)
        P3_contribution = np.multiply(t_cubed, P3)

        return list(np.add(np.add(P0_contribution, P1_contribution), np.add(P2_contribution, P3_contribution)))

    def horizontal_radius_of_curvature(self, lambda_parameter):

        curvature = self._curvature(lambda_parameter, horizontal=True)
        if np.isclose(curvature, 0, atol=1e-8):
            return 1e8

        return 1 / curvature

    def radius_of_curvature(self, lambda_parameter):
        """
        Compute the radius of curvature at lambda
        :param lambda_parameter: Scaled distance along curve
        :return: Radius of curvature
        """

        curvature = self._curvature(lambda_parameter)
        if np.isclose(curvature, 0, atol=1e-8):
            return 1e8

        return 1 / curvature

    def _curvature(self, lambda_parameter, horizontal=False):
        """
        Compute the magnitude of curvature
        :param lambda_parameter: Scaled distance along curve
        :return: Curvature

        https://www.khanacademy.org/math/multivariable-calculus/multivariable-derivatives/curvature/v/curvature-formula-part-5
        """

        # Derivatives
        dS_dt = self.df_dlambda(lambda_parameter)
        d2S_dt2 = self.d2x_dt2(lambda_parameter)
        dS_dt_mag = np.linalg.norm(dS_dt)

        # Curvature calculation
        dS_dt_cross_dS2_dt2 = np.cross(dS_dt, d2S_dt2)
        dS_dt_cross_dS2_dt2_normalised = np.divide(dS_dt_cross_dS2_dt2, dS_dt_mag * dS_dt_mag * dS_dt_mag)

        curvature = np.linalg.norm(dS_dt_cross_dS2_dt2_normalised)

        if horizontal:
            # Get horizontal curvature
            dT_dt_unit = np.divide(d2S_dt2, np.linalg.norm(d2S_dt2))
            dT_dt_horiz = [dT_dt_unit[0], dT_dt_unit[1], 0]
            dT_dt_horiz_mag = np.linalg.norm(dT_dt_horiz)

            curvature *= dT_dt_horiz_mag

        return curvature


    def df_dlambda(self, lambda_parameter):
        """
        For a value of parameter lambda, compute the first derivative wrt lambda
        :param lambda_parameter: float between 0 and 1 - value of lambda to get first derivative at
        :return: 3 element list - Derivative vector
        """

        # Unpack knot vectors
        P0 = self.knots[0]
        P1 = self.control_points[0]
        P2 = self.control_points[1]
        P3 = self.knots[1]

        # Calculate t parameters
        t_squared = lambda_parameter * lambda_parameter
        one_minus_t = (1 - lambda_parameter)
        one_minus_t_squared = one_minus_t * one_minus_t

        # Get contributions
        P0_contribution = np.multiply(-3 * one_minus_t_squared, P0)
        P1_contribution = np.multiply(3 * (1 - 4 * lambda_parameter + 3 * t_squared), P1)
        P2_contribution = np.multiply(3 * (2 * lambda_parameter - 3 * t_squared), P2)
        P3_contribution = np.multiply(3 * t_squared, P3)

        return list(np.add(np.add(P0_contribution, P1_contribution), np.add(P2_contribution, P3_contribution)))

    def d2x_dt2(self, lambda_parameter):
        """
        For a value of parameter lambda, compute the second derivative wrt lambda
        :param lambda_parameter: float between 0 and 1 - value of lambda to get second derivative at
        :return: 3 element list - Derivative vector
        """

        # Unpack knot vectors
        P0 = self.knots[0]
        P1 = self.control_points[0]
        P2 = self.control_points[1]
        P3 = self.knots[1]

        # Get contributions
        P0_contribution = np.multiply(6 * (1 - lambda_parameter), P0)
        P1_contribution = np.multiply(3 * (- 4 + 6 * lambda_parameter), P1)
        P2_contribution = np.multiply(6 * (1 - 3 * lambda_parameter), P2)
        P3_contribution = np.multiply(6 * lambda_parameter, P3)

        return list(np.add(np.add(P0_contribution, P1_contribution), np.add(P2_contribution, P3_contribution)))


def fit_cubic_bezier(points, **segment_options):
    """
    Fit n - 1 cubic bezier curves through n coordinates with C0, C1 and C2 continuity. (No explicit continuity at ends)
    :param points: List of coordinates to fit curve through
    :return: List of CubicBezier objects
    """

    # Check number of points
    if len(points) < 3:
        raise Exception('Not enough points')

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
        M[-1][-2] = 2
        M[-1][-1] = 7

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

    # Unpack P1
    P1_control_points = [[P1[i][p] for i in range(3)] for p in range(num_splines)]

    # Calculate P2
    P2 = [None] * 3
    for i in range(3):
        P2_i = [None] * num_splines
        for p in range(num_splines - 1):
            P2_i[p] = 2 * points[p + 1][i] - P1_control_points[p + 1][i]

        P2_i[-1] = 0.5 * (P1_control_points[-1][i] + points[-1][i])

        P2[i] = P2_i

    # Unpack P2
    P2_control_points = [[P2[i][p] for i in range(3)] for p in range(num_splines)]


    # Construct splines
    splines = [None] * num_splines

    for spline_index in range(num_splines):
        knots = [points[spline_index], points[spline_index + 1]]
        P1_j = P1_control_points[spline_index]
        P2_j = P2_control_points[spline_index]

        splines[spline_index] = CubicBezier(knots, [P1_j, P2_j], **segment_options)

    return splines