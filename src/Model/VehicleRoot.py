import numpy as np

from src.Model import ConstrainedParticle

class VehicleRoot(ConstrainedParticle.ConstrainedParticle):

    def __init__(self, vehicle_parameters={}, verbose=False):

        # Default attributes
        default_vehicle_attributes = {
            # Model properties
            'Mass': 170,
            'Crr': 1.5 * 0.001,     # http://www.eshopsem.com/boutique/product.php?id_product=75
            'Cd': 0.3,
            'A': 1.26,
            'PoweredWheelRadius': 0.279,
            'LongitudinalCoG': 0.5,     # Assume even weight distribution
        }

        for attribute in default_vehicle_attributes.keys():
            if attribute not in vehicle_parameters:
                vehicle_parameters[attribute] = default_vehicle_attributes[attribute]

        # Unpack vehicle attributes
        self.m = vehicle_parameters['Mass']     # Total vehicle mass (kg)
        self.Crr = vehicle_parameters['Crr']    # Coefficient of rolling resistance
        self.Cd = vehicle_parameters['Cd']      # Coefficient of drag
        self.A = vehicle_parameters['A']        # Frontal area (m^2)
        self.PoweredWheelRadius = vehicle_parameters['PoweredWheelRadius']       # Radius of tyre for powered wheel
        self.longitudinal_CoG = vehicle_parameters['LongitudinalCoG']

        # Admin properties
        self.verbose = verbose

        super(VehicleRoot, self).__init__()

    def post_initialisation(self, **kwargs):

        # Initialise controller
        self.controller = kwargs['controller']
        self.controller.initialise(track=self.track)
        self.control_function = self.controller.demand

        # Pre-calculate constants
        self.aero_force = 0.5 * self.Cd * self.A * self.track.rho
        self.mg = self.m * self.track.g
        self.rolling_resistance_force = self.mg * self.Crr

    def equation_of_motion(self, t, y, information_dictionary, **kwargs):

        if self.verbose:
            print('State: ' + str(y))

        # Get world variables from position
        segment_index = int(np.floor(y[0]))
        lambda_param = y[0] - segment_index
        segment = self.track.segments[segment_index]
        theta = segment.gradient(lambda_param)
        # segment_scale = self.track.df_dlambda(segment_index, lambda_param)
        velocity = y[1]

        # Get propulsive force
        throttle_demand = self.control_function(velocity=velocity, theta=theta, segment=theta, lambda_param=y[0])
        propulsive_force = self._propulsive_force(t, y, velocity, throttle_demand, information_dictionary)

        # Resistive forces
        Fw, Fa, Fc, Frr = self.resistive_forces(theta, velocity, segment_index, lambda_param)
        resistive_force = Fw + Fa + Fc + Frr

        # Net force
        net_force = (propulsive_force - resistive_force)
        acceleration = net_force / self.m

        # Calculate d^2lambda_dt^2 correction factor
        # correction_factor = y[1] * self.track.d_dx_dlambda_dt(segment_index, [lambda_param, y[1]])

        # Build state vector
        dy_dt = [
            y[1],
            (acceleration)
        ]

        information_dictionary['Gradient'] = 100 * (theta / (np.pi / 4))
        information_dictionary['P'] = propulsive_force
        information_dictionary['Frr'] = Frr
        information_dictionary['Fw'] = Fw
        information_dictionary['Fa'] = Fa
        information_dictionary['Fc'] = Fc
        information_dictionary['Velocity (m/s)'] = velocity
        information_dictionary['Throttle Demand'] = throttle_demand
        # information_dictionary['Segment Scale'] = segment_scale
        information_dictionary['Net Force'] = net_force
        information_dictionary['Acceleration'] = acceleration
        # information_dictionary['Correction Factor'] = correction_factor
        information_dictionary['lambda'] = y[0]
        information_dictionary['dlambda / dt'] = y[1]
        information_dictionary['d^2lambda / dt^2'] = dy_dt[1]

        self._further_calculations(y, dy_dt, velocity, throttle_demand, information_dictionary)

        return dy_dt

    def _further_calculations(self, y, dy_dt, velocity, throttle_demand, information_dictionary):
        pass

    def _propulsive_force(self, t_np1, y_n, velocity, demand, information_dictionary):

        return 0

    def resistive_forces(self, theta, V, segment_index, lambda_param):
        """
        :param theta:
        :param V:
        :param segment_index:
        :param lambda_param:
        :return:
        """

        direction_mod = direction_modifier(V)

        Fw = self._weight(theta)
        Fa = direction_mod * self._aerodynamic_drag(V)
        Fc = direction_mod * self._cornering_drag(V, segment_index, lambda_param)
        Frr = direction_mod * self._rolling_resistance()

        Fw = 0
        Fc = 0

        return Fw, Fa, Fc, Frr

    def _weight(self, theta):
        """
        :param theta:
        :return:
        """

        return self.mg * np.sin(theta)

    def _aerodynamic_drag(self, V):
        """
        :param segment_index:
        :param lambda_param:
        :param V:
        :return:
        """

        return self.aero_force * V * V

    def _rolling_resistance(self):
        """
        :param V:
        :param theta:
        :return:
        """

        return self.rolling_resistance_force

    def _cornering_drag(self, V, segment_index, lambda_param, alpha_deg=1):
        """
        :return:
        """

        # Assume max alpha = 3 deg
        alpha = alpha_deg * (np.pi / 180)

        # Track radius of curvature
        segment = self.track.segments[segment_index]
        R = segment.horizontal_radius_of_curvature(lambda_param)    # Longest resistive force calculation

        # Centripetal force
        Fy = (self.m * V * V) / (R)

        return Fy * np.tan(alpha)

def direction_modifier(V):
    """
    :param V:
    :return:
    """
    if V > 0:
        return 1

    elif V < 0:
        return -1

    else:
        return 0
