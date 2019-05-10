import numpy as np

from src.Model import ConstrainedParticle

class VehicleRoot(ConstrainedParticle.ConstrainedParticle):

    def __init__(self, vehicle_parameters={}, verbose=False):

        # Default attributes
        default_vehicle_attributes = {
            # Model properties
            'VehicleMass': 100,
            'DriverMass': 70,
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
        self.vehicle_mass = [vehicle_parameters['VehicleMass']]
        self.driver_mass = [vehicle_parameters['DriverMass']]
        self.m = self.vehicle_mass[0] + self.driver_mass[0]
        self.Crr = [vehicle_parameters['Crr']]
        self.Cd = [vehicle_parameters['Cd']]
        self.A = [vehicle_parameters['A']]
        self.PoweredWheelRadius = [vehicle_parameters['PoweredWheelRadius']]
        self.longitudinal_CoG = [vehicle_parameters['LongitudinalCoG']]
        self.g = 9.81
        self.rho = 1.225

        # Admin properties
        self.verbose = verbose

        super(VehicleRoot, self).__init__()

    def post_initialisation(self, **kwargs):

        # Initialise controller
        self.controller = kwargs['controller']
        self.controller.initialise(track=self.track)
        self.control_function = self.controller.demand

        # Pre-calculate constants
        self.m = self.vehicle_mass[0] + self.driver_mass[0]
        self.aero_force = 0.5 * self.Cd[0] * self.A[0] * self.rho
        self.mg = self.m * self.g
        self.rolling_resistance_force = self.mg * self.Crr[0]

    def update_equation(self, t, state, information_dictionary, **kwargs):

        if self.verbose:
            print('State: ' + str(state))

        # Get position on track
        segment, segment_index, lambda_parameter = self.track.segment_lambda_from_arc_length(state[0])
        theta = segment.gradient(lambda_parameter)
        velocity = state[1]

        # Get propulsive force
        throttle_demand = self.control_function(velocity=velocity, theta=theta, arc_length=state[0])
        propulsive_force = self._propulsive_force(t, state, throttle_demand, information_dictionary)

        # Resistive forces
        Fw, Fa, Fc, Frr = self.resistive_forces(theta, velocity, segment_index, lambda_parameter)
        resistive_force = Fw + Fa + Fc + Frr

        # Net force
        net_force = (propulsive_force - resistive_force)
        acceleration = net_force / self.m

        # Build state vector
        dy_dt = [
            state[1],
            acceleration
        ]

        information_dictionary['Gradient'] = 100 * (theta / (np.pi / 4))
        information_dictionary['P'] = propulsive_force
        information_dictionary['Frr'] = Frr
        information_dictionary['Fw'] = Fw
        information_dictionary['Fa'] = Fa
        information_dictionary['Fc'] = Fc
        information_dictionary['Velocity (m/s)'] = velocity
        information_dictionary['Throttle Demand'] = throttle_demand
        information_dictionary['Net Force (N)'] = net_force
        information_dictionary['Acceleration (m/s^2)'] = acceleration

        self._further_calculations(state, dy_dt, throttle_demand, information_dictionary)

        return dy_dt

    def _further_calculations(self, y, dy_dt, throttle_demand, information_dictionary):
        pass

    def _propulsive_force(self, t_np1, y_n, demand, information_dictionary):

        return 0

    def resistive_forces(self, theta, V, segment_index, lambda_param):
        """
        :param theta:
        :param V:
        :param segment_index:
        :param lambda_param:
        :return:
        """

        direction_mod = ConstrainedParticle.direction_modifier(V)

        Fw = -1 * self.weight_force(theta)
        Fa = direction_mod * self.aerodynamic_drag_force(V)
        Fc = direction_mod * self._cornering_drag(V, segment_index, lambda_param)
        Frr = direction_mod * self._rolling_resistance()

        return Fw, Fa, Fc, Frr

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

    def _velocity_to_omega_wheel(self, velocity):

        return velocity / self.PoweredWheelRadius[0]

    def _wheel_torque_to_linear(self, wheel_torque):

        return wheel_torque / self.PoweredWheelRadius[0]

