import numpy as np
import time

from src.Model import Model

class Vehicle(Model.Model):

    def __init__(self, powertrain, vehicle_parameters={}, verbose=False):

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

        # Powertrain
        self.powertrain = powertrain

        # Admin properties
        self.verbose = verbose

        super(Vehicle, self).__init__()

    def end_condition(self):

        if self.number_of_laps() != self.lap_limit:
            return True

        return False

    def pre_step(self, t_n, y_n):

        for i in range(2):
            y_n[i] = self._step_y_n[i]

    def post_step(self, t_np1, y_np1, information_dictionary):

        segment_index = int(np.floor(y_np1[0]))
        lambda_param = y_np1[0] - segment_index

        information_dictionary['segment'] = segment_index
        information_dictionary['lambda_param'] = lambda_param
        information_dictionary['t'] = t_np1
        information_dictionary['y'] = y_np1

        self._step_y_n = y_np1
        self.y.append(y_np1)
        self._update_lap_counter(segment_index)

    def initialise(self, initial_conditions, information_dictionary, **kwargs):

        # Initialise vehicle on track
        starting_segment = int(np.floor(initial_conditions[0]))
        self.current_segment = starting_segment
        self.laps = 0
        self.track = kwargs['track']
        self._step_y_n = initial_conditions
        self.y = [initial_conditions]
        self.highest_segment = starting_segment

        # Initialise controller
        self.controller = kwargs['controller']
        self.controller.initialise(track=self.track)
        self.control_function = self.controller.demand

        # Optional kwargs
        optional_kwargs = {'lap_limit': 1}
        for keyword in optional_kwargs:
            if keyword not in kwargs:
                kwargs[keyword] = optional_kwargs[keyword]

        self.lap_limit = kwargs['lap_limit']

        # Update information dictionary
        information_dictionary['V'] = initial_conditions[1] * self.track.segments[starting_segment].length
        information_dictionary['segment'] = starting_segment
        information_dictionary['lambda_param'] = initial_conditions[0]

        # Pre-calculate constants
        self.aero_force = 0.5 * self.Cd * self.A * self.track.rho
        self.mg = self.m * self.track.g
        self.rolling_resistance_force = self.mg * self.Crr

    def number_of_laps(self):
        """
        Number of laps car has done
        :return: Number of laps
        """
        return self.laps

    def _update_lap_counter(self, new_segment):

        # If first initial step
        if len(self.y) < 2:
            return

        # If segment incremented
        if (self.y[-1][0] < self.y[-2][0] and self.y[-1][1] > 0) or (np.floor(self.y[-1][0]) > np.floor(self.y[-2][0])):

            print(new_segment)

            if new_segment < self.highest_segment:

                self.laps += 1

            else:

                self.highest_segment = new_segment

    def equation_of_motion(self, t, y, information_dictionary, **kwargs):


        if self.verbose:
            print(y)

        # Unpack y
        segment_index = int(np.floor(y[0]))
        lambda_param = y[0] - segment_index

        # Increment
        if segment_index != self.current_segment:

            # Get velocity for previous segment
            old_seg_length = self.track.segments[self.current_segment].length
            old_seg_velocity = y[1] * old_seg_length

            # Continuity
            if segment_index > len(self.track.segments) - 1:
                segment_index = 0
                y[0] = 0

            elif segment_index < 0:
                segment_index = len(self.track.segments) - 1
                y[0] = segment_index + 0.99999

            self.current_segment = segment_index
            segment = self.track.segments[segment_index]
            y[1] = old_seg_velocity / segment.length
            self._step_y_n[1] = y[1]

        # Current track segment
        segment = self.track.segments[segment_index]

        # Unpack y
        theta = segment.gradient(lambda_param)  # Road angle (rad)
        segment_length = segment.length  # Length of current track segment (m)
        V = y[1] * segment_length  # Model speed

        # Propulsive force
        throttle_demand = self.control_function(V=V, theta=theta, segment=theta, lambda_param=lambda_param)
        propulsive_force = self._update_powertrain(t, information_dictionary, V, throttle_demand)

        # Resistive forces
        Fw, Fa, Fc, Frr = self.resistive_forces(theta, V, segment_index, lambda_param)
        resistive_force = Fw + Fa + Fc + Frr

        # Build state vector
        f = [
            y[1],
            (1 / self.m) * (propulsive_force - resistive_force)
        ]

        information_dictionary['Gradient'] = 100 * (theta / (np.pi / 4))
        information_dictionary['P'] = propulsive_force
        information_dictionary['Frr'] = Frr
        information_dictionary['Fw'] = Fw
        information_dictionary['Fa'] = Fa
        information_dictionary['Fc'] = Fc
        information_dictionary['V'] = V
        information_dictionary['Throttle Demand'] = throttle_demand

        return f

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
        Frr = direction_mod * self._rolling_resistance(V)

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

    def _rolling_resistance(self, V):
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

    def _update_powertrain(self, t_np1, information_dictionary, velocity, demand):
        """

        :param velocity: Linear vehicle velocity
        :param demand:
        :return:
        """

        # Convert linear velocity to wheel rotational speed
        omega_wheel = (1 / self.PoweredWheelRadius) * velocity

        # Get wheel torque
        wheel_torque = self.powertrain.update(t_np1, information_dictionary, omega_wheel, demand)

        # Linear force
        linear_force = wheel_torque / self.PoweredWheelRadius

        return linear_force


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
