import numpy as np
from src.Integration import RKF45

class SEMVehicle:

    def __init__(self, vehicle_parameters={}):

        # Default attributes
        default_vehicle_attributes = {
            # Vehicle properties
            'Mass': 170,
            'Crr': 1.5 * 0.001,     # http://www.eshopsem.com/boutique/product.php?id_product=75
            'Cd': 0.2,
            'A': 1.26,
            'PoweredWheelRadius': 0.279,
            'LongitudinalCoG': 0.5,     # Assume even weight distribution

            # Motor properties
            'motor_torque_constant': 0.001 * 123,
            'motor_speed_constant': 1 / (77.8 * (2 * np.pi / 60)),
            'R': 0.365,
            'L': 0.161 * 0.001,

            # Battery properties
            'Power': 250,
            'V_max': 48,
            'Battery_Efficiency': 0.9,

            # Transmission properties
            'transmission_ratio': 10,
            'transmision_efficiency': 0.8,
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

        # Motor properties
        self.motor_torque_constant = vehicle_parameters['motor_torque_constant']
        self.motor_speed_constant = vehicle_parameters['motor_speed_constant']
        self.R = vehicle_parameters['R']
        self.L = vehicle_parameters['L']
        self.Power = vehicle_parameters['Power']

        # Battery properties
        self.V_max = vehicle_parameters['V_max']
        self.battery_efficiency = vehicle_parameters['Battery_Efficiency']

        # Transmission properties
        self.transmission_ratio = vehicle_parameters['transmission_ratio']
        self.transmision_efficiency = vehicle_parameters['transmision_efficiency']

    def simulate(self, track, starting_segment, initial_conditions, control_function, time_step=0.025, lap_limit=1, time_limit=100, verbose=True):


        # Initialise vehicle on track
        self.track = track
        self.segment = [starting_segment]
        self.current_segment = starting_segment

        # Initialise state space
        self.f = self.equation_of_motion
        self.y = [initial_conditions]
        self.t = [0]
        self.time_step = time_step

        # Values
        self.info_dict = [{
            'V': initial_conditions[0] * self.track.segments[starting_segment].length,
            'segment': starting_segment,
            'lambda_param': initial_conditions[0]
        }]

        self.lambda_param = [initial_conditions[0]]


        self.control_function = control_function
        self.verbose = verbose

        self.segments_visited = [starting_segment]
        self.laps = 0

        t_n = self.t[-1]
        y_n = self.y[-1]

        self.solver = RKF45.RKF45(self.equation_of_motion)

        while t_n <= time_limit and self.number_of_laps() != lap_limit:
            if self.verbose:
                print('Time: ' + str(t_n))

            t_n = self.t[-1]
            y_n = self.y[-1]

            t_np1 = t_n + time_step

            y_np1, info_dict = self.update(t_np1, y_n)


            self.t.append(t_np1)
            self.y.append(y_np1)
            self.info_dict.append(info_dict)

        # Update info dict
        update_dictionary_keys(self.info_dict[1], self.info_dict[0])

        return self.info_dict

    def number_of_laps(self):
        """
        Number of laps car has done
        :return: Number of laps
        """
        return self.laps

    def update(self, t_np1, y_n):
        """
        Update vehicle system to time t_np1
        :return:
        """

        t_n = self.t[-1]
        h = t_np1 - t_n
        self._step_y_n = y_n

        info = {
            'P': 0,
            'Frr': 0,
            'Fw': 0,
            'Fa': 0,
            'Fc': 0,
            'V': 0
        }

        # y_np1 = RKF45.RKF45_step(self.f, t_n, self._step_y_n, h, info)
        y_np1 = self.solver.update(t_n, self._step_y_n, h, info)

        segment_index = int(np.floor(y_np1[0]))
        lambda_param = y_np1[0] - segment_index

        info['segment'] = segment_index
        info['lambda_param'] = lambda_param
        info['t'] = t_np1
        info['y'] = y_np1

        self._update_lap_counter(segment_index)

        return y_np1, info

    def _update_lap_counter(self, new_segment):

        # If first initial step
        if len(self.y) < 2:
            return

        # If segment incremented
        if (self.y[-1][0] < self.y[-2][0] and self.y[-1][1] > 0) or (np.floor(self.y[-1][0]) > np.floor(self.y[-2][0])):

            print(new_segment)

            self.segments_visited.append(new_segment)

            print(len(self.segments_visited), len(self.track.segments) + 1)

            if len(self.track.segments) == 1:
                self.laps += 1

            elif len(self.segments_visited) >= len(self.track.segments) + 1:
                self.laps += 1

        # Else if decremented
        if (self.y[-1][0] > self.y[-2][0] and self.y[-1][1] < 0) or (np.floor(self.y[-1][0]) < np.floor(self.y[-2][0])):
            self.segments_visited = [new_segment]

    def equation_of_motion(self, t, y, information_dictionary, **kwargs):
        """

        :param t:
        :param y:
        :return:
        """

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
        V = y[1] * segment_length  # Vehicle speed

        # Propulsive force
        throttle_demand = self.control_function(V, theta)
        motor_torque = self.motor_torque_constant * y[2]
        propulsive_force = (1 / self.PoweredWheelRadius) * self.transmision_efficiency * \
                            self.transmission_ratio * motor_torque

        # Resistive forces
        Fw, Fa, Fc, Frr = self.resistive_forces(theta, V, segment_index, lambda_param)
        resistive_force = Fw + Fa + Fc + Frr

        # Motor
        omega_motor = (self.transmission_ratio / self.PoweredWheelRadius) * V

        back_emf = self.motor_speed_constant * omega_motor
        V_max = max(np.roots([1, -1 * (back_emf), -1 * self.Power * self.R]))

        if V_max > self.V_max:
            V_max = self.V_max

        motor_voltage = V_max * throttle_demand
        electrical_power = (V * y[2]) / self.battery_efficiency

        if np.isclose(electrical_power, 0):
            motor_efficiency = 0

        else:
            motor_efficiency = (motor_torque * omega_motor) / (motor_voltage * y[2])

        information_dictionary['Fuel Power'] = max(0, electrical_power)



        # Build state vector
        f = [
            y[1],
            (1 / self.m) * (propulsive_force - resistive_force),
            (1 / self.L) * (motor_voltage - y[2] * self.R - self.motor_speed_constant * omega_motor)
        ]

        # information_dictionary['fuel_power'] = fuel_power
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

        return self._weight(theta), self._aerodynamic_drag(V), self._cornering_drag(V, segment_index, lambda_param), self._rolling_resistance(V, theta)

    def _weight(self, theta):
        """

        :param theta:
        :return:
        """

        return self.m * self.track.g * np.sin(theta)

    def _aerodynamic_drag(self, V):
        """

        :param segment_index:
        :param lambda_param:
        :param V:
        :return:
        """

        rho = self.track.rho

        # Aerodynamic drag
        Fa = 0.5 * rho * self.Cd * self.A * V * V

        return direction_modifier(V) * Fa

    def _rolling_resistance(self, V, theta):
        """

        :param V:
        :param theta:
        :return:
        """

        # Rolling resistance
        # Frr = self.m * self.track.g * np.cos(theta) * self.Crr * np.sign(V)
        Frr = self.m * self.track.g * self.Crr * np.sign(V)

        return direction_modifier(V) * Frr

    def _cornering_drag(self, V, segment_index, lambda_param, alpha_deg=1):
        """

        :return:
        """

        # Assume max alpha = 3 deg
        alpha = alpha_deg * (np.pi / 180)

        # Track radius of curvature
        segment = self.track.segments[segment_index]
        R = segment.horizontal_radius_of_curvature(lambda_param)

        # Centripetal force
        Fy = (self.m * V * V) / (R)

        return direction_modifier(V) * Fy * np.tan(alpha)


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


def update_dictionary_keys(full_dictionary, destination_dictionary):
    """
    Updates destination dictionary with keys its missing from full dictionary
    :param full_dictionary:
    :param destination_dictionary:
    :return:
    """

    for key in full_dictionary.keys():
        if key not in destination_dictionary.keys():

            destination_dictionary[key] = 0