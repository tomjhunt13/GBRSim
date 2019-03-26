import numpy as np

class Vehicle:
    def __init__(self, powertrain, transmission, vehicle_parameters={}):

        # Default attributes
        default_vehicle_attributes = {
            'Mass': 170,
            'Crr': 1.5 * 0.001,     # http://www.eshopsem.com/boutique/product.php?id_product=75
            'Cd': 0.2,
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
        self.longitudinal_CoG = vehicle_parameters['LongitudinalCoG']             # Assume even weight distribution

        self.powertrain = powertrain
        self.transmission = transmission

    def simulate(self, track, starting_segment, initial_conditions, control_function, time_step=0.01, lap_limit=1, time_limit=100):


        # Initialise vehicle on track
        self.track = track
        self.segment = [starting_segment]

        # Initialise state space
        self.f = self.equation_of_motion
        self.y = [np.array(initial_conditions)]
        self.t = [0]
        self.time_step = time_step

        # Values
        self.fuel_power = [0]
        self.P = [0]
        self.Frr = [0]
        self.Fw = [0]
        self.Fa = [0]
        self.Fd = [0]


        self.control_function = control_function

        self.segments_visited = [starting_segment]
        self.laps = 0

        while self.t[-1] <= time_limit and self.laps != lap_limit:
            self._step()

        return self.t, self.y, self.segment, self.fuel_power, self.P, self.Frr, self.Fw, self.Fa, self.Fd

    def _step(self):
        """

        :return:
        """

        h = self.time_step
        y_n = self.y[-1]
        t_n = self.t[-1]

        # Euler step
        f, fuel_power, P, Frr, Fw, Fa, Fd = self.f(t_n, y_n)
        k_1 = np.multiply(h, f)
        y_np1 = np.add(y_n, k_1)
        t_np1 = t_n + h


        # Current track segment
        segment_index = self.segment[-1]
        segment = self.track.segments[segment_index]
        segment_length = segment.length

        # Check if vehicle currently within bounds of current segment
        if y_np1[0] > 1:
            segment_index = increment(segment_index, len(self.track.segments) - 1)
            segment = self.track.segments[segment_index]

            if segment_index != self.segments_visited[-1]:
                self.segments_visited.append(segment_index)
                if len(self.segments_visited) == len(self.track.segments) + 1:
                    self.laps += 1

            elif len(self.track.segments) == 1:
                self.laps += 1

            # Update velocity to new track segment
            current_velocity = y_np1[1] * segment_length
            segment_length = segment.length
            new_param_velocity = current_velocity / segment_length

            y_np1 = np.array([
                0,
                new_param_velocity
            ])

        elif y_np1[0] < 0:
            segment_index = increment(segment_index, len(self.track.segments) - 1, increment=-1)
            segment = self.track.segments[segment_index]

            self.segments_visited = [segment_index]

            # Update velocity to new track segment
            current_velocity = y_np1[1] * segment_length
            segment_length = segment.length
            new_param_velocity = current_velocity / segment_length

            y_np1 = np.array([
                1,
                new_param_velocity
            ])

        self.y.append(y_np1)
        self.t.append(t_np1)
        self.segment.append(segment_index)
        self.fuel_power.append(fuel_power)
        self.P.append(P)
        self.Frr.append(Frr)
        self.Fw.append(Fw)
        self.Fa.append(Fa)
        self.Fd.append(Fd)

    def power(self, velocity, demand, t):
        """

        :param velocity: Linear vehicle velocity
        :param demand:
        :return:
        """

        # Convert linear velocity to wheel rotational speed
        omega_wheel = (1 / self.PoweredWheelRadius) * velocity

        # Convert wheel rotational speed to motor rotational speed
        omega_motor = omega_wheel * self.transmission.ratio

        # Torque and power at motor
        torque_motor, fuel_power = self.powertrain.power(omega_motor, demand, t)

        # Torque and power at wheel
        torque_wheel = self.transmission.ratio * self.transmission.efficiency * torque_motor

        # Linear force
        linear_force = torque_wheel * self.PoweredWheelRadius


        return linear_force, fuel_power

    def equation_of_motion(self, t, y):
        """

        :param t:
        :param y:
        :return:
        """

        # Current track segment
        segment_index = self.segment[-1]
        segment = self.track.segments[segment_index]

        # Unpack y
        theta = segment.gradient(y[0])  # Road angle (rad)
        segment_length = segment.length  # Length of current track segment (m)
        V = y[1] * segment_length  # Vehicle speed

        # Resistive forces
        Fw, Fa, Fc, Frr = self.resistive_forces(theta, V, segment_index, y[0])

        # Propulsive force
        throttle_demand = self.control_function(V, theta)

        # print(throttle_demand)

        P, fuel_power = self.power(V, throttle_demand, t)
        # P = 400

        # Equation of motion
        f = [
            y[1],

            (1 / (segment_length * self.m)) * (P - Frr - Fw - Fa - Fc)
        ]


        return f, fuel_power, P, Frr, Fw, Fa, Fc

    def resistive_forces(self, theta, V, segment_index, lambda_param):
        """

        :param theta:
        :param V:
        :param segment_index:
        :param lambda_param:
        :return:
        """

        return self._weight(theta), self._aerodynamic_drag(V), self._cornering_drag(V, segment_index, lambda_param), self._rolling_resitance(V, theta)


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

    def _rolling_resitance(self, V, theta):
        """

        :param V:
        :param theta:
        :return:
        """

        # Rolling resistance
        Frr = self.m * self.track.g * np.cos(theta) * self.Crr * np.sign(V)

        return direction_modifier(V) * Frr

    def _cornering_drag(self, V, segment_index, lambda_param, alpha_deg=1):
        """

        :return:
        """

        # Track radius of curvature
        segment = self.track.segments[segment_index]
        R = segment.horizontal_radius_of_curvature(lambda_param)

        # Centripetal force
        Fy = (self.m * V * V) / R

        # Assume max alpha = 3 deg
        alpha = alpha_deg * (np.pi / 180)

        return direction_modifier(V) * Fy * np.sin(alpha)


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



def increment(current_index, max_index, increment=1):
    """

    :param current_index:
    :param max_index:
    :return:
    """

    next_index = current_index + increment

    if next_index > max_index:
        return 0

    if next_index < 0:
        return max_index

    return next_index