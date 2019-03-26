import numpy as np

class Vehicle:
    def __init__(self, powertrain, transmission, vehicle_parameters={}):

        # Default attributes
        default_vehicle_attributes = {
            'Mass': 170,
            'Crr': 1.5 * 0.001,     # http://www.eshopsem.com/boutique/product.php?id_product=75
            'Cd': 0.3,
            'A': 1.3,
            'PoweredWheelRadius': 0.2,
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


        self.current_segment = starting_segment

        # Initialise state space
        self.f = self.equation_of_motion
        self.y = [initial_conditions]
        self.t = [0]
        self.time_step = time_step

        # Values
        self.fuel_power = [0]
        self.P = [0]
        self.Frr = [0]
        self.Fw = [0]
        self.Fa = [0]
        self.Fc = [0]
        self.V = [initial_conditions[0] * self.track.segments[starting_segment].length]
        self.lambda_param = [initial_conditions[0]]


        self.control_function = control_function

        self.segments_visited = [starting_segment]
        self.laps = 0

        while self.t[-1] <= time_limit and self.laps != lap_limit:
            self._step()



        self.fuel_power = make_average(self.fuel_power)
        self.P = make_average(self.P)
        self.Frr = make_average(self.Frr)
        self.Fw = make_average(self.Fw)
        self.Fc = make_average(self.Fc)
        self.Fa = make_average(self.Fa)
        self.V = make_average(self.V)


        return self.t[:-1], self.y[:-1], self.segment[:-1], self.fuel_power, self.P, self.Frr, self.Fw, self.Fa, self.Fc, self.lambda_param[:-1], self.V
        # return self.t, self.y, self.segment, self.lambda_param

    def _step(self):
        """

        :return:
        """

        h = self.time_step
        self.y_n = self.y[-1]
        t_n = self.t[-1]

        # Euler step
        print('pre: ' + str(self.y_n))
        k_1 = np.multiply(h, self.f(t_n, self.y_n))
        k_2 = np.multiply(h, self.f(t_n + (h / 2.0), np.add(self.y_n, np.multiply((0.5), k_1))))
        k_3 = np.multiply(h, self.f(t_n + (h / 2.0), np.add(self.y_n, np.multiply((0.5), k_2))))
        k_4 = np.multiply(h, self.f(t_n + h, np.add(self.y_n, k_3)))

        # Apply weighting
        k_1_w = np.multiply(k_1, 1 / 6)
        k_2_w = np.multiply(k_2, 1 / 3)
        k_3_w = np.multiply(k_3, 1 / 3)
        k_4_w = np.multiply(k_4, 1 / 6)

        y_np1 = np.add(self.y_n, np.add(k_1_w, np.add(k_2_w, np.add(k_3_w, k_4_w))))


        # f, fuel_power, P, Frr, Fw, Fa, Fd, lambda_param = self.f(t_n, self.y_n)
        # f, fuel_power, P, Frr, Fw, Fa, Fd, lambda_param = self.f(t_n, self.y_n)
        # k_1 = np.multiply(h, f)
        # print('post: ' + str(self.y_n))
        # y_np1 = np.add(self.y_n, k_1)
        t_np1 = t_n + h

        segment_index = int(np.floor(y_np1[0]))
        lambda_param = y_np1[0] - segment_index


        self.y.append(y_np1)
        self.t.append(t_np1)
        self.segment.append(self.current_segment)
        # self.fuel_power.append(fuel_power)
        # self.P.append(P)
        # self.Frr.append(Frr)
        # self.Fw.append(Fw)
        # self.Fa.append(Fa)
        # self.Fd.append(Fd)
        self.lambda_param.append(lambda_param)

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

        print(y)

        # Unpack y
        segment_index = int(np.floor(y[0]))

        print(segment_index)

        lambda_param = y[0] - segment_index
        # segment = self.track.segments[segment_index]


        # Increment
        if segment_index != self.current_segment:

            # Get velocity for previous segment
            old_seg_length = self.track.segments[self.current_segment].length
            old_seg_velocity = y[1] * old_seg_length

            # Continuity
            if segment_index > len(self.track.segments) - 1:
                segment_index = 0

            elif segment_index < 0:
                segment_index = len(self.track.segments) - 1

            self.current_segment = segment_index
            segment = self.track.segments[segment_index]
            y[1] = old_seg_velocity / segment.length
            self.y_n[1] = y[1]

            asdfs = 3
            print('Now')



        #
        #
        # # Current track segment
        segment = self.track.segments[segment_index]

        # Unpack y
        theta = segment.gradient(lambda_param)  # Road angle (rad)
        segment_length = segment.length  # Length of current track segment (m)
        print(segment_length)

        V = y[1] * segment_length  # Vehicle speed

        print(V)



        # Resistive forces
        Fw, Fa, Fc, Frr = self.resistive_forces(theta, V, segment_index, lambda_param)
        print(Fw, Fa, Fc, Frr)

        # Propulsive force
        throttle_demand = self.control_function(V, theta)

        P, fuel_power = self.power(V, throttle_demand, t)
        print(P)

        # P = 400

        # Equation of motion
        f = [
            y[1],

            (1 / (segment_length * self.m)) * (P - Frr - Fw - Fa - Fc)
        ]

        self.fuel_power.append(fuel_power)
        self.P.append(P)
        self.Frr.append(Frr)
        self.Fw.append(Fw)
        self.Fa.append(Fa)
        self.Fc.append(Fc)
        self.V.append(V)


        print(f)

        return f
        # return f, fuel_power, P, Frr, Fw, Fa, Fc, lambda_param

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

    def _cornering_drag(self, V, segment_index, lambda_param, alpha_deg=3):
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

def make_average(original):

    length = int((len(original) - 1) / 4)

    empty = []
    for i in range(length):
        print(i)

        k1 = original[4 * i]
        k2 = original[4 * i + 1]
        k3 = original[4 * i + 2]
        k4 = original[4 * i + 3]

        weighted_average = (1 / 6) * (k1 + k4 + 2 * (k2 + k3))
        empty.append(weighted_average)

    return empty

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