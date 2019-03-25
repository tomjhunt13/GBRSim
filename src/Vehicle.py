import math
import numpy as np

class Vehicle:
    def __init__(self, vehicle_parameters, powertrain):

        # Vehicle attributes
        self.m = vehicle_parameters['Mass']     # Total vehicle mass (kg)
        self.Crr = vehicle_parameters['Crr']    # Coefficient of rolling resistance
        self.Cd = vehicle_parameters['Cd']      # Coefficient of drag
        self.A = vehicle_parameters['A']        # Frontal area (m^2)
        self.Cs = vehicle_parameters['Cs']      # Cornering stiffness
        self.PoweredWheelRadius = vehicle_parameters['PoweredWheelRadius']       # Radius of tyre for powered wheel

        self.powertrain = powertrain

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

            elif len(self.track.track) == 1:
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
        omega = (1 / self.PoweredWheelRadius) * velocity


        torque, fuel_power = self.powertrain.power(omega, demand, t)
        linear_force = torque * self.PoweredWheelRadius


        return linear_force, fuel_power

    def equation_of_motion(self, t, y):
        """

        :param t:
        :param y:
        :return:
        """

        # Current track segment
        segment = self.track.segments[self.segment[-1]]

        g = 9.81
        rho = 1.225

        theta = segment.gradient(y[0])      # Road angle (rad)
        segment_length = segment.length     # Length of current track segment (m)
        V = y[1] * segment_length           # Vehicle speed

        # Propulsive force
        throttle_demand = self.control_function(V, theta)

        P, fuel_power = self.power(V, throttle_demand, t)
        P = 400

        # Cornering drag
        R = segment.horizontal_radius_of_curvature(y[0])
        Fz = g * (self.m / 2)  # Assume even weight distribution
        alpha = (self.m * y[1] * y[1]) / (R * Fz * self.Cs)       # Slip angle (rad)
        Fy = Fz * self.Cs * alpha
        Fd = Fy * math.sin(alpha)

        # Weight
        Fw = self.m * g * math.sin(theta)


        if y[1] != 0:
            # Rolling resistance
            Frr = self.m * g * math.cos(theta) * self.Crr * np.sign(V)

            # Aerodynamic drag
            Fa = 0.5 * rho * self.Cd * self.A * V * V * np.sign(V)

        else:
            Frr = 0
            Fa = 0



        # Equation of motion
        f = [
            y[1],

            (1 / (segment_length * self.m)) * (P - Frr - Fw - Fa - Fd)
        ]



        return f, fuel_power, P, Frr, Fw, Fa, Fd


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