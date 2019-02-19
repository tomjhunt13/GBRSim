import math

from src.RK4 import *
from src.Powertrain import *
from src.Track import *

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

    def simulate(self, track, starting_segment, initial_conditions, time_step=0.01, time_limit=[0, 100]):


        # Initialise vehicle on track
        self.track = track
        self.segment = [starting_segment]

        # Initialise state space
        self.f = self.equation_of_motion
        self.y = [np.array(initial_conditions)]
        self.t = [time_range[0]]
        self.fuel_power = [0]
        self.time_step = time_step

        while self.t[-1] <= time_range[1]:
            self._step()

        return self.t, self.y, self.segment, self.fuel_power

    def _step(self):
        """

        :return:
        """

        h = self.time_step
        y_n = self.y[-1]
        t_n = self.t[-1]

        # Euler step
        f, fuel_power = self.f(t_n, y_n)
        k_1 = np.multiply(h, f)
        y_np1 = np.add(y_n, k_1)
        t_np1 = t_n + h


        # Current track segment
        segment_index = self.segment[-1]
        segment = self.track.track[segment_index]
        segment_length = segment['length']

        # Check if vehicle currently within bounds of current segment
        if y_np1[0] > 1:
            segment_index = increment(segment_index, len(self.track.track) - 1)
            segment = self.track.track[segment_index]

            # Update velocity to new track segment
            current_velocity = y_np1[1] * segment_length
            segment_length = segment['length']
            new_param_velocity = current_velocity / segment_length

            y_np1 = np.array([
                0,
                new_param_velocity
            ])

        elif y_np1[0] < 0:
            segment_index = increment(segment_index, len(self.track.track) - 1, increment=-1)
            segment = self.track.track[segment_index]

            # Update velocity to new track segment
            current_velocity = y_np1[1] * segment_length
            segment_length = segment['length']
            new_param_velocity = current_velocity / segment_length

            y_np1 = np.array([
                0,
                new_param_velocity
            ])

        self.y.append(y_np1)
        self.t.append(t_np1)
        self.segment.append(segment_index)
        self.fuel_power.append(fuel_power)


    def power(self, velocity, demand):
        """

        :param velocity: Linear vehicle velocity
        :param demand:
        :return:
        """

        # Convert linear velocity to wheel rotational speed
        omega = (1 / self.PoweredWheelRadius) * velocity


        torque, fuel_power = self.powertrain.power(omega, demand)
        linear_force = torque * self.PoweredWheelRadius


        return linear_force, fuel_power

    def equation_of_motion(self, t, y):
        """

        :param t:
        :param y:
        :return:
        """

        # Current track segment
        segment = self.track.track[self.segment[-1]]

        g = 9.81
        rho = 1.225

        theta = self.track.gradient(self.segment[-1], y[0])     # Road angle (rad)
        segment_length = segment['length']        # Length of current track segment (m)
        V = y[1] * segment_length   # Vehicle speed

        # Propulsive force
        if V > 10 or theta < 0:
            throttle_demand = 0

        else:
            throttle_demand = 1

        P, fuel_power = self.power(V, throttle_demand)

        # Cornering drag
        if segment['type'] == 'arc':
            R = segment['radius']
            Fz = g * (self.m / 2)  # Assume even weight distribution
            alpha = (self.m * y[1] * y[1]) / (R * Fz * self.Cs)       # Slip angle (rad)
            Fy = Fz * self.Cs * alpha
            Fd = Fy * math.sin(alpha)
        else:
            Fd = 0

        # Weight
        Fw = self.m * g * math.sin(theta)


        if y[1] != 0:
            # Rolling resistance
            Frr = self.m * g * self.Crr * np.sign(V)

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



        return f, fuel_power


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


if __name__ == '__main__':

    # Track
    r = 200
    theta = np.pi / 32

    track = [
        {'type': 'arc', 'start': [0, 0, 0],
         'end': [r * np.sin(theta), 0, r - r * np.cos(theta)],
         'centre': [0, 0, r]},

        {'type': 'arc', 'start': [r * np.sin(theta), 0, r - r * np.cos(theta)],
         'end': [2 * r * np.sin(theta), 0, 2 * (r - r * np.cos(theta))],
         'centre': [2 * r * np.sin(theta), 0, 2 * (r - r * np.cos(theta)) - r]},

        {'type': 'line', 'start': [2 * r * np.sin(theta), 0, 2 * (r - r * np.cos(theta))],
         'end': [100, 0, 2 * (r - r * np.cos(theta))]},

        {'type': 'arc', 'start': [100, 0, 2 * (r - r * np.cos(theta))],
         'end': [150, 50, 2 * (r - r * np.cos(theta))],
         'centre': [100, 50, 2 * (r - r * np.cos(theta))]},

        {'type': 'arc', 'start': [150, 50, 2 * (r - r * np.cos(theta))],
         'end': [100, 100, 2 * (r - r * np.cos(theta))],
         'centre': [100, 50, 2 * (r - r * np.cos(theta))]},

        {'type': 'line', 'start': [100, 100, 2 * (r - r * np.cos(theta))],
         'end': [0, 100, 0]},

        {'type': 'arc', 'start': [0, 100, 0], 'end': [-50, 50, 0], 'centre': [0, 50, 0]},
        {'type': 'arc', 'start': [-50, 50, 0], 'end': [0, 0, 0], 'centre': [0, 50, 0]}

    ]

    track_1 = [
        {'type': 'line', 'start': [0, 0, 0], 'end': [100, 0, 0]},
        {'type': 'arc', 'start': [100, 0, 0], 'end': [150, 50, 0], 'centre': [100, 50, 0]},
        {'type': 'arc', 'start': [150, 50, 0], 'end': [100, 100, 0], 'centre': [100, 50, 0]},
        {'type': 'line', 'start': [100, 100, 0], 'end': [0, 100, 0]},
        {'type': 'arc', 'start': [0, 100, 0], 'end': [-50, 50, 0], 'centre': [0, 50, 0]},
        {'type': 'arc', 'start': [-50, 50, 0], 'end': [0, 0, 0], 'centre': [0, 50, 0]}
    ]


    track = Track(track_1)

    powertrain = Powertrain()

    v = Vehicle(1, powertrain, track)
    s = RK4()

    t, y = s.solve(v.equation_of_motion, [0.5, 0], time_step=0.05, time_range=[0, 100])


    print(3)
    x_0 = [x[0] for x in y]
    # x_1 = [x[1] for x in v.state_history]
    #
    import matplotlib.pyplot as plt

    plt.plot(t, x_0)
    # plt.plot(v.t, x_1)
    #
    plt.show()
    a = 0

    # Axes3D.scatter(xs, ys, zs=0, zdir='z', s=20, c=None, depthshade=True, *args, **kwargs)¶
