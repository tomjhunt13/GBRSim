import math

from src.RK4 import *
from src.Powertrain import *

class Vehicle:
    def __init__(self, vehicle_parameters, powertrain):

        # Vehicle attributes
        self.m = 150         # Total vehicle mass (kg)
        self.Crr = 0.005    # Coefficient of rolling resistance
        self.Cd = 0.27       # Coefficient of drag
        self.A = 1.3          # Frontal area (m^2)
        self.Cs = 0.3       # Cornering stiffness
        self.PoweredWheelRadius = 0.2       # Radius of tyre for powered wheel

        self.powertrain = powertrain

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

        g = 9.81
        rho = 1.225

        # Propulsive force
        throttle_demand = 0

        P, fuel_power = self.power(y[1], throttle_demand)

        theta = -0.02         # Road angle (rad)
        cornering = False   # Bool - is current track segment a corner?

        segment_length = 100        # Length of current track segment (m)
        V = y[1] * segment_length   # Vehicle speed

        # Cornering drag
        if cornering:
            R = 10000
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

        return f

if __name__ == '__main__':

    powertrain = Powertrain()

    v = Vehicle(1, powertrain)
    s = RK4()

    t, y = s.solve(v.equation_of_motion, [0, 0], time_step=0.05, time_range=[0, 100])


    print(3)
    x_0 = [x[0] * 100 for x in y]
    # x_1 = [x[1] for x in v.state_history]
    #
    import matplotlib.pyplot as plt

    plt.plot(t, x_0)
    # plt.plot(v.t, x_1)
    #
    plt.show()
    a = 0
