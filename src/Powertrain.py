import numpy as np
from scipy.integrate import odeint
from math import pi

class Transmission:
    def __init__(self, ratio=1, efficiency=1):
        """

        :param ratio:
        :param efficiency:
        """
        self.ratio = ratio
        self.efficiency = efficiency


class BrushedMotor:
    def __init__(self, motor_properties, battery_properties):
        """
        Approximate brushed motor model without resistance
        """

        # Motor properties
        self.torque_constant = motor_properties['torque_constant']
        self.motor_speed_constant = motor_properties['motor_speed_constant']
        self.R = motor_properties['R']
        self.L = motor_properties['L']

        # Battery properties
        self.V_max = battery_properties['V_max']
        self.battery_efficiency = battery_properties['battery_efficiency']

        # State
        self.t_n = 0
        self.i_n = 50
        self.di_dt_n = 0

    def power(self, omega, demand, t):
        """

        :param wheel_speed: Rotational speed of wheel (radians / second)
        :param demand: Throttle demand between 0 and 1
        :return:
        """

        max_power = 300
        back_emf = self._back_emf(omega)
        L_di_dt = self.L * self.di_dt_n

        V_max = max(np.roots([1, -1 * (L_di_dt + back_emf), -1 * max_power * self.R]))
        print(V_max)

        if V_max > self.V_max:
            V_max = self.V_max

        # Update i
        dt = (t - self.t_n)
        t_motor = np.linspace(0, dt, 31)
        V = demand * V_max

        sol = odeint(self.state_equation, [self.i_n], t_motor, args=(V, omega))

        i_np1 = sol[-1][0]

        # print(i_np1)

        # Torque
        T_m = self.torque_constant * i_np1

        # Power
        power = (V * i_np1) / self.battery_efficiency

        # print('Efficiency: ' +  str((T_m * omega) / (V * i_np1)))

        # Update state
        self.i_n = i_np1
        self.di_dt_n = self.state_equation([i_np1], 0, V, omega)
        self.t_n = self.t_n + dt

        return T_m, power


    def state_equation(self, x, t, V, omega):
        """
        Returns di / dt
        :param i:
        :param omega:
        :param V:
        :return:
        """

        return (1 / self.L) * (V - x[0] * self.R - self.motor_speed_constant * omega)

    def _back_emf(self, omega):
        return self.motor_speed_constant * omega


def MaxonRE65():
    """

    :return:

    https://www.maxonmotor.com/maxon/view/category/motor?etcc_cu=onsite&etcc_med_onsite=Product&etcc_cmp_onsite=RE+Program&etcc_plc=Overview-Page-DC-Motors&etcc_var=%5bcom%5d%23en%23_d_&target=filter&filterCategory=re
    """

    # Mechanical Properties
    Kt = 0.001 * 123
    Kw = 1 / (77.8 * (2 * pi / 60))

    # Electrical Properties
    R = 0.365
    L = 0.161 * 0.001

    # Power Supply
    V_max = 48
    Battery_Efficiency = 0.9

    # Motor properties
    motor_properties = {'torque_constant': Kt, 'motor_speed_constant': Kw, 'R': R, 'L': L}
    battery_properties = {'V_max': V_max, 'battery_efficiency': Battery_Efficiency}

    return BrushedMotor(motor_properties, battery_properties)

def Moog_C42_L90_30():
    """
    """

    # Mechanical Properties
    Kt = 0.5791
    Kw = 0.5730

    # Electrical Properties
    R = 1.45
    L = 5.4 * 0.001

    # Power Supply
    V_max = 48
    Battery_Efficiency = 0.9

    # Motor properties
    motor_properties = {'torque_constant': Kt, 'motor_speed_constant': Kw, 'R': R, 'L': L}
    battery_properties = {'V_max': V_max, 'battery_efficiency': Battery_Efficiency}

    return BrushedMotor(motor_properties, battery_properties)

def Moog_C42_L90_10():
    """
    """

    # Mechanical Properties
    Kt = 0.3531
    Kw = 0.3533

    # Electrical Properties
    R = 0.6
    L = 2 * 0.001

    # Power Supply
    V_max = 48
    Battery_Efficiency = 0.9

    # Motor properties
    motor_properties = {'torque_constant': Kt, 'motor_speed_constant': Kw, 'R': R, 'L': L}
    battery_properties = {'V_max': V_max, 'battery_efficiency': Battery_Efficiency}

    return BrushedMotor(motor_properties, battery_properties)


