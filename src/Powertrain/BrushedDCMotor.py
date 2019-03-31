# import numpy as np
from src.RK4 import *


class BrushedMotor:
    def __init__(self, motor_properties, battery_properties, verbose=False):
        """
        Approximate brushed motor model without resistance
        """

        # Motor properties
        self.torque_constant = motor_properties['torque_constant']
        self.motor_speed_constant = motor_properties['motor_speed_constant']
        self.R = motor_properties['R']
        self.L = motor_properties['L']
        self.Power = motor_properties['Power']

        # Battery properties
        self.V_max = battery_properties['V_max']
        self.battery_efficiency = battery_properties['battery_efficiency']

        # State
        self.t_n = 0
        self.i_n = 0
        self.di_dt_n = 0

        # Admin properties
        self.verbose = verbose

    def _update(self, t_np1, omega_n, demand):
        """

        :param wheel_speed: Rotational speed of wheel (radians / second)
        :param demand: Throttle demand between 0 and 1
        :return:
        """

        # Get voltage from controller
        V = self._controller(omega_n, demand)


        # Update i
        dt = (t_np1 - self.t_n)
        y_n = [self.i_n]

        info_dict = {'V': V, 'omega': omega_n}

        y_np1 = RK4_step(self._state_equation, self.t_n, y_n, dt, {}, V, omega_n)
        i_np1 = y_np1[0]

        print('i: ' + str(i_np1))

        # Torque
        T_m = self.torque_constant * i_np1

        # Power
        power = (self.V * i_np1) / self.battery_efficiency

        print('Efficiency: ' + str((T_m * omega_n) / (self.V * i_np1)))

        # Update state
        self.i_n = i_np1
        self.di_dt_n = self._state_equation(0, [i_np1], {})
        self.t_n = self.t_n + dt

        return T_m, power


    def _state_equation(self, t, y, info_dict, kjargs):
        """
        Returns di / dt
        :param i:
        :param omega:
        :param V:
        :return:
        """

        V = kjargs

        return (1 / self.L) * (V - y[0] * self.R - self.motor_speed_constant * omega)

    def _controller(self, omega_n, demand):
        """
        Calculates voltage
        :param omega_n:
        :param demand:
        :return:
        """

        back_emf = self._back_emf(omega_n)
        L_di_dt = self.L * self.di_dt_n

        V_max = max(np.roots([1, -1 * (L_di_dt + back_emf), -1 * self.Power * self.R]))

        if V_max > self.V_max:
            V_max = self.V_max

        V = V_max * demand

        if self.verbose:
            print(V)

        return V

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
    Power = 250

    # Power Supply
    V_max = 48
    Battery_Efficiency = 0.9

    # Motor properties
    motor_properties = {'torque_constant': Kt, 'motor_speed_constant': Kw, 'R': R, 'L': L, 'Power': Power}
    battery_properties = {'V_max': V_max, 'battery_efficiency': Battery_Efficiency}

    return BrushedMotor(motor_properties, battery_properties)

def Moog_C42_L90_30():
    """
    https://www.moog.com/literature/MCG/moc23series.pdf
    """

    # Mechanical Properties
    Kt = 0.5791
    Kw = 0.5730

    # Electrical Properties
    R = 1.45
    L = 5.4 * 0.001
    Power = 359

    # Power Supply
    V_max = 48
    Battery_Efficiency = 0.9

    # Motor properties
    motor_properties = {'torque_constant': Kt, 'motor_speed_constant': Kw, 'R': R, 'L': L, 'Power': Power}
    battery_properties = {'V_max': V_max, 'battery_efficiency': Battery_Efficiency}

    return BrushedMotor(motor_properties, battery_properties)

def Moog_C42_L90_10():
    """
    https://www.moog.com/literature/MCG/moc23series.pdf
    """

    # Mechanical Properties
    Kt = 0.3531
    Kw = 0.3533

    # Electrical Properties
    R = 0.6
    L = 2 * 0.001
    Power = 317

    # Power Supply
    V_max = 48
    Battery_Efficiency = 0.9


    # Motor properties
    motor_properties = {'torque_constant': Kt, 'motor_speed_constant': Kw, 'R': R, 'L': L, 'Power': Power}
    battery_properties = {'V_max': V_max, 'battery_efficiency': Battery_Efficiency}

    return BrushedMotor(motor_properties, battery_properties)