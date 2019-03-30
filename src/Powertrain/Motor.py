import numpy as np
from src.RK4 import *


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
        self.Power = motor_properties['Power']

        # Battery properties
        self.V_max = battery_properties['V_max']
        self.battery_efficiency = battery_properties['battery_efficiency']

        # State
        self.t_n = 0
        self.i_n = 50
        self.di_dt_n = 0

    def _step(self, t, omega, demand):
        """

        :param wheel_speed: Rotational speed of wheel (radians / second)
        :param demand: Throttle demand between 0 and 1
        :return:
        """

        back_emf = self._back_emf(omega)
        L_di_dt = self.L * self.di_dt_n

        V_max = max(np.roots([1, -1 * (L_di_dt + back_emf), -1 * self.Power * self.R]))
        print('V max: ' + str(V_max))

        if V_max > self.V_max:
            V_max = self.V_max

        # Update i
        dt = (t - self.t_n)
        t_motor = np.linspace(0, dt, 2)
        V = demand * V_max

        info = {''}

        y_np1, info = RK4_step(self.f, t_n, self._step_y_n, h, info_total)

        sol = odeint(self.state_equation, [self.i_n], t_motor, args=(V, omega))

        i_np1 = sol[-1][0]

        print('i: ' + str(i_np1))

        # Torque
        T_m = self.torque_constant * i_np1

        # Power
        power = (V * i_np1) / self.battery_efficiency

        print('Efficiency: ' +  str((T_m * omega) / (V * i_np1)))

        # Update state
        self.i_n = i_np1
        self.di_dt_n = self.state_equation([i_np1], 0, V, omega)
        self.t_n = self.t_n + dt

        return T_m, power


    def _state_equation(self, x, t, V, omega):
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