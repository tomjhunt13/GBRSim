import numpy as np

from src.Model import Model
from src.Integration import RK4


class BrushedDCMotor(Model.Model):
    def __init__(self, motor_parameters={}, dt=1e-3, solver=RK4.RK4, verbose=False):
        """
        Approximate brushed motor model without resistance
        """

        # Default attributes
        default_motor_attributes = {
        # Motor properties
        'torque_constant': 0.001 * 123,
        'speed_constant': 1 / (77.8 * (2 * np.pi / 60)),
        'R': 0.365,
        'L': 0.161 * 0.001,

        # Battery properties
        'Power': 250,
        'V_max': 48,
        'Battery_Efficiency': 0.9,
        }

        for attribute in default_motor_attributes.keys():
            if attribute not in motor_parameters:
                motor_parameters[attribute] = default_motor_attributes[attribute]

        # Motor properties
        self.torque_constant = motor_parameters['torque_constant']
        self.speed_constant = motor_parameters['speed_constant']
        self.R = motor_parameters['R']
        self.L = motor_parameters['L']
        self.Power = motor_parameters['Power']

        # Battery properties
        self.V_max = motor_parameters['V_max']
        self.battery_efficiency = [motor_parameters['Battery_Efficiency']]

        # State
        self.reset()

        # Admin properties
        self.verbose = verbose

        # Numerical properties
        self.solver = solver
        self.dt = dt

        super(BrushedDCMotor, self).__init__()

    def reset(self):
        self.t_n = 0
        self.i_n = 0
        self.di_dt_n = 0

    def update(self, t_np1, omega, demand, information_dict):
        """

        :param wheel_speed: Rotational speed of wheel (radians / second)
        :param demand: Throttle demand between 0 and 1
        :return:
        """

        # Get voltage from controller
        V = self.motor_controller(omega, demand)

        # Current state
        t_n = self.t_n
        i_n = self.i_n

        # Get next state
        y_np1 = self.simulate([i_n], V=V, omega=omega, dt=self.dt, t_start=t_n, t_end=t_np1, verbose=self.verbose, solver=self.solver)
        i_np1 = y_np1[-1]['y'][0]

        # Torque
        torque = self.torque_constant * i_np1

        # Power
        electrical_power = (V * i_np1) / self.battery_efficiency[0]

        if np.isclose(electrical_power, 0):
            motor_efficiency = 0

        else:
            motor_efficiency = (torque * omega) / (V * i_np1)

        if self.verbose:
            print('Motor')
            print('i: ' + str(i_np1))
            print('Motor Torque: ' + str(torque))
            print('Efficiency: ' + str(motor_efficiency))

        # Update stored state
        self.i_n = i_np1
        self.t_n = t_np1

        # Update dictionaries
        information_dict['Fuel Power'] = max(0, electrical_power)
        information_dict['Motor Current'] = i_np1
        information_dict['Motor Efficiency'] = motor_efficiency
        information_dict['Motor Torque'] = torque
        information_dict['Motor Applied Voltage'] = V

        return torque, electrical_power

    def initialise(self, initial_conditions, information_dictionary, **kwargs):
        self.V = kwargs['V']
        self.omega = kwargs['omega']

    def update_equation(self, t, state, info_dict, **kwargs):
        """
        Motor update equation
        :param  t: Time at previous step
        :param state: Single element list of previous state. Current, i, = state[0]
        :param info_dict: Information dictionary to fill
        :return: di / dt
        """

        di_dt = (1 / self.L) * (self.V - state[0] * self.R - self.back_emf(self.omega))
        info_dict['di_dt'] = di_dt

        return di_dt

    def motor_controller(self, omega_n, demand):
        """
        Calculates voltage applied to motor
        :param omega_n: Motor speed at time t_n
        :param demand: Throttle position between 0 and 1
        :return: Motor applied voltage
        """

        back_emf = self.back_emf(omega_n)
        V_max = max(np.roots([1, -1 * back_emf, -1 * self.Power * self.R]))

        if V_max > self.V_max:
            V_max = self.V_max

        V = V_max * demand

        if self.verbose:
            print('Controller Voltage Out:  ' + str(V))

        return V

    def back_emf(self, omega):
        """
        Calculate the induced back emf from spinning motor
        :param omega: Motor speed
        :return: Back emf value
        """
        return self.speed_constant * omega


# ------- Pre-made motor instances ------- #

def MaxonRE65(dt=1e-3, solver=RK4.RK4, verbose=False, power=250, V_max=48, battery_efficiency=0.9):
    """
    https://www.maxonmotor.com/maxon/view/category/motor?etcc_cu=onsite&etcc_med_onsite=Product&etcc_cmp_onsite=RE+Program&etcc_plc=Overview-Page-DC-Motors&etcc_var=%5bcom%5d%23en%23_d_&target=filter&filterCategory=re
    """

    # Mechanical Properties
    Kt = 0.001 * 123
    Kw = 1 / (77.8 * (2 * np.pi / 60))

    # Electrical Properties
    R = 0.365
    L = 0.161 * 0.001

    # Motor properties
    motor_parameters = {'torque_constant': Kt, 'speed_constant': Kw, 'R': R, 'L': L, 'Power': power,
                        'V_max': V_max, 'battery_efficiency': battery_efficiency}

    return BrushedDCMotor(motor_parameters, dt=dt, solver=solver, verbose=verbose)

def Moog_C42_L90_30(dt=1e-3, solver=RK4.RK4, verbose=False, power=359, V_max=90, battery_efficiency=0.9):
    """
    https://www.moog.com/literature/MCG/moc23series.pdf
    """

    # Mechanical Properties
    Kt = 0.5791
    Kw = 0.5730

    # Electrical Properties
    R = 1.45
    L = 5.4 * 0.001

    # Motor properties
    motor_parameters = {'torque_constant': Kt, 'speed_constant': Kw, 'R': R, 'L': L, 'Power': power,
                        'V_max': V_max, 'battery_efficiency': battery_efficiency}

    return BrushedDCMotor(motor_parameters, dt=dt, solver=solver, verbose=verbose)

def Moog_C42_L90_10(dt=1e-3, solver=RK4.RK4, verbose=False, power=317, V_max=48, battery_efficiency=0.9):
    """
    https://www.moog.com/literature/MCG/moc23series.pdf
    """

    # Mechanical Properties
    Kt = 0.3531
    Kw = 0.3533

    # Electrical Properties
    R = 0.6
    L = 2 * 0.001

    # Motor properties
    motor_parameters = {'torque_constant': Kt, 'speed_constant': Kw, 'R': R, 'L': L, 'Power': power,
                        'V_max': V_max, 'battery_efficiency': battery_efficiency}

    return BrushedDCMotor(motor_parameters, dt=dt, solver=solver, verbose=verbose)