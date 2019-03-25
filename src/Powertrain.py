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
        :param motor_properties:

                Dictionary of form - {'torque_constant': __ (Nm/A), 'motor_efficiency': __,
                                      'motor_speed_constant': __ (rpm / V), 'motor_coil_resistance': __ (Ohms)}

        :param battery_properties:

                Dictionary of form - {'max_discharge_power': __ (W), 'discharge_voltage': __ (V), 'battery_efficiency': __}

        :param transmission_properties:

                Dictionary of form - {'transmission_ratio': __, 'transmission_efficiency': __}

        :param number_of_motors: Number of motors used
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
        self.i_n = 0

    def power(self, omega, demand, t):
        """

        :param wheel_speed: Rotational speed of wheel (radians / second)
        :param demand: Throttle demand between 0 and 1
        :return:
        """

        """
        Equations:
        P = I V = I^2 R
        T = (k_t / R) * (V - E) = (k_t / R) * (V − k_e * ω)
        
        P = electrical power supplied
        I = current supplied
        V = voltage supplied
        R = electrical resistance of motor
        E = induced back voltage due to the rotor spinning
        T = torque exerted on motor shaft
        k_t = motor torque constant
        k_e = motor speed constant
        omega = motor shaft angular speed
        
        
        Sources: 
        - http://www.inf.fu-berlin.de/lehre/WS04/Robotik/motors.pdf
        - https://www.maxonmotor.com/medias/sys_master/root/8815460712478/DC-EC-Key-Information-14-EN-42-50.pdf
        """

        # Motor speed

        #
        # # Power
        # P = self.max_discharge_power * demand
        #
        # # Voltage
        # V = self.discharge_voltage
        #
        # # Current
        # i = P / V
        #
        #
        # Tm = self.torque_constant * i * (1 - (motor_speed / (V * self.motor_speed_constant)))
        #
        # Tw = Tm * self.transmission_ratio * self.transmission_efficiency
        #
        # return self.number_of_motors * Tw, self.number_of_motors * P * (1 / self.battery_efficiency)


        # Update i
        dt = t - self.t_n
        V = demand * self.V_max


        di_dt = self.update_equation(self.i_n, omega, V)
        i_np1 = self.i_n + dt * di_dt

        # Torque
        T_m = self.torque_constant * i_np1

        # Power
        power = (V * i_np1) / self.battery_efficiency

        # Update state
        self.i_n = i_np1
        self.t_n = self.t_n + dt

        return T_m, power



    def update_equation(self, i, omega, V):
        """
        Returns di / dt
        :param i:
        :param omega:
        :param V:
        :return:
        """

        a =  (V - i * self.R - self.motor_speed_constant * omega)

        return a

        return (1 / self.L) * (V - i * self.R - self.motor_speed_constant * omega)


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



