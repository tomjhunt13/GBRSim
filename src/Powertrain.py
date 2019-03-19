from math import pi

class BrushedMotor:
    def __init__(self, motor_properties, battery_properties, transmission_properties, number_of_motors=1):
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
        # self.motor_efficiency = motor_properties['motor_efficiency']
        self.motor_speed_constant = 1 / (motor_properties['motor_speed_constant'] * 2 * pi / 60)
        # self.no_load_speed = motor_properties['no_load_speed'] * 2 * pi / 60
        self.number_of_motors = number_of_motors

        # Battery properties
        self.max_discharge_power = battery_properties['max_discharge_power']
        self.discharge_voltage = battery_properties['discharge_voltage']
        self.battery_efficiency = battery_properties['battery_efficiency']

        # Transmission
        self.transmission_ratio = transmission_properties['transmission_ratio']
        self.transmission_efficiency = transmission_properties['transmission_efficiency']

        # State
        self.t_n = 0
        self.i_n = 75

        # Electrical Properties
        self.R = 480 * 0.001
        self.L = 14 * 0.001 * 0.001


    def power(self, wheel_speed, demand, t):
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
        motor_speed = wheel_speed * self.transmission_ratio

        # Voltage
        V = demand * self.discharge_voltage

        # Update i
        dt = t - self.t_n

        # i_np1 = (V - self.motor_speed_constant * motor_speed) / self.R

        di_dt = self.update_equation(self.i_n, motor_speed, V)
        i_np1 = self.i_n + dt * di_dt

        # Torque
        T_m = self.torque_constant * i_np1
        T_w = T_m * self.transmission_ratio * self.transmission_efficiency

        # Power
        power = (V * i_np1) / self.battery_efficiency

        # Update state
        self.i_n = i_np1
        self.t_n = self.t_n + dt

        return self.number_of_motors * T_w, self.number_of_motors * power



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

