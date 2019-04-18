from src.RK4 import *

class Powertrain:
    def __init__(self, verbose=False):
        self.verbose = verbose

    def update(self, t_np1, information_dictionary, omega_wheel, demand):

        torque_wheel = 0
        fuel_power = 0

        return torque_wheel, fuel_power


class DirectTransmission(Powertrain):

    def __init__(self, motor, transmission_ratio, transmission_efficiency=1, verbose=False):

        self.ratio = [transmission_ratio]
        self.efficiency = transmission_efficiency
        self.motor = motor

        super(DirectTransmission, self).__init__(verbose=verbose)

    def reset(self):
        self.motor.reset()

    def update(self, t_np1, information_dictionary, omega_wheel, demand):

        # Convert wheel speed to motor speed
        omega_motor = omega_wheel * self.ratio[0]

        # Get motor torque
        T_m, fuel_power = self.motor.update(t_np1, omega_motor, demand, information_dictionary)

        # Convert to wheel torque
        T_wheel = T_m * self.ratio[0] * self.efficiency

        # Update information_dictionary
        information_dictionary['Wheel Torque'] = T_wheel
        # information_dictionary['Fuel Power'] = fuel_power

        if self.verbose:
            print('Powertrain')
            print('Wheel Torque: ' + str(T_wheel))

        return T_wheel


class FreeWheel(Powertrain):

    def __init__(self, motor, transmission_ratio, free_wheel_properties, transmission_efficiency=1, verbose=False):

        self.ratio = [transmission_ratio]
        self.efficiency = transmission_efficiency
        self.motor = motor

        # Unpack free wheel properties
        self.motor_shaft_inertia = free_wheel_properties['motor_shaft_inertia']
        self.motor_shaft_viscous = free_wheel_properties['motor_shaft_viscous']
        self.motor_shaft_constant = free_wheel_properties['motor_shaft_constant']

        self.free_wheel_properties = free_wheel_properties

        super(FreeWheel, self).__init__(verbose=verbose)

        # Set up state
        self.reset()

    def reset(self):
        self.motor_omega_n = 0
        self.t_n = 0
        self.motor.reset()

    def update(self, t_np1, information_dictionary, omega_wheel, demand):

        # Convert wheel speed to motor speed
        omega_motor = omega_wheel * self.ratio[0]

        # Get motor torque
        T_m, fuel_power = self.motor.update(t_np1, omega_motor, demand, information_dictionary)

        if demand > 0:
            engaged = True

        else:
            engaged = False
            T_m = 0

        # # Update motor shaft speed
        # y_n = [omega_motor]
        # t_n = self.t_n
        # dt = t_np1 - t_n
        # y_np1 = RK4_step(self._state_equation, self.t_n, y_n, dt, {}, T_m=T_m)
        # omega_motor_np1 = y_np1[0]
        #
        #
        # # Check if free wheel engaged
        # if y_np1 >= omega_motor:
        #     engaged = True
        #     self.motor_omega_n = omega_wheel
        #
        # # Else free wheel not engaged, no torque transmitted
        # else:
        #     engaged = False
        #     self.motor_omega_n = omega_motor_np1
        #     T_m = 0


        wheel_torque = T_m * self.ratio[0] * self.efficiency
        self.t_n = t_np1

        information_dictionary['Motor  Speed'] = self.motor_omega_n
        information_dictionary['Wheel Torque'] = wheel_torque
        information_dictionary['Free Wheel Engaged'] = engaged

        if wheel_torque < 0:
            print()

        return wheel_torque


    def  _state_equation(self, t, y, info_dict, **kwargs):

        T_m = kwargs['kwargs']['T_m']

        omega = y[0]

        dw_dt = (1 / self.motor_shaft_inertia) * (T_m - self.motor_shaft_viscous * omega - self.motor_shaft_constant)

        return dw_dt



