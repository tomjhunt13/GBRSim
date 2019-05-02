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

    def __init__(self, motor, transmission_ratio, transmission_efficiency=1, verbose=False):

        self.ratio = [transmission_ratio]
        self.efficiency = transmission_efficiency
        self.motor = motor

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
        if demand > 0:
            engaged = True
            T_m, fuel_power = self.motor.update(t_np1, omega_motor, demand, information_dictionary)

        else:
            engaged = False
            self.motor.update(t_np1, 0, demand, information_dictionary)
            T_m = 0

        wheel_torque = T_m * self.ratio[0] * self.efficiency
        self.t_n = t_np1

        information_dictionary['Motor  Speed'] = self.motor_omega_n
        information_dictionary['Wheel Torque'] = wheel_torque
        information_dictionary['Free Wheel Engaged'] = engaged

        return wheel_torque


