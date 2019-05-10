class Powertrain:
    def __init__(self, motor, transmission_ratio, transmission_efficiency=1, verbose=False):

        self.ratio = [transmission_ratio]
        self.efficiency = [transmission_efficiency]
        self.motor = motor
        self.verbose = verbose

        self.reset()

    def reset(self):
        self.motor.reset()

    def update(self, t_np1, information_dictionary, omega_wheel, demand):
        pass


class DirectTransmission(Powertrain):

    def __init__(self, motor, transmission_ratio, transmission_efficiency=1, verbose=False):

        super(DirectTransmission, self).__init__(motor=motor,
                                                 transmission_ratio=transmission_ratio,
                                                 transmission_efficiency=transmission_efficiency,
                                                 verbose=verbose)

    def update(self, t_np1, information_dictionary, omega_wheel, demand):

        # Convert wheel speed to motor speed
        omega_motor = omega_wheel * self.ratio[0]

        # Get motor torque
        T_m, fuel_power = self.motor.update(t_np1, omega_motor, demand, information_dictionary)

        # Convert to wheel torque
        T_wheel = T_m * self.ratio[0] * self.efficiency[0]

        # Update information_dictionary
        information_dictionary['Wheel Torque'] = T_wheel

        if self.verbose:
            print('Powertrain')
            print('Wheel Torque: ' + str(T_wheel))

        return T_wheel


class FreeWheel(Powertrain):

    def __init__(self, motor, transmission_ratio, transmission_efficiency=1, verbose=False):

        super(FreeWheel, self).__init__(motor=motor,
                                        transmission_ratio=transmission_ratio,
                                        transmission_efficiency=transmission_efficiency,
                                        verbose=verbose)

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

        wheel_torque = T_m * self.ratio[0] * self.efficiency[0]

        information_dictionary['Wheel Torque'] = wheel_torque
        information_dictionary['Free Wheel Engaged'] = engaged

        return wheel_torque


