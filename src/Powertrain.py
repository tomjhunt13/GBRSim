from math import pi

class Powertrain:
    def __init__(self):

        # Torque velocity curve
        self.torque = [[50, 70, 80, 85, 80, 70], [0, 100, 200, 300, 400, 500]]
        self.engine_efficiency = 0.3

        # Transmission
        self.transmission_ratio = 2
        self.transmission_efficiency = 0.9

    def power(self, wheel_speed, demand):
        """

        :param wheel_speed: Rotational speed of wheel (radians / second)
        :param demand: Throttle demand between 0 and 1
        :return:
        """

        # Convert wheel speed to engine speed
        engine_speed = wheel_speed * self.transmission_ratio

        # Get the torque produced by engine - Temporary!
        engine_torque_max = InterpolateList(engine_speed, self.torque[1], self.torque[0])


        engine_torque = demand * engine_torque_max

        # Fuel use
        engine_power = engine_torque * engine_speed
        fuel_power = engine_power / (self.engine_efficiency * self.transmission_efficiency)

        return engine_torque, fuel_power

class BrushedMotor:
    def __init__(self, stall_torque, no_load_speed, transmission_ratio=1, transmission_efficiency=0.9, motor_efficiency=0.9):
        """

        :param stall_torque: Torque of motor at 0 rpm (Nm)
        :param no_load_speed: Angular speed at no load (rpm)
        """

        self.stall_torque = stall_torque
        self.no_load_speed = no_load_speed * pi * 2 / 60
        self.motor_efficiency = motor_efficiency

        # Transmission
        self.transmission_ratio = transmission_ratio
        self.transmission_efficiency = transmission_efficiency

    def power(self, wheel_speed, demand):
        """

        :param wheel_speed: Rotational speed of wheel (radians / second)
        :param demand: Throttle demand between 0 and 1
        :return:
        """

        # Check for negative velocity
        if wheel_speed < 0:
            return self.stall_torque, 0

        # Convert wheel speed to engine speed
        motor_speed = wheel_speed * self.transmission_ratio

        # Check for too fast
        if motor_speed > self.no_load_speed:
            return 0, 0

        # Get the torque produced by motor
        max_torque = self.stall_torque - motor_speed * (self.stall_torque / self.no_load_speed)
        torque = demand * max_torque

        # Energy use
        engine_power = torque * motor_speed
        real_power = engine_power / (self.motor_efficiency * self.transmission_efficiency)

        return torque, real_power



def InterpolateList(x, list_x, list_y):

    if x < list_x[0]:
        return list_y[0]

    index = 0
    while list_x[index] < x:
        index += 1

        if index > len(list_x) - 1:
            return list_y[-1]

    # Interpolate

    return list_y[index]