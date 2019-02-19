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
    def __init__(self, torque_constant, transmission_ratio=125, transmission_efficiency=0.9, motor_efficiency=0.85):
        """

        :param torque_constant: Torque constant of motor (Nm/A)
        """

        self.torque_constant = torque_constant
        self.motor_efficiency = motor_efficiency

        # Transmission
        self.transmission_ratio = transmission_ratio
        self.transmission_efficiency = transmission_efficiency


        # Battery properties
        self.max_discharge_power = 1000
        self.discharge_voltage = 24
        self.battery_efficiency = 0.9

    def power(self, wheel_speed, demand):
        """

        :param wheel_speed: Rotational speed of wheel (radians / second)
        :param demand: Throttle demand between 0 and 1
        :return:
        """

        # Current
        current = demand * ((self.max_discharge_power * self.battery_efficiency) / self.discharge_voltage)

        # Torque
        output_torque = self.transmission_efficiency * self.motor_efficiency * current * self.torque_constant * self.transmission_ratio

        #  Power consumed
        power = demand * self.max_discharge_power

        return output_torque, power



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