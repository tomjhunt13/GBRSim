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