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
    def __init__(self, motor_properties, battery_properties, transmission_properties, number_of_motors=1):
        """
        Approximate brushed motor model without resistance
        :param motor_properties:

                Dictionary of form - {'torque_constant': __ (Nm/A), 'motor_efficiency': __ }

        :param battery_properties:

                Dictionary of form - {'max_discharge_power': __ (W), 'discharge_voltage': __ (V), 'battery_efficiency': __}

        :param transmission_properties:

                Dictionary of form - {'transmission_ratio': __, 'transmission_efficiency': __}

        :param number_of_motors: Number of motors used
        """


        # Motor properties
        self.torque_constant = motor_properties['torque_constant']
        self.motor_efficiency = motor_properties['motor_efficiency']
        self.number_of_motors = number_of_motors

        # Battery properties
        self.max_discharge_power = battery_properties['max_discharge_power']
        self.discharge_voltage = battery_properties['discharge_voltage']
        self.battery_efficiency = battery_properties['battery_efficiency']

        # Transmission
        self.transmission_ratio = transmission_properties['transmission_ratio']
        self.transmission_efficiency = transmission_properties['transmission_efficiency']




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

        return self.number_of_motors * output_torque, self.number_of_motors * power



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