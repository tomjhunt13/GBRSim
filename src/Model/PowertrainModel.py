import numpy as np

class BrushedDCMotor:
    def __init__(self, motor_parameters={}):

        # Default attributes
        default_motor_attributes = {
            # Motor properties
            'motor_torque_constant': 0.001 * 123,
            'motor_speed_constant': 1 / (77.8 * (2 * np.pi / 60)),
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
        self.motor_torque_constant = motor_parameters['motor_torque_constant']
        self.motor_speed_constant = motor_parameters['motor_speed_constant']
        self.R = motor_parameters['R']
        self.L = motor_parameters['L']
        self.Power = motor_parameters['Power']

        # Battery properties
        self.V_max = motor_parameters['V_max']
        self.battery_efficiency = motor_parameters['Battery_Efficiency']


        def equation_of_motion(self, t, y, information_dictionary, **kwargs):

            # Unpack kwargs
            V = kwargs['V']
            omega = kwargs['omega']

            

            return [(1 / self.L) * (V - y[0] * self.R - self.motor_speed_constant * omega)]


