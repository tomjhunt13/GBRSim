import numpy as np

from src.Model import VehicleRoot

import time

class IntegratedModel(VehicleRoot.VehicleRoot):

    def __init__(self, vehicle_parameters={}, verbose=False):

        # Default attributes
        default_vehicle_attributes = {
            # Model properties
            'Mass': 170,
            'Crr': 1.5 * 0.001,     # http://www.eshopsem.com/boutique/product.php?id_product=75
            'Cd': 0.2,
            'A': 1.26,
            'PoweredWheelRadius': 0.279,
            'LongitudinalCoG': 0.5,     # Assume even weight distribution

            # Motor properties
            'motor_torque_constant': 0.001 * 123,
            'motor_speed_constant': 1 / (77.8 * (2 * np.pi / 60)),
            'R': 0.365,
            'L': 0.161 * 0.001,

            # Battery properties
            'Power': 250,
            'V_max': 48,
            'Battery_Efficiency': 0.9,

            # Transmission properties
            'transmission_ratio': 10,
            'transmission_efficiency': 0.8,
        }

        for attribute in default_vehicle_attributes.keys():
            if attribute not in vehicle_parameters:
                vehicle_parameters[attribute] = default_vehicle_attributes[attribute]

        super(IntegratedModel, self).__init__(vehicle_parameters=vehicle_parameters, verbose=verbose)

        # Motor properties
        self.motor_torque_constant = vehicle_parameters['motor_torque_constant']
        self.motor_speed_constant = vehicle_parameters['motor_speed_constant']
        self.R = vehicle_parameters['R']
        self.L = vehicle_parameters['L']
        self.Power = vehicle_parameters['Power']

        # Battery properties
        self.V_max = vehicle_parameters['V_max']
        self.battery_efficiency = vehicle_parameters['Battery_Efficiency']

        # Transmission properties
        self.transmission_ratio = vehicle_parameters['transmission_ratio']
        self.transmission_efficiency = vehicle_parameters['transmission_efficiency']

    def _further_calculations(self, y, dy_dt, velocity, throttle_demand, information_dictionary):

        # Update current
        omega_motor = (self.transmission_ratio / self.PoweredWheelRadius) * velocity
        back_emf = self.motor_speed_constant * omega_motor
        V_max = max(np.roots([1, -1 * (back_emf), -1 * self.Power * self.R]))

        if V_max > self.V_max:
            V_max = self.V_max

        motor_voltage = V_max * throttle_demand
        electrical_power = (motor_voltage * y[2]) / self.battery_efficiency

        if self.verbose:
            print('Back EMF: ' + str(back_emf))

        # Append to state vector
        dy_dt.append((1 / self.L) * (motor_voltage - y[2] * self.R - self.motor_speed_constant * omega_motor))

        information_dictionary['Motor Current'] = y[2]
        information_dictionary['Fuel Power'] = max(0, electrical_power)

    def _propulsive_force(self, t_np1, y_n, velocity, demand, information_dictionary):

        motor_torque = self.motor_torque_constant * y_n[2]
        propulsive_force = (1 / self.PoweredWheelRadius) * self.transmission_efficiency * \
                           self.transmission_ratio * motor_torque

        return propulsive_force