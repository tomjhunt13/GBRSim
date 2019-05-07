import numpy as np
import time

from src.Model import VehicleRoot

class Vehicle(VehicleRoot.VehicleRoot):

    def __init__(self, powertrain, vehicle_parameters={}, verbose=False):

        super(Vehicle, self).__init__(vehicle_parameters=vehicle_parameters, verbose=verbose)
        self.powertrain = powertrain

    def _propulsive_force(self, t_np1, y_n, velocity, demand, information_dictionary):
        """
        :param velocity: Linear vehicle velocity
        :param demand:
        :return:
        """

        # Convert linear velocity to wheel rotational speed
        omega_wheel = (1 / self.PoweredWheelRadius) * velocity

        # Get wheel torque
        wheel_torque = self.powertrain.update(t_np1, information_dictionary, omega_wheel, demand)

        # Linear force
        linear_force = wheel_torque / self.PoweredWheelRadius

        return linear_force