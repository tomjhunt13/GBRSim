from src.Model import VehicleRoot

class SeparatedVehicleModel(VehicleRoot.VehicleRoot):

    def __init__(self, powertrain, vehicle_parameters={}, verbose=False):

        super(SeparatedVehicleModel, self).__init__(vehicle_parameters=vehicle_parameters, verbose=verbose)
        self.powertrain = powertrain

    def _propulsive_force(self, t_np1, y_n, demand, information_dictionary):

        # Convert linear velocity to wheel rotational speed
        omega_wheel = self.velocity_to_omega_wheel(y_n[1])

        # Get wheel torque
        wheel_torque = self.powertrain.update(t_np1, information_dictionary, omega_wheel, demand)

        # Linear force
        linear_force = self.wheel_torque_to_linear(wheel_torque)

        return linear_force