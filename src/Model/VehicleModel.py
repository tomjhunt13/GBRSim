import numpy as np
import time

from src.Model import VehicleRoot

class Vehicle(VehicleRoot.VehicleRoot):

    def __init__(self, powertrain, vehicle_parameters={}, verbose=False):


        super(Vehicle, self).__init__(vehicle_parameters=vehicle_parameters, verbose=verbose)

        # Powertrain
        self.powertrain = powertrain

    def equation_of_motion(self, t, y, information_dictionary, **kwargs):


        if self.verbose:
            print(y)

        # Unpack y
        segment_index = int(np.floor(y[0]))
        lambda_param = y[0] - segment_index

        # Current track segment
        segment = self.track.segments[segment_index]

        # Unpack y
        theta = segment.gradient(lambda_param)  # Road angle (rad)
        segment_scale = self.track.dx_dlambda(segment_index, lambda_param)
        V = y[1] * segment_scale  # Model speed

        # Propulsive force
        throttle_demand = self.control_function(V=V, theta=theta, segment=theta, lambda_param=y[0])
        propulsive_force = self._update_powertrain(t, information_dictionary, V, throttle_demand)

        # Resistive forces
        Fw, Fa, Fc, Frr = self.resistive_forces(theta, V, segment_index, lambda_param)
        resistive_force = Fw + Fa + Fc + Frr

        # Net force
        net_force = (propulsive_force - resistive_force)
        acceleration = net_force / self.m

        # Build state vector
        f = [
            y[1],
            # (1 / segment_scale) * (acceleration)
            (1 / segment_scale) * (acceleration - y[1] * self.track.d_dx_dlambda_dt(segment_index, [lambda_param, y[1]]))
        ]

        information_dictionary['Gradient'] = 100 * (theta / (np.pi / 4))
        information_dictionary['P'] = propulsive_force
        information_dictionary['Frr'] = Frr
        information_dictionary['Fw'] = Fw
        information_dictionary['Fa'] = Fa
        information_dictionary['Fc'] = Fc
        information_dictionary['V'] = V
        information_dictionary['Throttle Demand'] = throttle_demand
        information_dictionary['Segment Scale'] = segment_scale
        information_dictionary['Net Force'] = net_force
        information_dictionary['Acceleration'] = acceleration

        return f

    def _update_powertrain(self, t_np1, information_dictionary, velocity, demand):
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