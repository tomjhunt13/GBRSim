import numpy as np

from src.Model import ConstrainedParticle

class FreefallingMass(ConstrainedParticle.ConstrainedParticle):

    def __init__(self, g=10, mass=1):

        super(FreefallingMass, self).__init__()

        self.mass = mass
        self.g = g
        self.mg = mass * g

    def update_equation(self, t, state, information_dictionary, **kwargs):

        # Get position on track
        segment, segment_index, lambda_parameter = self.track.segment_lambda_from_arc_length(state[0])

        # Get weight force
        theta = segment.gradient(lambda_parameter)
        weight_force = self.weight_force(theta)

        # Fill in update equation
        dx_dt = [
            state[1],
            weight_force / self.mass
        ]

        return dx_dt


class ConstantForce(ConstrainedParticle.ConstrainedParticle):

    def __init__(self, g=10, mass=1, force=1):

        super(ConstantForce, self).__init__()

        self.mass = mass
        self.g = g
        self.mg = mass * g
        self.force = force

    def update_equation(self, t, state, information_dictionary, **kwargs):

        # Get position on track
        segment, segment_index, lambda_parameter = self.track.segment_lambda_from_arc_length(state[0])

        # Get weight force
        theta = segment.gradient(lambda_parameter)
        weight_force = self.weight_force(theta)

        # Net force
        net_force = self.force + weight_force

        # Fill in update equation
        dx_dt = [
            state[1],
            net_force / self.mass
        ]

        return dx_dt


class FreefallWithFriction(ConstrainedParticle.ConstrainedParticle):

    def __init__(self, g=10, mass=1, friction_coefficient=1):

        super(FreefallWithFriction, self).__init__()

        self.mass = mass
        self.g = g
        self.mg = mass * g
        self.friction_coefficient = friction_coefficient

    def update_equation(self, t, state, information_dictionary, **kwargs):

        # Get position on track
        segment, segment_index, lambda_parameter = self.track.segment_lambda_from_arc_length(state[0])

        # Get net force
        theta = segment.gradient(lambda_parameter)
        weight_force = self.weight_force(theta)

        if state[1] != 0:
            friction_force = -1 * np.sign(state[1]) * self.friction_coefficient
        else:
            friction_force = 0

        net_force = friction_force + weight_force

        # Fill in update equation
        dx_dt = [
            state[1],
            net_force / self.mass
        ]

        return dx_dt

class FreefallWithAeroResistance(ConstrainedParticle.ConstrainedParticle):

    def __init__(self, g=10, mass=1, aero_force=1):

        super(FreefallWithAeroResistance, self).__init__()

        self.mass = mass
        self.g = g
        self.mg = mass * g
        self.aero_force = aero_force

    def update_equation(self, t, state, information_dictionary, **kwargs):

        # Get position on track
        segment, segment_index, lambda_parameter = self.track.segment_lambda_from_arc_length(state[0])

        # Get net force
        theta = segment.gradient(lambda_parameter)
        weight_force = self.weight_force(theta)
        direction_mod = ConstrainedParticle.direction_modifier(state[1])
        aero_drag = -1 * direction_mod * self.aerodynamic_drag_force(state[1])


        net_force = weight_force + aero_drag

        # Fill in update equation
        dx_dt = [
            state[1],
            net_force / self.mass
        ]

        return dx_dt

