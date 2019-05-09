from src.Model import ConstrainedParticle

class FreefallingMass(ConstrainedParticle.ConstrainedParticle):

    def __init__(self, g=10, mass=1):

        super(FreefallingMass, self).__init__()

        self.mass = mass
        self.g = g
        self.mg = mass * g

    def update_equation(self, t, state, information_dictionary, **kwargs):

        # Get position on track
        segment, lambda_parameter = self.track.segment_lambda_from_arc_length(state[0])

        # Get weight force
        theta = segment.gradient(lambda_parameter)
        weight_force = -1 * self._weight(theta)

        # Fill in update equation
        dx_dt = [
            state[1],
            weight_force / self.mass
        ]

        return dx_dt

