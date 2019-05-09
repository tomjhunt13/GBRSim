from src.Model import ConstrainedParticle

class FreefallingMass(ConstrainedParticle.ConstrainedParticle):

    def __init__(self, g=10, mass=1):

        super(FreefallingMass, self).__init__()

        self.mass = mass
        self.g = g
        self.weight = mass * g

    def update_equation(self, t, y, information_dictionary, **kwargs):

        dx_dt = [
            y[1],
            self.weight / self.mass
        ]

        return dx_dt

