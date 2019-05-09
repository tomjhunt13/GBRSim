import unittest
import numpy as np

from src.Model.tests import Models
from src.Track import Track, Circle

class TestArcLengthIntegration(unittest.TestCase):
    """

    """

    def setUp(self):

        # ------- Circular ------- #
        self.radius = 5

        # Set up track
        vertical_circle = Circle.VerticalCircle(self.radius)
        self.circular_track = Track.Track([vertical_circle])

    def test_circular_force_mass_a_1(self):
        """
        F, a, m = 1

        In equilibrium mg sin(theta) = F

        => theta = 90 degrees (lambda = 0)

        """

        # Simulation parameters
        g = 1
        mass = 1
        F = 1
        particle = Models.ConstantForce(g=g, force=F, mass=mass)

        # Run simulation
        model_results = particle.simulate([0, 0], track=self.circular_track, dt=0.05, t_end=100, verbose=False)

        # Unpack result
        t = [y['t'] for y in model_results[1:]]
        y = [y['y'] for y in model_results[1:]]
        arc_length = [y['y'][0] for y in model_results[1:]]
        arc_length_final = np.interp(1, t, arc_length)

        self.assertAlmostEqual(arc_length_final, 5, places=4)


if __name__ == '__main__':
    unittest.main()