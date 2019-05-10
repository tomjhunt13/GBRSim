import unittest
import numpy as np

from src.Model.tests import Models
from src.Track import Track, Line, Circle

class TestArcLengthIntegration(unittest.TestCase):
    """

    """

    def setUp(self):

        # ------- Vertical track test ------- #

        self.vertical_track_height = 200

        # Set up track
        line = Line.Line([[0, 0, self.vertical_track_height], [0, 0, 0]])
        self.vertical_track = Track.Track([line])

        # ------- Circular ------- #
        self.radius = 1

        # Set up track
        vertical_circle = Circle.VerticalCircle(self.radius)
        self.circular_track = Track.Track([vertical_circle])

    def test_vertical_free_fall_from_top(self):
        """
        In equilibrium:

            1/2 * rho * A * Cd * V^2 = mg

            let rho = A = Cd = 1

            => V = sqrt(2 m g)

            let m = 1, g = 2

            => V_equilibrium = 2

        """

        # Simulation parameters
        g = 2
        mass = 1
        particle = Models.FreefallWithAeroResistance(g=g, mass=mass, aero_force=0.5)

        # Run simulation
        model_results = particle.simulate([0, 0], track=self.vertical_track, dt=0.25, t_end=1000, verbose=False)

        # Unpack result
        t = [y['t'] for y in model_results[1:]]
        y = [i['y'] for i in model_results[1:]]

        self.assertAlmostEqual(y[-1][1], 2, places=4)


if __name__ == '__main__':
    unittest.main()