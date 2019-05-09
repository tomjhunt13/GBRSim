import unittest
import numpy as np

from src.Model.tests import Models
from src.Track import Track, Line, Circle

class TestArcLengthIntegration(unittest.TestCase):
    """

    """

    def setUp(self):

        # ------- Vertical track test ------- #
        # Set up track
        line = Line.Line([[0, 0, 0.5], [0, 0, 0]])
        self.vertical_track = Track.Track([line])

        # # ------- Circular ------- #
        # self.radius = 1
        #
        # # Set up track
        # vertical_circle = Circle.VerticalCircle(self.radius)
        # self.circular_track = Track.Track([vertical_circle])

    def test_vertical_free_fall_from_top(self):
        """
        Using SUVAT:

            v^2 = u^2 + 2 a s

            u = 0
            let a = 1
            let s = 2 (4 laps)

            => v = 2

            v = u + a t

            t = 2
        """

        # Simulation parameters
        g = 1
        mass = 1
        particle = Models.FreefallingMass(g=g, mass=mass)

        # Run simulation
        model_results = particle.simulate([0, 0], track=self.vertical_track, dt=1e-3, t_end=4, verbose=False, lap_limit=4)

        # Unpack result
        t = [y['t'] for y in model_results[1:]]
        y = [i['y'] for i in model_results[1:]]
        arc_length = [i[0] for i in y]

        self.assertAlmostEqual(t[-1], 2, places=2)
        self.assertAlmostEqual(arc_length[-1], 0, places=2)
        self.assertAlmostEqual(particle.net_distance, 2, places=2)


if __name__ == '__main__':
    unittest.main()