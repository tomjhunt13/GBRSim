import unittest
import numpy as np

from src.Model.tests import Models
from src.Integration import RK4
from src.Track import Track, Line

class TestArcLengthIntegration(unittest.TestCase):
    """

    """

    def setUp(self):

        # ------- Vertical track test ------- #

        self.vertical_track_height = 5

        # Set up track
        line_1 = Line.Line([[0, 0, self.vertical_track_height], [0, 0, (3. / 4.) * self.vertical_track_height]])
        line_2 = Line.Line([[0, 0, (3. / 4.) * self.vertical_track_height], [0, 0, (2. / 4.) * self.vertical_track_height]])
        line_3 = Line.Line([[0, 0, (2. / 4.) * self.vertical_track_height], [0, 0, (1. / 4.) * self.vertical_track_height]])
        line_4 = Line.Line([[0, 0, (1. / 4.) * self.vertical_track_height], [0, 0, 0]])
        self.vertical_track = Track.Track([line_1, line_2, line_3, line_4])

    def test_vertical_free_fall(self):
        """
        Using SUVAT:

            s = ut + 1/2 * at^2,    u = 0

            => t =  sqrt(2 * s / a)

            s = 5, a = 10

            => t_end = 1

            v = sqrt(2 * a  * s)
        """

        # Simulation parameters
        g = 10
        mass = 1
        particle = Models.FreefallingMass(g=g, mass=mass)

        # Run simulation
        model_results = particle.simulate([0, 0], track=self.vertical_track, dt=0.001, t_end=1, verbose=False)

        # Unpack result
        t = [y['t'] for y in model_results[1:]]
        arc_length = [y['y'][0] for y in model_results[1:]]
        arc_length_final = np.interp(1, t, arc_length)

        self.assertAlmostEqual(arc_length_final, 1, places=4)

if __name__ == '__main__':
    unittest.main()