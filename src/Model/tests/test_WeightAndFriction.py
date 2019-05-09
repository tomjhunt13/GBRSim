import unittest
import numpy as np

from src.Model.tests import Models
from src.Track import Track, Line, Circle

class TestArcLengthIntegration(unittest.TestCase):
    """

    """

    def setUp(self):
        # ------- Circular ------- #
        self.radius = 1

        # Set up track
        vertical_circle = Circle.VerticalCircle(self.radius)
        self.circular_track = Track.Track([vertical_circle])

    def test_circular_mass_a_1(self):
        """
        Starting on vertical:

        1/2 m v^2  = mgh

        => v_max = sqrt(2 * g * h)
        """

        # Simulation parameters
        g = 1
        mass = 1
        friction_coefficient = 0.05
        particle = Models.FreefallWithFriction(g=g, mass=mass, friction_coefficient=friction_coefficient)

        # Run simulation
        model_results = particle.simulate([0, 0],
                                          track=self.circular_track, dt=0.25, t_end=10000, verbose=False)

        # Unpack result
        y = [y['y'] for y in model_results[1:]]
        arc_length = [i[0] for i in y]

        final_lambda = self.circular_track.segments[0].lambda_from_arc_length(arc_length[-1])

        self.assertAlmostEqual(final_lambda, 0.75, places=3)



if __name__ == '__main__':
    unittest.main()