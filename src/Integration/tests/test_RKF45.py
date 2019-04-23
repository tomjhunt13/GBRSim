import unittest

from src.Integration import RKF45

class TestRKF45(unittest.TestCase):

    def test_RKF45_weighting_scalars(self):

        k_1 = 1
        k_2 = 1
        k_3 = 1
        k_4 = 1
        k_5 = 1
        k_6 = 1

        dy_4, dy_5 = RKF45.RKF45_weighting(k_1, k_2, k_3, k_4, k_5, k_6)

        self.assertAlmostEqual(dy_4, 1.,  places=3)
        self.assertAlmostEqual(dy_5, 1.)

    def test_RKF45_weighting_vectors(self):

        k_1 = [1, 1]
        k_2 = [1, 1]
        k_3 = [1, 1]
        k_4 = [1, 1]
        k_5 = [1, 1]
        k_6 = [1, 1]

        dy_4, dy_5 = RKF45.RKF45_weighting(k_1, k_2, k_3, k_4, k_5, k_6)

        self.assertAlmostEqual(dy_4[0], 1.,  places=3)
        self.assertAlmostEqual(dy_4[1], 1., places=3)
        self.assertAlmostEqual(dy_5[0], 1.)
        self.assertAlmostEqual(dy_5[1], 1.)

if __name__ == '__main__':
    unittest.main()