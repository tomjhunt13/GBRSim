import unittest

from src.Integration import RKF45
from src.Integration.tests import IVP_test_cases

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

    def test_separation_of_variables(self):
        ivp = IVP_test_cases.IVP()

        s = RKF45.RKF45()

        results = s.solve(ivp, ivp.separation_of_variables_grad, {}, [1], dt=0.005, t_start=0, t_end=3)

        for result_t in results[1:]:

            self.assertAlmostEqual(result_t['y'][0], ivp.separation_of_variables_analytical(result_t['t'], 1), places=1)


if __name__ == '__main__':
    unittest.main()