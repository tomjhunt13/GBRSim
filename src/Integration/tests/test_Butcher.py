import numpy as np
import unittest

from src.Integration import Butcher
from src.Integration.tests import IVP_test_cases

class TestButcher(unittest.TestCase):

    def test_wikipedia_IVP(self):

        ivp = IVP_test_cases.IVP()

        s = Butcher.Butcher(A=[[], [2. / 3.]],  b=[1. / 4., 3. / 4.], h=[0, 2. / 3.])

        results = s.solve(ivp, ivp.wikipedia_f, {}, [1], dt=0.025, t_start=1, t_end=1.1)

        self.assertAlmostEqual(results[1]['y'][0], 1.066869388, places=5)

    def test_RK4_func_IVP(self):
        ivp = IVP_test_cases.IVP()

        s = Butcher.Butcher()

        results = s.solve(ivp, ivp.rk4_test_func, {}, [1], dt=0.1, t_start=0, t_end=1)

        self.assertAlmostEqual(results[1]['y'][0], 0.9655827899)
        self.assertAlmostEqual(results[2]['y'][0], 0.937796275)
        self.assertAlmostEqual(results[3]['y'][0], 0.9189181059)

    def test_seperation_of_variables_RK4(self):
        ivp = IVP_test_cases.IVP()

        s = Butcher.RK4()

        results = s.solve(ivp, ivp.separation_of_variables_grad, {}, [1], dt=0.01, t_start=0, t_end=1)

        for result_t in results[1:]:

            self.assertAlmostEqual(result_t['y'][0], ivp.separation_of_variables_analytical(result_t['t'], 1), places=5)

    def test_separation_of_variables_RK8(self):
        ivp = IVP_test_cases.IVP()

        s = Butcher.RK8()

        results = s.solve(ivp, ivp.separation_of_variables_grad, {}, [1], dt=0.5, t_start=0, t_end=1)

        for result_t in results[1:]:

            self.assertAlmostEqual(result_t['y'][0], ivp.separation_of_variables_analytical(result_t['t'], 1), places=5)



if __name__ == '__main__':
    unittest.main()