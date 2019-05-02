import numpy as np

from src.Integration import RK4, Butcher

class SimulationWrapper:

    def __init__(self, car, track, controller, max_lap_time, solver=RK4.RK4, max_sim_time=300, verbose=True):
        self.car = car
        self.track = track
        self.controller = controller
        self.max_lap_time = max_lap_time
        self.max_sim_time = max_sim_time
        self.solver = solver
        self.verbose = verbose

    def cost(self):

        model_kwargs = {'track': self.track, 'controller': self.controller}
        solver = self.solver()
        vehicle_results = solver.solve(self.car, self.car.equation_of_motion, model_kwargs, [1e-4, 1e-4], dt=0.25,
                                       t_end=self.max_sim_time, verbose=False)

        fuel_power = [d['Fuel Power'] for d in vehicle_results]
        t = [d['t'] for d in vehicle_results]
        energy = np.trapz(fuel_power, t)
        time = t[-1]

        multiplier = 1 + max(time - self.max_lap_time, 0)

        cost = energy * multiplier

        if self.verbose:
            print('Completion Time: ' + str(time) + ', Energy: ' + str(energy) + ', Cost: ' + str(cost))

        return energy * multiplier