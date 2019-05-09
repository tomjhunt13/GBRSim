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

        self.track_length = self.track.total_length()

    def cost(self):

        vehicle_results = self.car.simulate([1e-4, 1e-4], dt=0.25, t_end=self.max_sim_time, verbose=False,
                                             track=self.track, controller=self.controller)

        fuel_power = [d['Fuel Power'] for d in vehicle_results]
        t = [d['t'] for d in vehicle_results]
        arc_length = [d['y'][0] for d in vehicle_results[1:]]
        energy = np.trapz(fuel_power, t)
        time = t[-1]

        time_multiplier = 1 + max(time - self.max_lap_time, 0)
        distance = self.car.net_distance
        completion_multiplier = max(0, self.track.length - distance)

        cost = energy * time_multiplier + completion_multiplier * 3600000

        if self.verbose:
            print('Completion Time: ' + str(time) + ', Energy: ' + str(energy) + ', Cost: ' + str(cost) + ', Distance: ' + str(distance))

        return energy * time_multiplier



