import numpy as np

class SimulationWrapper:

    def __init__(self, sim_function, track, control_function, max_time):
        self.sim_function = sim_function
        self.track = track
        self.control_function = control_function
        self.max_time = max_time


    def cost(self):

        vehicle_results = self.sim_function(self.track, 0, [0, 0], control_function=self.control_function, time_step=0.02, time_limit=500, lap_limit=1)
        fuel_power = [d['Fuel Power'] for d in vehicle_results]
        t = [d['t'] for d in vehicle_results]
        energy = np.trapz(fuel_power, t)
        time = t[-1]

        multiplier = 1 + max(time - self.max_time, 0)
        return energy * multiplier