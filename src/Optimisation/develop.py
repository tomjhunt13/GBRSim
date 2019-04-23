import numpy as np

from src.Track import ImportTrack
from src.Optimisation import Optimiser
from src import Control
from src.Vehicle import Vehicle
from src.Powertrain import BrushedDCMotor
# from src.Results import Results
from src.Powertrain import Powertrain


class Simulation:

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

# def RunSimulation():
#




track = ImportTrack.import_year('2018')



motor = BrushedDCMotor.Moog_C42_L90_10(verbose=False)
powertrain = Powertrain.DirectTransmission(motor, 12, transmission_efficiency=0.8)
control = Control.BurnAndCoast_Velocity()
v = Vehicle.Vehicle(powertrain)


total_time = 45 * 60
# total_time = 39 * 60
# laps = 11
laps = 15
desired_time = total_time / laps
# print(desired_time)

sim = Simulation(v.simulate, track, control.demand, desired_time)


optimiser = Optimiser.Optimiser()

optimiser.AddVariable('Transmission Ratio', powertrain.ratio, 5, 15)
optimiser.AddVariable('MinVel', control.min_vel, 1, 7)
optimiser.AddVariable('MaxVel', control.min_vel, 10, 25)
optimum = optimiser.Optimise(sim.cost)




