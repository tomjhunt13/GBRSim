import numpy as np

# Model
from src.Track import ImportTrack
from src.Model import VehicleModel
from src.Model import PowertrainModel
from src.Model import BrushedDCMotor

# Optimisation
from src.Strategy import SimulationWrapper
from src.Strategy import Controller

# Optimiser
from src.Optimisation import SA


# Create model
track = ImportTrack.import_year('2018')
controller = Controller.BurnAndCoast(number_of_burns=3)
controller = Controller.ConstantPower()

motor = BrushedDCMotor.MaxonRE65(dt=1e-3, verbose=False)
powertrain = PowertrainModel.FreeWheel(motor, 10, transmission_efficiency=0.8, verbose=False)
car = VehicleModel.Vehicle(powertrain, verbose=False)



total_time = 45 * 60
# total_time = 39 * 60
# laps = 11
laps = 15
desired_time = total_time / laps

sim = SimulationWrapper.SimulationWrapper(car, track, controller, desired_time)





optimiser = SA.SA()

optimiser.AddVariable('Transmission Ratio', powertrain.ratio, 5, 15)
# optimiser.AddVariable('MinVel', control.min_vel, 1, 7)
# optimiser.AddVariable('MaxVel', control.min_vel, 10, 25)
optimum = optimiser.Optimise(sim.cost, max_iterations=100)

print(optimum)




