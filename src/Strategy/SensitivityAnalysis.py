import numpy as np

from types import MethodType

# Model
from src.Track import ImportTrack
from src.Model import SeparatedVehicleModel
from src.Model import PowertrainModel
from src.Model import BrushedDCMotor

# Strategy
from src.Strategy import SimulationWrapper
from src.Strategy import Controller

# Optimiser
from src.Optimisation import Sensitivity


def sensitivity_cost(self):

    vehicle_results = self.simulate()

    fuel_power = [d['Fuel Power'] for d in vehicle_results]
    t = [d['t'] for d in vehicle_results]
    energy = np.trapz(fuel_power, t)
    time = t[-1]
    arc_length = [d['Arc Length (m)'] for d in vehicle_results]
    distance = max(arc_length) / 1000

    print('Time: ' + str(time))

    return distance /  (energy / 3.6e6)


# ------- Import Track ------- #

# 2019
track = ImportTrack.import_year('2019')
total_time = 39 * 60
laps = 11

# 2018
# track = ImportTrack.import_year('2018')
# total_time = 35 * 60
# laps = 15

desired_time = (total_time / laps) - 5

# ------- Make Model ------- #

# Model parameters
transmission_efficiency = 0.7
vehicle_mass = 150
Cd = 0.4
battery_efficiency = 0.8
driver_mass = 70

controller = Controller.ConstantPower()

motor = BrushedDCMotor.MaxonRE65(dt=1e-3, verbose=False, battery_efficiency=battery_efficiency)
powertrain = PowertrainModel.FreeWheel(motor, 10, transmission_efficiency=transmission_efficiency, verbose=False)
car = SeparatedVehicleModel.SeparatedVehicleModel(powertrain, vehicle_parameters={'VehicleMass': vehicle_mass, 'Cd': Cd, 'DriverMass': driver_mass}, verbose=False)
sim = SimulationWrapper.SimulationWrapper(car, track, controller, desired_time)


# ------- Sensitivity ------- #
sim.sensitivity_cost = MethodType(sensitivity_cost, sim)

sensitivity = Sensitivity.Sensitivity()
sensitivity.add_variable('Transmission Efficiency', powertrain.efficiency, dx=transmission_efficiency*0.01)
sensitivity.add_variable('Mass', car.vehicle_mass, dx=vehicle_mass*0.01)
sensitivity.add_variable('Cd', car.Cd, dx=Cd*0.01)
sensitivity.add_variable('Battery Efficiency', motor.battery_efficiency, dx=battery_efficiency*0.01)

sensitivity.sensitivity(sim.sensitivity_cost)









