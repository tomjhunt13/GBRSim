import time
from src.Track import ImportTrack, Track, Line, Circle
from src.Strategy import Controller
from src.Model import VehicleModel, PowertrainModel, BrushedDCMotor, IntegratedModel
from src.Results import Results

"""
To do:

1) Make parent to optimiser which handles variable changes

2) Look for telemetry from previous GBR


"""

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

controller = Controller.ConstantPower()
transmission_ratio = 12

motor = BrushedDCMotor.MaxonRE65(dt=1e-3, verbose=False, battery_efficiency=0.9)
powertrain = PowertrainModel.DirectTransmission(motor, transmission_ratio, transmission_efficiency=0.8, verbose=False)

vehicle_parameters = {'Cd': 0.3, 'VehicleMass': 100}

car = VehicleModel.Vehicle(powertrain, verbose=True, vehicle_parameters=vehicle_parameters)

# car = IntegratedModel.IntegratedModel({'transmission_ratio': transmission_ratio}, verbose=True)


t_s = time.time()
vehicle_results = car.simulate([1e-4, 1e-4], dt=0.25, t_end=300, verbose=True, track=track, controller=controller)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)


