from src.Track import ImportTrack
from src.Optimisation.Optimisation import *
# from src.Powertrain.Powertrain_original import *
from src.Control import *
from src.Vehicle.Vehicle import *
from src.Powertrain.BrushedDCMotor import *
from src.Powertrain.Transmission import *

from src.Powertrain.Powertrain import *

track = ImportTrack.import_year('2019')

# transmission = Transmission(26.42073489139331, 0.93)
# transmission = Transmission(10, 0.93)

motor = Moog_C42_L90_10(verbose=True)
powertrain = DirectTransmission(motor, 10, transmission_efficiency=0.93)

# control = CutoffSpeed(8)
# control = BurnAndCoast_Velocity(min_vel=2, max_vel=4)
control = ConstantPower()


v = Vehicle(powertrain)

# OptimiseTransmissionRatio(15, v, track, control.demand)

# total_time = 45 * 60
total_time = 39 * 60
laps = 11
desired_time = total_time / laps
print(desired_time)
#
# optimisation_result = OptimiseTransmissionRatio_SpecificTime(2, desired_time, v, track, control.demand)
# ratio = optimisation_result['x'][0]
#
# transmission.ratio = ratio

#
# print(optimisation_result[0])
# optimisation_result = minimize(TransmissionRatio_MinMax_TimeCost, [10, 1.7546696194154812, 10.106020212266744], args=(v, track, control, 600, desired_time), method='Nelder-Mead')

vehicle_results = v.simulate(track, 0, [0, 0], control_function=control.demand, time_step=0.01, time_limit=1000, lap_limit=1)
# print(vehicle_results[0][-1])
# vehicle_results = v.simulate(track, 0, [0, 0], control_function=control.demand, time_step=0.05, time_limit=600)
# print(vehicle_results[0][-1])
# vehicle_results = v.simulate(track, 0, [0, 0], control_function=control.demand, time_step=0.025, time_limit=600)
# print(vehicle_results[0][-1])
# vehicle_results = v.simulate(track, 0, [0, 0], control_function=control.demand, time_step=0.01, time_limit=600)
# print(vehicle_results[0][-1])


Animate(track, vehicle_results)


