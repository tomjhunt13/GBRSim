from src.Powertrain import *
from src.Vehicle.Vehicle import *
from src.Results import *
from src.Control import *
from src.Track import ImportTrack
from src.Optimisation.Optimisation import *



track = ImportTrack.import_year('2018')

# transmission = Transmission(26.42073489139331, 0.93)
transmission = Transmission(9, 0.93)
# powertrain = MaxonRE65()
powertrain = Moog_C42_L90_10()

# control = CutoffSpeed(8)
# control = BurnAndCoast_Velocity(min_vel=1.7546696194154812, max_vel=10.106020212266744)
control = ConstantPower()


v = Vehicle(powertrain, transmission)

# OptimiseTransmissionRatio(15, v, track, control.demand)

total_time = 45 * 60
laps = 10
desired_time = total_time / laps
print(desired_time)

# optimisation_result = OptimiseTransmissionRatio_SpecificTime(6, desired_time, v, track, control.demand)
# ratio = optimisation_result['x'][0]
#
# transmission.ratio = ratio

#
# print(optimisation_result[0])
# optimisation_result = minimize(TransmissionRatio_MinMax_TimeCost, [27.851898193359375, 2, 8], args=(v, track, control, 600, desired_time), method='Nelder-Mead')

vehicle_results = v.simulate(track, 0, [0, 0], control_function=control.demand, time_step=0.01, time_limit=600)


Animate(track, vehicle_results)


