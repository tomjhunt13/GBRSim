import time

from src.Track import ImportTrack
from src.Integration import RKF45, RK4, DP45, Butcher
from src import Control
from src.Vehicle import SEMVehicle
from src.Results import Results

track = ImportTrack.import_year('2018')
control = Control.ConstantPower()

car = SEMVehicle.SEMVehicle()

model_kwargs = {'track': track, 'control_function': control.demand}


# s = RKF45.RKF45()   # Elapsed time: 189.21647214889526
# s = RK4.RK4()       # Elapsed time: 183.0775592327118
# s = DP45.DP45()       # Elapsed time: Huuuge
s = Butcher.RK8()

t_s = time.time()
vehicle_results = s.solve(car, car.equation_of_motion, model_kwargs, [1e-4, 1e-4, 1e-4], dt=0.001, t_end=10, verbose=True)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)







"""
motor = BrushedDCMotor.MaxonRE65(verbose=False)
# motor = BrushedDCMotor.Moog_C42_L90_10()
powertrain = Powertrain.DirectTransmission(motor, 6.923523966325385, transmission_efficiency=0.8)


# control = CutoffSpeed(8)
control = Control.BurnAndCoast_Velocity(min_vel=1.7270681164579773, max_vel=24.09947236113141)
# control = Control.ConstantPower()


v = SEMVehicle.SEMVehicle()

# OptimiseTransmissionRatio(15, v, track, control.demand)

total_time = 45 * 60
# total_time = 39 * 60
# laps = 11
laps = 15
desired_time = total_time / laps
print(desired_time)
#
# optimisation_result = Optimisation.OptimiseTransmissionRatio_EnergyTime([10], desired_time, v, track, control.demand)
# ratio = optimisation_result['x'][0]

# transmission.ratio = ratio

#
# res = Optimisation.OptimiseTransmissionRatio_MinMax_EnergyTime([8, 4, 17], desired_time, v, track, control)
# print(optimisation_result[0])


import time

t_start = time.time()

vehicle_results = v.simulate(track, 0, [0, 0, 15], control_function=control.demand, time_step=0.01, time_limit=500, lap_limit=1)

t_end = time.time()

t_total = t_end - t_start

print(t_total)
#
#
# # write_csv(vehicle_results)
Results.process_results(track, vehicle_results)


"""