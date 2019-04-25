from src.Track import ImportTrack


from src.Integration import RKF45


from src.Optimisation import Optimisation
from src import Control
from src.Vehicle import SEMVehicle
from src.Powertrain import BrushedDCMotor
from src.Results import Results
from src.Powertrain import Powertrain

track = ImportTrack.import_year('2019')
control = Control.ConstantPower()

car = SEMVehicle.SEMVehicle()

model_kwargs = {'track': track, 'control_function': control.demand}


s = RKF45.RKF45()
vehicle_results = s.solve(car, car.equation_of_motion, model_kwargs, [0,0,0], dt=0.0001, t_end=50)
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