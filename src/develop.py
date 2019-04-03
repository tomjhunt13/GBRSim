from src.Track import ImportTrack
from src.Optimisation.Optimisation import *

from src.Control import *
from src.Vehicle.Vehicle import *
from src.Powertrain.BrushedDCMotor import *
from src.Results.Results import *
from src.Powertrain.Powertrain import *

track = ImportTrack.import_year('2018')



motor = MaxonRE65(verbose=True)
powertrain = DirectTransmission(motor, 5, transmission_efficiency=0.93)
# free_wheel_properties = {'motor_shaft_inertia': 1340 * (0.001) * (0.01 * 0.01),  'motor_shaft_viscous': 0, 'motor_shaft_constant': 0}
# powertrain = FreeWheel(motor, 15, free_wheel_properties, transmission_efficiency=0.93)


# control = CutoffSpeed(8)
# control = BurnAndCoast_Velocity(min_vel=1, max_vel=4)
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

vehicle_results = v.simulate(track, 0, [0, 0], control_function=control.demand, time_step=0.0025, time_limit=400, lap_limit=1)


# write_csv(vehicle_results)
process_results(track, vehicle_results)


