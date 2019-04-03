from src.Track import ImportTrack
from src.Track import Track
from src.Track import Line

from src.Optimisation import Optimisation
from src import Control
from src.Vehicle import Vehicle
from src.Powertrain import BrushedDCMotor
from src.Results import Results
from src.Powertrain import Powertrain

track = ImportTrack.import_year('2018')
# l1 = Line.Line([[0, 0, 0], [100, 0, 0]])
# l2 = Line.Line([[100, 0, 0], [100, 100, 0]])
# track = Track.Track([l1, l2])


motor = BrushedDCMotor.MaxonRE65(verbose=False)
# motor =  BrushedDCMotor.Moog_C42_L90_10()
powertrain = Powertrain.DirectTransmission(motor, 5.25, transmission_efficiency=0.93)
# free_wheel_properties = {'motor_shaft_inertia': 1340 * (0.001) * (0.01 * 0.01),  'motor_shaft_viscous': 0, 'motor_shaft_constant': 0}
# powertrain = FreeWheel(motor, 15, free_wheel_properties, transmission_efficiency=0.93)


# control = CutoffSpeed(8)
# control = BurnAndCoast_Velocity(min_vel=1, max_vel=4)
control = Control.ConstantPower()


v = Vehicle.Vehicle(powertrain)

# OptimiseTransmissionRatio(15, v, track, control.demand)

# total_time = 45 * 60
total_time = 39 * 60
laps = 11
desired_time = total_time / laps
print(desired_time)
#
optimisation_result = Optimisation.OptimiseTransmissionRatio_SpecificTime(5, desired_time, v, track, control.demand)
# ratio = optimisation_result['x'][0]

# transmission.ratio = ratio

#
# print(optimisation_result[0])
# optimisation_result = minimize(TransmissionRatio_MinMax_TimeCost, [10, 1.7546696194154812, 10.106020212266744], args=(v, track, control, 600, desired_time), method='Nelder-Mead')

# vehicle_results = v.simulate(track, 0, [0, 0], control_function=control.demand, time_step=0.01, time_limit=500, lap_limit=1)
#
#
# # write_csv(vehicle_results)
# Results.process_results(track, vehicle_results)


