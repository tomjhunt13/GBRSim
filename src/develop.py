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


motor = BrushedDCMotor.MaxonRE65(verbose=True)
# motor =  BrushedDCMotor.Moog_C42_L90_10()
powertrain = Powertrain.DirectTransmission(motor, 14.698399052931432, transmission_efficiency=0.93)
# powertrain = Powertrain.DirectTransmission(motor, 8.140108198711413, transmission_efficiency=0.93)
# free_wheel_properties = {'motor_shaft_inertia': 1340 * (0.001) * (0.01 * 0.01),  'motor_shaft_viscous': 0, 'motor_shaft_constant': 0}
# powertrain = Powertrain.FreeWheel(motor, 14.698399052931432, free_wheel_properties, transmission_efficiency=0.93)


# control = CutoffSpeed(8)
# control = Control.BurnAndCoast_Velocity(min_vel=5.418174027298977, max_vel=15.984388994483382)
control = Control.ConstantPower()


v = Vehicle.Vehicle(powertrain)

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

vehicle_results = v.simulate(track, 0, [0, 0], control_function=control.demand, time_step=0.02, time_limit=500, lap_limit=1)

t_end = time.time()

t_total = t_end -  t_start

print(t_total)
#
#
# # write_csv(vehicle_results)
Results.process_results(track, vehicle_results)


