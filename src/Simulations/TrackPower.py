from src.Vehicle.Vehicle import *
from src.Track.ImportTrack import *
from src.Powertrain import *
from src.Control import *

track = import_year('2019')




# Andy butler or  Luke else alpha = 1
motor_properties = {'torque_constant': 0.0631, 'motor_efficiency': 0.8, 'motor_speed_constant': 138, 'no_load_speed': 2690}
battery_properties = {'max_discharge_power': 300, 'discharge_voltage': 48, 'battery_efficiency': 0.9}
transmission_properties = {'transmission_ratio': 50, 'transmission_efficiency': 0.95}

powertrain = BrushedMotor(motor_properties, battery_properties, transmission_properties, number_of_motors=1)
control = ConstantPower()


vehicle = Vehicle(powertrain)
vehicle.track = track


V = 5

Fw = []
Fa = []
Fc = []
Frr = []
F = []


for segment_index, segment in enumerate(track.segments):
    print('Segment: ' + str(segment_index))
    for t in range(10):
        lambda_param = 0.1 * t
        print('t: ' +  str(lambda_param))

        theta = segment.gradient(lambda_param)

        weight, aerodynamic_drag, cornering_drag, rolling_resistance = vehicle.resistive_forces(theta, V, segment_index, lambda_param)
        print('Weight: ' + str(weight) + ', Aero: ' + str(aerodynamic_drag) + ', Cornering: ' + str(cornering_drag) + ', Rolling Resistance: ' + str(rolling_resistance))

        Fw.append(weight)
        Fa.append(aerodynamic_drag)
        Fc.append(cornering_drag)
        Frr.append(rolling_resistance)

        total = weight + aerodynamic_drag + cornering_drag + rolling_resistance
        print('Total: ' + str(total))

        F.append(total)

max_force = max(F)
print(max_force)


max_power = max_force * V
print(max_power)

# vehicle_results = v.simulate(track, 0, [0.5, 0], control_function=control.demand, time_step=0.01, time_limit=210)