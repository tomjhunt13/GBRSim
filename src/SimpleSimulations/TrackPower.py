from src.Vehicle.Vehicle import *
from src.Track.ImportTrack import *
from src.Powertrain import *
from src.Control import *

track = import_year('2018')

# track




# Andy butler or  Luke else alpha = 1

transmission = Transmission(1, 1)
powertrain = MaxonRE65()
control = ConstantPower()


vehicle = Vehicle(powertrain, transmission)
vehicle.track = track

# V_mph = 14.9
V = 6.675

Fw = []
Fa = []
Fc = []
Frr = []
F = []
length = 0

for segment_index, segment in enumerate(track.segments):
    print('Segment: ' + str(segment_index))
    length += segment.length
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

        # total = weight + aerodynamic_drag + cornering_drag + rolling_resistance
        total = weight + aerodynamic_drag + rolling_resistance
        print('Total: ' + str(total))

        F.append(total)

max_force = max(F)
print(max_force)


max_power = max_force * V
print(max_power)

t = length / V
print(11 * ((max_power * t * 2) / (3.6 * 1e6)))

# vehicle_results = v.simulate(track, 0, [0.5, 0], control_function=control.demand, time_step=0.01, time_limit=210)