from src.Vehicle.Vehicle import *
from src.Track.ImportTrack import *
from src.Powertrain import *
from src.Control import *

track = import_year('2019')



# Motor

# Andy butler or  Luke else alpha = 1
motor_properties = {'torque_constant': 0.0631, 'motor_efficiency': 0.8, 'motor_speed_constant': 138, 'no_load_speed': 2690}
battery_properties = {'max_discharge_power': 300, 'discharge_voltage': 48, 'battery_efficiency': 0.9}
transmission_properties = {'transmission_ratio': 50, 'transmission_efficiency': 0.95}

powertrain = BrushedMotor(motor_properties, battery_properties, transmission_properties, number_of_motors=1)
control = ConstantPower()

# Vehicle attributes
mass = 150      # Total vehicle mass (kg)
Crr = 0.005     # Coefficient of rolling resistance
Cd = 0.27       # Coefficient of drag
A = 1.3         # Frontal area (m^2)
Cs = 0.3        # Cornering stiffness
PoweredWheelRadius = 0.2       # Radius of tyre for powered wheel

vehicle_parameters = {'Mass': mass, 'Crr': Crr, 'Cd': Cd, 'A': A, 'Cs': Cs, 'PoweredWheelRadius': PoweredWheelRadius}

vehicle = Vehicle(vehicle_parameters, powertrain)
vehicle.track = track

V = 8

for segment_index, segment in enumerate(track.segments):

    print(segment_index)

    lambda_param = 0.5
    theta = segment.gradient(lambda_param)


    print(vehicle.resistive_forces(theta, V, segment_index, lambda_param))

# vehicle_results = v.simulate(track, 0, [0.5, 0], control_function=control.demand, time_step=0.01, time_limit=210)