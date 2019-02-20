from src.Track import *
from src.Powertrain import *
from src.Vehicle import *
from src.Results import *

# Make Track
h = 5

track_list = [
    {'type': 'line', 'start': [0, 0, 0],
     'end': [100, 0, h]},

    {'type': 'arc', 'start': [100, 0, h],
     'end': [150, 50, h],
     'centre': [100, 50, h]},

    {'type': 'arc', 'start': [150, 50, h],
     'end': [100, 100, h],
     'centre': [100, 50, h]},

    {'type': 'line', 'start': [100, 100, h],
     'end': [0, 100, 0]},

    {'type': 'arc', 'start': [0, 100, 0], 'end': [-50, 50, 0], 'centre': [0, 50, 0]},
    {'type': 'arc', 'start': [-50, 50, 0], 'end': [0, 0, 0], 'centre': [0, 50, 0]}

]

bathwick = [
    {'type': 'line', 'start': [0, 0, 0], 'end': [1900, 0, 150]}
]

track = Track(bathwick)

# Motor
motor_properties = {'torque_constant': 0.054, 'motor_efficiency': 0.9, 'motor_speed_constant': 178, 'motor_coil_resistance': 2.4}
battery_properties = {'max_discharge_power': 300, 'discharge_voltage': 24, 'battery_efficiency': 0.9}
transmission_properties = {'transmission_ratio': 5000, 'transmission_efficiency': 0.95}

powertrain = BrushedMotor(motor_properties, battery_properties, transmission_properties, number_of_motors=1)

# Vehicle attributes
mass = 150      # Total vehicle mass (kg)
Crr = 0.005     # Coefficient of rolling resistance
Cd = 0.27       # Coefficient of drag
A = 1.3         # Frontal area (m^2)
Cs = 0.3        # Cornering stiffness
PoweredWheelRadius = 0.2       # Radius of tyre for powered wheel

vehicle_parameters = {'Mass': mass, 'Crr': Crr, 'Cd': Cd, 'A': A, 'Cs': Cs, 'PoweredWheelRadius': PoweredWheelRadius}

v = Vehicle(vehicle_parameters, powertrain)
vehicle_results = v.simulate(track, 0, [0, 0], time_step=0.01, time_limit=500)


Animate(track, vehicle_results)