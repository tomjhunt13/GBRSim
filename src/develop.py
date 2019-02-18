import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track import *
from src.Powertrain import *
from src.Vehicle import *
from src.Results import *

# Make Track
r = 200
theta = np.pi / 32

track_list = [
    {'type': 'line', 'start': [0, 0, 0],
     'end': [100, 0, 1]},

    {'type': 'arc', 'start': [100, 0, 1],
     'end': [150, 50, 1],
     'centre': [100, 50, 1]},

    {'type': 'arc', 'start': [150, 50, 1],
     'end': [100, 100, 1],
     'centre': [100, 50, 1]},

    {'type': 'line', 'start': [100, 100, 1],
     'end': [0, 100, 0]},

    {'type': 'arc', 'start': [0, 100, 0], 'end': [-50, 50, 0], 'centre': [0, 50, 0]},
    {'type': 'arc', 'start': [-50, 50, 0], 'end': [0, 0, 0], 'centre': [0, 50, 0]}

]

track = Track(track_list)

# Motor
stall_torque = 200
no_load_speed = 5000
powertrain = BrushedMotor(stall_torque, no_load_speed, transmission_ratio=20)

# Vehicle attributes
mass = 150      # Total vehicle mass (kg)
Crr = 0.005     # Coefficient of rolling resistance
Cd = 0.27       # Coefficient of drag
A = 1.3         # Frontal area (m^2)
Cs = 0.3        # Cornering stiffness
PoweredWheelRadius = 0.2       # Radius of tyre for powered wheel

vehicle_parameters = {'Mass': mass, 'Crr': Crr, 'Cd': Cd, 'A': A, 'Cs': Cs, 'PoweredWheelRadius': PoweredWheelRadius}

v = Vehicle(vehicle_parameters, powertrain)
vehicle_results = v.simulate(track, 0, [0, 0], time_step=0.01, time_range=[0, 100])


Animate(track, vehicle_results)