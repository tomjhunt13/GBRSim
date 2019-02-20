from scipy.optimize import minimize

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

track = Track(track_list)

motor_properties = {'torque_constant': 0.054, 'motor_efficiency': 0.9, 'motor_speed_constant': 178, 'motor_coil_resistance': 2.4}
battery_properties = {'max_discharge_power': 300, 'discharge_voltage': 24, 'battery_efficiency': 0.9}
transmission_properties = {'transmission_ratio': 125, 'transmission_efficiency': 0.95}


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


def TransmissionRatioSim(ratio, *args):

    power = args[0]
    number_of_motors = args[1]
    powertrain = args[2]
    track = args[3]
    vehicle = args[4]

    # Motor properties
    powertrain.number_of_motors = number_of_motors
    powertrain.max_discharge_power = power
    powertrain.transmission_ratio = ratio[0]

    print('Ratio: ' + str(ratio[0]))

    vehicle_results = vehicle.simulate(track, 0, [0, 0], time_step=0.01, time_limit=1000)
    time = vehicle_results[0][-1]

    print('Time: ' + str(time))

    return time



# Optimise
x0 = [1000]

res = minimize(TransmissionRatioSim, x0, args=(300, 1, powertrain, track, v), method='Nelder-Mead')
