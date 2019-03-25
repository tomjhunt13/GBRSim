from scipy.optimize import minimize

from src.Control import *
from src.Powertrain import *
from src.Vehicle.Vehicle import *
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

# motor_properties = {'torque_constant': 0.054, 'motor_efficiency': 0.8, 'motor_speed_constant': 178, 'no_load_speed': 4090}
# battery_properties = {'max_discharge_power': 300, 'discharge_voltage': 24, 'battery_efficiency': 0.9}
# transmission_properties = {'transmission_ratio': 125, 'transmission_efficiency': 0.95}
#
#
# powertrain = BrushedMotor(motor_properties, battery_properties, transmission_properties, number_of_motors=1)
#
# # Vehicle attributes
# mass = 150      # Total vehicle mass (kg)
# Crr = 0.005     # Coefficient of rolling resistance
# Cd = 0.27       # Coefficient of drag
# A = 1.3         # Frontal area (m^2)
# Cs = 0.3        # Cornering stiffness
# PoweredWheelRadius = 0.2       # Radius of tyre for powered wheel
#
# vehicle_parameters = {'Mass': mass, 'Crr': Crr, 'Cd': Cd, 'A': A, 'Cs': Cs, 'PoweredWheelRadius': PoweredWheelRadius}
#
# v = Vehicle(vehicle_parameters, powertrain)


"""
Optimisation strategy:

Outer optimisation:
    - Minimise energy consumption. Free variable: motor cutoff speed
    
    Inner optimisation:
        - Minimise time taken to complete track for given cutoff speed. Free variable: transmission ratio
"""


def TransmissionRatioSim(ratio, *args):

    power = args[0]
    number_of_motors = args[1]
    powertrain = args[2]
    track = args[3]
    vehicle = args[4]
    cutoff_speed = args[5]

    # Motor properties
    powertrain.number_of_motors = number_of_motors
    powertrain.max_discharge_power = power
    powertrain.transmission_ratio = ratio[0]

    print('Ratio: ' + str(ratio[0]))

    vehicle_results = vehicle.simulate(track, 0, [0, 0], time_step=0.01, time_limit=1000, power_cutoff_speed=cutoff_speed)
    time = vehicle_results[0][-1]

    print('Time: ' + str(time))

    return time


def OptimiseTransmissionRatio(vehicle_cutoff_speed, power, number_of_motors, powertrain, track, vehicle):

    print('Optimising transmission ratio for cutoff speed: ' + str(vehicle_cutoff_speed))
    x0 = [1000]
    return minimize(TransmissionRatioSim, x0, args=(power, number_of_motors, powertrain, track, vehicle, vehicle_cutoff_speed), method='Nelder-Mead')

def CutoffSpeedSim(cutoff_speed, *args):
    print('Cutoff speed: ' + str(cutoff_speed[0]))

    power = args[0]
    number_of_motors = args[1]
    powertrain = args[2]
    track = args[3]
    vehicle = args[4]

    # Motor properties
    powertrain.number_of_motors = number_of_motors
    powertrain.max_discharge_power = power
    powertrain.transmission_ratio = OptimiseTransmissionRatio(cutoff_speed[0], power, number_of_motors, powertrain, track, vehicle)['x'][0]



    vehicle_results = vehicle.simulate(track, 0, [0, 0], time_step=0.01, time_limit=1000, power_cutoff_speed=cutoff_speed[0])

    # Break out simulation results
    t, y, s, fuel_power, P, Frr, Fw, Fa, Fd = vehicle_results
    energy_use = np.trapz(fuel_power, t)

    print('Energy: ' + str(energy_use))

    return energy_use



"""
Optimisation strategy 2
"""

def BurnAndCoast_TransmissionRatio(input_vector, *args):
    """

    :param input_vector: numpy ndarray of form - [minimum_velocity, maximum_velocity, transmission_ratio]
    :param args:
    :return:
    """

    # Unpack input_vector
    min_vel = input_vector[0]
    max_vel  = input_vector[1]
    transmission_ratio = input_vector[2]
    print('Min Velocity: ' + str(min_vel) + ', Max Velocity: ' + str(max_vel) + ', Transmission Ratio: ' + str(transmission_ratio))

    # Unpack args
    controller = args[0]
    powertrain = args[1]
    vehicle = args[2]
    track = args[3]
    max_time = args[4]

    # Tweak values
    controller.min_vel = min_vel
    controller.max_vel = max_vel
    powertrain.transmission_ratio = transmission_ratio

    # Run simulation
    t, y, s, fuel_power, P, Frr, Fw, Fa, Fd = vehicle.simulate(track, 0, [0, 0], control_function=controller.demand,
                                                               time_step=0.01, time_limit=max_time)

    energy_use = np.trapz(fuel_power, t)
    print('Energy Use: ' + str(energy_use))

    if np.isclose(t[-1], max_time):
        print('Failed course')
        energy_use = 1000000000

    return energy_use

def BurnAndCoast_TransmissionRatio_OperatingVoltage(input_vector, *args):
    """

    :param input_vector: numpy ndarray of form - [minimum_velocity, maximum_velocity, transmission_ratio]
    :param args:
    :return:
    """

    # Unpack input_vector
    # min_vel = input_vector[0]
    # max_vel  = input_vector[1]
    transmission_ratio = input_vector[0]

    print('Transmission Ratio: '  + str(transmission_ratio))

    # operating_voltage = input_vector[1]
    # print('Transmission Ratio: '
    #       + str(transmission_ratio) + ', Operating Voltage: ' + str(operating_voltage))


    # Unpack args
    controller = args[0]
    powertrain = args[1]
    vehicle = args[2]
    track = args[3]
    max_time = args[4]

    # Tweak values
    powertrain.transmission_ratio = transmission_ratio
    # powertrain.discharge_voltage = operating_voltage

    # Run simulation
    t, y, s, fuel_power, P, Frr, Fw, Fa, Fd = vehicle.simulate(track, 0, [0.5, 0], control_function=controller.demand,
                                                               time_step=0.01, time_limit=max_time)

    print('Lambda ' + str(y[-1][0]))

    # if np.isclose(t[-1], max_time):
    #     print('Failed course')
    #     energy_use = 1000000000

    return 1 - y[-1][0]


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
motor_properties = {'torque_constant': 0.123, 'motor_efficiency': 0.8, 'motor_speed_constant': 178, 'no_load_speed': 4090}
battery_properties = {'max_discharge_power': 300, 'discharge_voltage': 48, 'battery_efficiency': 0.9}
transmission_properties = {'transmission_ratio': 5000, 'transmission_efficiency': 0.95}

powertrain = BrushedMotor(motor_properties, battery_properties, transmission_properties, number_of_motors=1)
# control = BurnAndCoast_Velocity(min_vel=3, max_vel=7.5)
control = ConstantPower()

# Vehicle attributes
mass = 150      # Total vehicle mass (kg)
Crr = 0.005     # Coefficient of rolling resistance
Cd = 0.27       # Coefficient of drag
A = 1.3         # Frontal area (m^2)
Cs = 0.3        # Cornering stiffness
PoweredWheelRadius = 0.2       # Radius of tyre for powered wheel

vehicle_parameters = {'Mass': mass, 'Crr': Crr, 'Cd': Cd, 'A': A, 'Cs': Cs, 'PoweredWheelRadius': PoweredWheelRadius}

v = Vehicle(vehicle_parameters, powertrain)

# Optimise
# x0 = [2, 7.5, 2000]
x0 = [50]

res = minimize(BurnAndCoast_TransmissionRatio_OperatingVoltage, x0, args=(control, powertrain, v, track, 50), method='Nelder-Mead')

print(res)
