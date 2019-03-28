from scipy.optimize import minimize

from src.Control import *
from src.Powertrain import *
from src.Vehicle.Vehicle import *
from src.Results import *



def Simulation(vehicle, track, control_function, time_limit):

    vehicle_results = vehicle.simulate(track, 0, [0, 0], control_function=control_function, time_step=0.05, time_limit=time_limit)

    t, y, s, lambda_param, info_dict = vehicle_results

    fuel_power = [d['fuel_power'] for d in info_dict]
    # V = [d['V'] for d in info_dict]
    # P = [d['P'] for d in info_dict]
    # Frr = [d['Frr'] for d in info_dict]
    # Fw = [d['Fw'] for d in info_dict]
    # Fa = [d['Fa'] for d in info_dict]
    # Fc = [d['Fc'] for d in info_dict]

    energy = np.trapz(fuel_power, t)
    time = t[-1]

    return energy, time


def TransmissionRatioSim(ratio, *args):

    vehicle = args[0]
    track = args[1]
    control_function = args[2]
    time_limit = args[3]

    # Motor properties
    transmission = vehicle.transmission
    transmission.ratio = ratio[0]


    print('Ratio: ' + str(ratio[0]))

    energy, time = Simulation(vehicle, track, control_function, time_limit)

    print('Time: ' + str(time))

    return time

def TransmissionRatioSim_TimeCost(ratio, *args):

    desired_time = args[4]

    result = TransmissionRatioSim(ratio, args[0], args[1], args[2], args[3])

    error = desired_time - result

    return np.sqrt(error * error)


def OptimiseTransmissionRatio_Speed(initial_ratio, vehicle, track, control_function, time_limit=500):

    x0 = [initial_ratio]
    return minimize(TransmissionRatioSim, x0, args=(vehicle, track, control_function, time_limit), method='Nelder-Mead')

def OptimiseTransmissionRatio_SpecificTime(initial_ratio, desired_time, vehicle, track, control_function, time_limit=500):

    x0 = [initial_ratio]
    return minimize(TransmissionRatioSim_TimeCost, x0, args=(vehicle, track, control_function, time_limit, desired_time), method='Nelder-Mead')






def TransmissionRatio_MinMaxSpeed_Time(x, *args):
    vehicle = args[0]
    track = args[1]
    control_instance = args[2]
    time_limit = args[3]

    # Motor properties
    transmission = vehicle.transmission
    transmission.ratio = x[0]

    control_instance.min_vel = x[1]
    control_instance.max_vel = x[2]

    print('Ratio: ' + str(x[0]) + ', Min Velocity: ' + str(x[1]) + ', Max Velocity: ' + str(x[2]))

    energy, time = Simulation(vehicle, track, control_instance.demand, time_limit)

    print('Time: ' + str(time))

    return time

def TransmissionRatio_MinMax_TimeCost(x, *args):
    desired_time = args[4]

    result = TransmissionRatio_MinMaxSpeed_Time(x, args[0], args[1], args[2], args[3])

    error = desired_time - result

    return np.sqrt(error * error)

# def OptimiseTransmissionRatio_MinMaxSpeed(x0, vehicle, track, control_function, time_limit=500):
#
#     return minimize(TransmissionRatioSim, x0, args=(vehicle, track, control_function, time_limit), method='Nelder-Mead')





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
    powertrain.transmission_ratio = OptimiseTransmissionRatio_Speed(cutoff_speed[0], power, number_of_motors, powertrain, track, vehicle)['x'][0]



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

