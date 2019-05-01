from src.Model.VehicleModel import *
from src.Track.ImportTrack import *
from src.Model.BrushedDCMotor import *
from src.Model.PowertrainModel import *
from src.Control import *

import matplotlib.pyplot as plt



def force_power(vehicle, V):

    track = vehicle.track

    Fw = []
    Fa = []
    Fc = []
    Frr = []
    F = []
    lambda_list = []

    length = 0

    for segment_index, segment in enumerate(track.segments):

        length += segment.length
        num_seg = 4

        for t in range(num_seg):

            increment = 1 / num_seg

            lambda_param = increment * t
            theta = segment.gradient(lambda_param)

            print(segment_index, lambda_param)

            weight, aerodynamic_drag, cornering_drag, rolling_resistance = vehicle.resistive_forces(theta, V, segment_index, lambda_param)
            # print('Weight: ' + str(weight) + ', Aero: ' + str(aerodynamic_drag) + ', Cornering: ' + str(cornering_drag) + ', Rolling Resistance: ' + str(rolling_resistance))

            Fw.append(weight)
            Fa.append(aerodynamic_drag)
            Fc.append(cornering_drag)
            Frr.append(rolling_resistance)
            lambda_list.append(length)

            total = weight + aerodynamic_drag + cornering_drag + rolling_resistance
            # total = weight + aerodynamic_drag + rolling_resistance
            # print('Total: ' + str(total))

            F.append(total)

    lambda_list = [100 * i / lambda_list[-1] for i in lambda_list]
    power = [V * i for i in F]

    return Fw, Fa, Fc, Frr, F, lambda_list, power, length


def power_plot(vehicle, V_list):

    fig_1, ax_1 = plt.subplots()
    ax_1.set_title('Power')
    ax_1.grid()
    ax_1.legend()

    fig_2, ax_2 = plt.subplots()
    ax_2.set_title('Fw')
    ax_2.grid()
    ax_2.legend()

    fig_3, ax_3 = plt.subplots()
    ax_3.set_title('Fa')
    ax_3.grid()
    ax_3.legend()

    fig_4, ax_4 = plt.subplots()
    ax_4.set_title('Fc')
    ax_4.grid()
    ax_4.legend()

    fig_5, ax_5 = plt.subplots()
    ax_5.set_title('Frr')
    ax_5.grid()
    ax_5.legend()

    fig_6, ax_6 = plt.subplots()
    ax_6.set_title('F')
    ax_6.grid()
    ax_6.legend()


    for V in V_list:

        Fw, Fa, Fc, Frr, F, lambda_list, power, length = force_power(vehicle, V)

        ax_1.plot(lambda_list, power, label='V: ' + str(V))
        ax_2.plot(lambda_list, Fw, label='V: ' + str(V))
        ax_3.plot(lambda_list, Fa, label='V: ' + str(V))
        ax_4.plot(lambda_list, Fc, label='V: ' + str(V))
        ax_5.plot(lambda_list, Frr, label='V: ' + str(V))
        ax_6.plot(lambda_list, F, label='V: ' + str(V))

    ax_1.legend()
    ax_2.legend()
    ax_3.legend()
    ax_4.legend()
    ax_5.legend()
    ax_6.legend()


    plt.show()






motor = Moog_C42_L90_10()
free_wheel_properties = {'motor_shaft_inertia': 1340 * (0.001) * (0.01 * 0.01),  'motor_shaft_viscous': 0, 'motor_shaft_constant':  0 }
powertrain = FreeWheel(motor, 10, free_wheel_properties, transmission_efficiency=0.93)
control = ConstantPower()

vehicle_parameters= {
            'Mass': 170,
            'Crr': 1.5 * 0.001,     # http://www.eshopsem.com/boutique/product.php?id_product=75
            'Cd': 0.3,
            'A': 1.26,
            'PoweredWheelRadius': 0.279,
            'LongitudinalCoG': 0.5,     # Assume even weight distribution
        }
vehicle = Vehicle(powertrain, vehicle_parameters=vehicle_parameters)

track = import_year('2018')
vehicle.track = track

V_list = [2, 4, 6, 8, 10]
power_plot(vehicle, V_list)

#
# V = 6
# # Fw, Fa, Fc, Frr, F, lambda_list, power, length = force_power(vehicle, V)
#
#
# max_force = max(F)
# print(max_force)
#
# mean_force = (np.mean(F))
# print(mean_force)
#
#
# max_power = max(power)
# print(max_power)
#
# t = length / V
# # print('Lap time: ' + str(t))
# print((1 / (0.9 * 0.9 * 0.85)) * 10 * ((max_power * t) / (3.6 * 1e6)))
#
# # vehicle_results = v.simulate(track, 0, [0.5, 0], control_function=control.demand, time_step=0.01, time_limit=210)