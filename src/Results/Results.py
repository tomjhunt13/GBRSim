import csv
import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
from src import src_location



class Results:
    pass


def write_csv(vehicle_results, file_path=None):

    print('Writing CSV')
    # File path
    if not file_path:
        file_path = src_location + '../tmp.csv'

    with open(file_path, 'w', newline='') as csvfile:

        csv_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

        # Header of csv file
        header = [key for key in vehicle_results[0].keys()]
        csv_writer.writerow(header)

        # Body of csv file
        for i in range(len(vehicle_results)):

            row = [None] * len(header)
            for index, key in enumerate(header):
                row[index] = vehicle_results[i][key]

            csv_writer.writerow(row)


def construct_coordinates(track, vehicle_results):


    s = [d['segment'] for d in vehicle_results]
    lambda_param = [d['lambda_param'] for d in vehicle_results]

    # Iterate over timesteps
    for index in range(len(s)):
        if s[index] > len(track.segments) - 1:
            s[index] = len(track.segments) - 1


        coordinates = track.position(s[index], lambda_param[index])
        vehicle_results[index]['x_pos'] = coordinates[0]
        vehicle_results[index]['y_pos'] = coordinates[1]
        vehicle_results[index]['z_pos'] = coordinates[2]


def process_results(track, vehicle_results):


    # Append coordinates to vehicle_results
    construct_coordinates(track, vehicle_results)

    for i in range(len(vehicle_results)):
        vehicle_results[i]['Velocity (mph)'] = vehicle_results[i]['Velocity (m/s)'] * 2.237

        del vehicle_results[i]['y']

    write_csv(vehicle_results)

    # Break out simulation results
    vehicle_results = vehicle_results

    t = [d['t'] for d in vehicle_results]
    fuel_power = [d['Fuel Power'] for d in vehicle_results]
    V = [d['Velocity (m/s)'] for d in vehicle_results]
    P = [d['P'] for d in vehicle_results]
    Frr = [d['Frr'] for d in vehicle_results]
    Fw = [d['Fw'] for d in vehicle_results]
    Fa = [d['Fa'] for d in vehicle_results]
    Fc = [d['Fc'] for d in vehicle_results]
    s = [d['segment'] for d in vehicle_results]
    lambda_param = [d['lambda_param'] for d in vehicle_results]


    print(np.trapz(fuel_power, t))
    print(t[-1])

    # Scale Velocity to mph
    V = [i * 2.237 for i in V]

    # 3D position
    x_pos = []
    y_pos = []
    z_pos = []

    # Iterate over timesteps
    for index in range(len(t)):
        if s[index] < len(track.segments):
            coordinates = track.position(s[index], lambda_param[index])
            x_pos.append(coordinates[0])
            y_pos.append(coordinates[1])
            z_pos.append(coordinates[2])


    # Initialise figure
    fig2D = plt.figure()

    vel_ax = fig2D.add_subplot(421)
    vel_ax.set_title('Velocity')

    lambda_ax = fig2D.add_subplot(422)
    lambda_ax.set_title('Lambda / Segment')

    fuel_ax = fig2D.add_subplot(423)
    fuel_ax.set_title('Power Consumption')

    P_ax = fig2D.add_subplot(424)
    P_ax.set_title('P')

    Frr_ax = fig2D.add_subplot(425)
    Frr_ax.set_title('Frr')

    Fw_ax = fig2D.add_subplot(426)
    Fw_ax.set_title('Fw')

    Fa_ax = fig2D.add_subplot(427)
    Fa_ax.set_title('Fa')

    Fc_ax = fig2D.add_subplot(428)
    Fc_ax.set_title('Fc')

    # lambda_ax.plot(t, [x[0] for x in y])
    lambda_ax.plot(t, s)
    vel_ax.plot(t, V)
    fuel_ax.plot(t, fuel_power)
    P_ax.plot(t, P)
    Frr_ax.plot(t, Frr)
    Fw_ax.plot(t, Fw)
    Fa_ax.plot(t, Fa)
    Fc_ax.plot(t, Fc)


    plt.show()

    # 3D
    fig3D = plt.figure()
    track_ax = fig3D.add_subplot(111, projection='3d')

    # Draw track
    track_x, track_y, track_z = track.draw_coordinates()
    track_plot = track_ax.plot(track_x, track_y, track_z)
    vehicle_rep = track_ax.scatter(x_pos, y_pos, z_pos, zdir='z', s=0.1, c=[[1, 0, 0]], depthshade=True)

    plt.show()