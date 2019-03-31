import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D




def Animate(track, vehicle_results):

    # Break out simulation results
    t, y, info_dict = vehicle_results

    fuel_power = [d['Fuel Power'] for d in info_dict]
    V = [d['V'] for d in info_dict]
    P = [d['P'] for d in info_dict]
    Frr = [d['Frr'] for d in info_dict]
    Fw = [d['Fw'] for d in info_dict]
    Fa = [d['Fa'] for d in info_dict]
    Fc = [d['Fc'] for d in info_dict]
    s = [d['segment'] for d in info_dict]
    lambda_param = [d['lambda_param'] for d in info_dict]


    print(np.trapz(fuel_power, t))
    print(t[-1])

    # Scale Velocity to mph
    V = [i * 2.237 for i in V]

    # 3D position
    # x_pos = [None] * len(t)
    # y_pos = [None] * len(t)
    # z_pos = [None] * len(t)
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

    # #
    # #     # Get 3D coordinates
    # #     coordinates = track.position(s[index], y[index][0])
    # #     # vehicle_rep.set_xdata(coordinates[0])
    # #
    # #     # plt.draw()
    # #     plt.pause(0.01)
    #
    #
    # # # Initialise marker
    # # coordinates = track.position(s[0], y[0][0])

    # # #
    # # # plt.ion()

    # #
    # # # Iterate over timesteps
    # # for index in range(len(t)):
    # #
    # #     # time.sleep(0.01)
    # #
    # #     # Get 3D coordinates
    # #     coordinates = track.position(s[index], y[index][0])
    # #     # vehicle_rep.set_xdata(coordinates[0])
    # #
    # #     # plt.draw()
    # #     plt.pause(0.01)

