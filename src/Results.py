import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D




def Animate(track, vehicle_results):
    track = track
    vehicle_results = vehicle_results

    # Break out simulation results
    t, y, s, fuel_power = vehicle_results

    print(np.trapz(fuel_power, t))

    # Velocity - Time plot
    linear_velocity = [None] * len(t)

    # 3D position
    x_pos = [None] * len(t)
    y_pos = [None] * len(t)
    z_pos = [None] * len(t)

    # Iterate over timesteps
    for index in range(len(t)):
        # Get segment length
        seg_length = track.track[s[index]]['length']

        linear_velocity[index] = y[index][1] * seg_length

        coordinates = track.position(s[index], y[index][0])
        x_pos[index] = coordinates[0]
        y_pos[index] = coordinates[1]
        z_pos[index] = coordinates[2]


    # Initialise figure
    fig2D = plt.figure()
    vel_ax = fig2D.add_subplot(311)
    lambda_ax = fig2D.add_subplot(312)
    fuel_ax = fig2D.add_subplot(313)

    lambda_ax.plot(t, [x[0] for x in y])
    lambda_ax.plot(t, s)
    vel_ax.plot(t, linear_velocity)
    fuel_ax.plot(t, fuel_power)

    plt.show()


    # 3D
    fig3D = plt.figure()
    track_ax = fig3D.add_subplot(111, projection='3d')

    # Draw track
    track_x, track_y, track_z = track.draw_coordinates()
    track_plot = track_ax.plot(track_x, track_y, track_z)

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
    # vehicle_rep = track_ax.scatter(x_pos, y_pos, z_pos, zdir='z', s=0.1, c=[[1, 0, 0]], depthshade=True)
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

