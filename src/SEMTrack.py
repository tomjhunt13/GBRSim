import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track import *

track_coordinates_2018 = '/Users/tom/Documents/University/Y3_S2/Shell_Eco-marathon_Europe_-_London_2018_track_(xyz)_SZEnergy_Team_HUN.csv'
track_coordinates_2019 = '/Users/tom/Documents/University/Y3_S2/Track2019.csv'

with open(track_coordinates_2019, newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')

    x = []
    y = []
    z = []

    for row in spamreader:

        x.append(row[0])
        y.append(row[1])
        z.append(row[2])

x = [float(l) for l in x[1:]]
y = [float(l) for l in y[1:]]
z = [float(l) for l in z[1:]]


fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')
track_plot = track_ax.plot(x, y, z)


# Track representation
track_2018_dict = [
    {'type': 'line', 'start': [22, 87.5, 1], 'end': [71, 63, 0.9]},
    {'type': 'line', 'start': [71, 63, 0.9], 'end': [182, 15, 0.25]},
    {'type': 'line', 'start': [182, 15, 0.25], 'end': [275, -8, -0.2]},
]


track_2019_dict = [
    {'type': 'line', 'start': [6.849792, 0.884736, 0], 'end': [0, 0, 0]},
    {'type': 'line', 'start': [0, 0, 0], 'end': [-79.875072, -9.704448, 0]},
    {'type': 'arc', 'start': [-79.875072, -9.704448, 0], 'end': [-126.2482254,	-58.973184, 0], 'centre': [-76.889088, -58.973184, 0]},
    {'type': 'arc', 'start': [-126.2482254,	-58.973184, 0], 'end': [-73.903104, -108.24192, 0], 'centre': [-76.889088, -58.973184, 0]},
    {'type': 'line', 'start': [-73.903104, -108.24192, 0], 'end': [122.425344, -87.340032, 0]},
    {'type': 'line', 'start': [122.425344, -87.340032, 0], 'end': [371.755008, -50.015232, 0]},
    # {'type': 'arc', 'start': [371.755008, -50.015232, 0], 'end': [406.233792, -54.195264, 0], 'centre': [373.56, -179.36, 0]},

    {'type': 'line', 'start': [371.755008, -50.015232, 0], 'end': [406.233792, -54.195264, 0]},






]

track = Track(track_2019_dict)

track_x, track_y, track_z = track.draw_coordinates()
track_plot = track_ax.plot(track_x, track_y, track_z, color=[1, 0, 0])






scaling = np.array([getattr(track_ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
track_ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)


track_ax.set_xlim([350, 425])
track_ax.set_ylim([-100, -25])
# track_ax.set_zlim([-0.02, 0.02])

plt.show()
