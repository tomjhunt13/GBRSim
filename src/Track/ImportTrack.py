import csv
import matplotlib.pyplot as plt

from src.Track.BezierSpline import *

from mpl_toolkits.mplot3d import Axes3D

from src.Track_original import *

track_coordinates_2018 = '/Users/tom/Documents/University/Y3_S2/Shell_Eco-marathon_Europe_-_London_2018_track_(xyz)_SZEnergy_Team_HUN.csv'
track_coordinates_2019 = '/Users/tom/Documents/University/Y3_S2/Track2019.csv'

with open(track_coordinates_2018, newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')

    x = []
    y = []
    z = []

    for row in spamreader:

        x.append(row[0])
        y.append(row[1])
        z.append(row[2])

x = [float(l) for l in x[10:30]]
y = [float(l) for l in y[10:30]]
z = [float(l) for l in z[10:30]]

points = [[x[i], y[i], z[i]] for i in range(len(x))]

splines = fit_bezier(points)


fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')
# track_plot = track_ax.plot(x, y, z)

for spline in splines:

    # colours = [[1, 0, 0], [0, 0, 1]]
    # points = [spline.knots, spline.control_points]
    # for i in range(2):
    #     for j in range(2):
    #         track_ax.scatter(points[i][j][0], points[i][j][1], points[i][j][2], zdir='z', s=5, c=[colours[i]],
    #                          depthshade=True)

    x, y, z = spline.draw_coordinates(num_segments=5)

    track_ax.plot(x, y, z)
    # track_ax.plot(x, y, [0 for i in z])

plt.show()

