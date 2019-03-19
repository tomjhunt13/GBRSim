import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track.BezierSpline import *

knots = [[0, 0, 0], [1, 0, 1]]

# knots = [[0, 0, 0], [3, 0, 0]]

control_points = [[1, 0, 0], [2, 0, 0]]


b = CubicBezier(knots, control_points)

x, y, z = b.draw_coordinates()

print(b.direction(0))
print(b.direction(1))

fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')


colours  = [[1, 0, 0], [0, 0, 1]]
points = [knots, control_points]
for i in range(2):

    for j in range(2):


        track_ax.scatter(points[i][j][0], points[i][j][1], points[i][j][2], zdir='z', s=5, c=[colours[i]], depthshade=True)

track_ax.plot(x, y, z)

plt.show()