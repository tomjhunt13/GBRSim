import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track.BezierSpline import *

knots = [[0, 0, 0], [1, 0, 1]]
control_points = [[0, 1, 0], [1, 1, 0]]


b = CubicBezier(knots, control_points)

x, y, z = b.draw_coordinates()

fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')

track_ax.plot(x, y, z)

plt.show()