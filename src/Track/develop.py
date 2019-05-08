import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track import BezierSpline

knots = [[0, 0, 0], [12, 0, 0]]
control_points = [[4, 0, 0], [8, 0, 0]]

b = BezierSpline.CubicBezier(knots, control_points)


x, y, z = b.draw_coordinates()


fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')
track_plot = track_ax.plot(x, y, z)

plt.show()

