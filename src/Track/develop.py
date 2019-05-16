import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track import BezierSpline, Line, Circle
from src.Track import ImportTrack


# Load track
track = ImportTrack.import_year('2018')

# Create figure
fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d', proj_type='ortho')

# Draw each curve individually
for segment in track.segments[:]:

    x, y, z = segment.draw_coordinates()

    track_ax.plot(x, y, z, linewidth=3)



track_ax.view_init(elev=-90., azim=-90.)
track_ax.set_aspect('equal')
track_ax.grid(False)
track_ax.set_xticks([])
track_ax.set_yticks([])
track_ax.set_zticks([])

plt.subplots_adjust(left=0.01, right=0.99, bottom=0.01, top=0.99, wspace=0.01, hspace=0.01)

plt.show()






"""
# ------- Vertical Circle ------- #
vertical_circle = Circle.VerticalCircle(10)

x, y, z = vertical_circle.draw_coordinates()


fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')
track_plot = track_ax.plot(x, y, z)

plt.show()

# ------------ Bezier example ------------------- #
knots = [[0, 0, 0], [12, 0, 0]]
control_points = [[4, 0, 0], [8, 0, 0]]

b = BezierSpline.CubicBezier(knots, control_points)


x, y, z = b.draw_coordinates()


fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')
track_plot = track_ax.plot(x, y, z)

plt.show()
"""

