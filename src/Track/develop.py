import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track.BezierSpline import *





r = 2
lkhjb = (4.0 / 3.0) * (-1 + np.sqrt(2))
circle = [[0, r, 0], [r, r * lkhjb, 0], [r * lkhjb, r, 0], [r, 0, 0]]

points = [[0, 0, 0], [1, 2, 0], [2, 4, -4], [3, 6, 0]]

# knots = [[0, 0, 0], [3, 0, 0]]

splines = fit_bezier(points)

fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')

for spline in splines:

    colours = [[1, 0, 0], [0, 0, 1]]
    points = [spline.knots, spline.control_points]
    for i in range(2):
        for j in range(2):
            track_ax.scatter(points[i][j][0], points[i][j][1], points[i][j][2], zdir='z', s=5, c=[colours[i]],
                             depthshade=True)

    x, y, z = spline.draw_coordinates()

    track_ax.plot(x, y, z)
    # track_ax.plot(x, y, [0 for i in z])

plt.show()




#
#
#
# knots = [[0, 0, 0], [1, 0, 1]]
#
# # knots = [[0, 0, 0], [3, 0, 0]]
#
# control_points = [[0, 1, 0], [1, 1, 0]]
#
#
# r = 2
#
# lkhjb = (4.0 / 3.0) * (-1 + np.sqrt(2))
#
# circle = [
#     [[0, r, 0], [r, 0, 0]],
#     [[r *  lkhjb, r, 0],  [r, r* lkhjb, 0]]
# ]
#
# # circle = [
# #     [[r, 0, 0], [0, r, 0]],
# #     [[r, r* lkhjb, 0], [r *  lkhjb, r, 0]]
# # ]
#
# knots = circle[0]
# control_points = circle[1]
#
# knots = [[0, 0, 0], [3, 0, 0]]
# control_points = [[1, 0, 0], [2, 0, 0]]
#
#
#
# b = CubicBezier(knots, control_points)
#
# x, y, z = b.draw_coordinates()
#
#
# print(b.radius_of_curvature(0))
# print(b.radius_of_curvature(0.5))
# print(b.radius_of_curvature(1))
#
#
#
# fig3D = plt.figure()
# track_ax = fig3D.add_subplot(111, projection='3d')
#
#
# colours  = [[1, 0, 0], [0, 0, 1]]
# points = [knots, control_points]
# for i in range(2):
#
#     for j in range(2):
#
#
#         track_ax.scatter(points[i][j][0], points[i][j][1], points[i][j][2], zdir='z', s=5, c=[colours[i]], depthshade=True)
#
# track_ax.plot(x, y, z)
# track_ax.plot(x, y, [0 for i in z])
#
# plt.show()