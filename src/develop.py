import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track import *
from src.Powertrain import *
from src.Vehicle import *
from src.RK4 import *
from src.Results import *

# Make Track
r = 200
theta = np.pi / 32

track_list = [
    {'type': 'line', 'start': [0, 0, 0],
     'end': [100, 0, 5]},

    {'type': 'arc', 'start': [100, 0, 5],
     'end': [150, 50, 5],
     'centre': [100, 50, 5]},

    {'type': 'arc', 'start': [150, 50, 5],
     'end': [100, 100, 5],
     'centre': [100, 50, 5]},

    {'type': 'line', 'start': [100, 100, 5],
     'end': [0, 100, 0]},

    {'type': 'arc', 'start': [0, 100, 0], 'end': [-50, 50, 0], 'centre': [0, 50, 0]},
    {'type': 'arc', 'start': [-50, 50, 0], 'end': [0, 0, 0], 'centre': [0, 50, 0]}

]

track = Track(track_list)


results = Results(track)

results.Animate()
# track_x, track_y, track_z = track.draw_coordinates()
#
#
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# ax.plot(track_x, track_y, track_z)
#
# ax.set_aspect('equal')
#
#
#
# ax.scatter(0, 0, 0, zdir='z', s=20, c=[[1,0,0]], depthshade=True)
#
# plt.show()
#
# powertrain = Powertrain()
#
# v = Vehicle(1, powertrain, track)
# s = RK4()
#
# t, y = s.solve(v.equation_of_motion, [0.5, 0], time_step=0.05, time_range=[0, 100])
#
# print(3)
# x_0 = [x[0] for x in y]
# # x_1 = [x[1] for x in v.state_history]
# #
# import matplotlib.pyplot as plt
#
# plt.plot(t, x_0)
# # plt.plot(v.t, x_1)
# #
# plt.show()
# a = 0
#
# # Axes3D.scatter(xs, ys, zs=0, zdir='z', s=20, c=None, depthshade=True, *args, **kwargs)¶
