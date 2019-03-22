import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track.ImportTrack import *
from src.Track.Line import *


t = import_year('2018')

t = Track([Line([[0, 0, 0], [1900, 0, 150]])])



file_path = '/Users/tom/Documents/University/Y3_S2/GBRSim/src/Track/CoordinateCSVs/Track_2019.csv'
file_path = '/Users/tom/Documents/University/Y3_S2/GBRSim/src/Track/CoordinateCSVs/Track_2018.csv'

# t = import_track(file_path, start_row=1, end_row=10)

fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')

for seg in t.segments:
    print('Horiz: ' + str(seg.horizontal_radius_of_curvature(0.5)))
    print(seg.radius_of_curvature(0.5))
    x, y, z = seg.draw_coordinates(num_segments=20)
    track_ax.plot(x, y, z)

# x, y, z = t.draw_coordinates(num_segments=5)
#
#
#
# ljhg = t.total_length()
# print(ljhg)
#
# fig3D = plt.figure()
# track_ax = fig3D.add_subplot(111, projection='3d')
# track_plot = track_ax.plot(x, y, z)

# scaling = np.array([getattr(track_ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
# track_ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)
# track_ax.set_zlim([-0.02, 0.02])

plt.show()

