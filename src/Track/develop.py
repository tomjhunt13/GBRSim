import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track.ImportTrack import *
from src.Track.Line import *


# t = import_year('2019')

t = Track([Line([[0, 0, 0], [1900, 0, 150]])])



x, y, z = t.draw_coordinates(num_segments=5)



ljhg = t.total_length()
print(ljhg)

fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')
track_plot = track_ax.plot(x, y, z)

# scaling = np.array([getattr(track_ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
# track_ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)
# track_ax.set_zlim([-0.02, 0.02])

plt.show()

