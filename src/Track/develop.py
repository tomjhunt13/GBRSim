import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from src.Track.ImportTrack import *

t = import_year('2018')
x, y, z = t.draw_coordinates(num_segments=5)

fig3D = plt.figure()
track_ax = fig3D.add_subplot(111, projection='3d')
track_plot = track_ax.plot(x, y, z)
plt.show()

