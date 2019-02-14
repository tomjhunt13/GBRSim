import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Results:
    def __init__(self, track):
        self.track = track

    def Animate(self):
        track_x, track_y, track_z = self.track.draw_coordinates()

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(track_x, track_y, track_z)

        ax.set_aspect('equal')

        ax.scatter(0, 0, 0, zdir='z', s=20, c=[[1, 0, 0]], depthshade=True)

        plt.show()

