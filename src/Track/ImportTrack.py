import csv

from src import src_location
from src.Track.BezierSpline import *
from src.Track.Track import *


def read_coordinate_file(file_path, start_row=1, end_row=None):
    """
    Read a three columned CSV file and return a list of coordinates
    :param file_path:
    :param start_row:
    :param end_row:
    :return:
    """

    # Read file
    with open(file_path, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')

        x = []
        y = []
        z = []

        for row in spamreader:
            x.append(row[0])
            y.append(row[1])
            z.append(row[2])

    if end_row is None:
        end_row = len(x) - 1

    x = [float(l) for l in x[start_row:end_row]]
    y = [float(l) for l in y[start_row:end_row]]
    z = [float(l) for l in z[start_row:end_row]]

    return [[x[i], y[i], z[i]] for i in range(len(x))]


def import_track(file_path, start_row=1, end_row=None):
    """

    :return:
    """
    return Track(fit_cubic_bezier(read_coordinate_file(file_path, start_row=start_row, end_row=end_row)))


def import_year(year):
    """

    :param year:
    :return:
    """

    if year == '2018':
        return import_track(src_location + '/Track/CoordinateCSVs/Track_2018.csv')

    raise Exception('Could not find track for year ' + str(year))


if __name__ == "__main__":

    t = import_year('2018')

    x, y, z = t.draw_coordinates(num_segments=5)

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    fig3D = plt.figure()
    track_ax = fig3D.add_subplot(111, projection='3d')
    track_plot = track_ax.plot(x, y, z)

    plt.show()

