import csv

from src import src_location
from src.Track.BezierSpline import *
from src.Track.Track import *


def read_coordinate_file(file_path, start_row=1, end_row=None):
    """
    Read a three columned CSV file and construct coordinate list
    :param file_path: Path of csv file
    :param start_row: Row to start reading from
    :param end_row: Row to stop reading at
    :return: Coordinate list
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
    Create a Track instance from a coordinate CSV file
    :return: Track instance
    """
    return Track(fit_cubic_bezier(read_coordinate_file(file_path, start_row=start_row, end_row=end_row)))


def import_year(year):
    """
    Create a Track instance from a certain year coordinate CSV file
    :param year: String - year of track
    :return: Track instance
    """

    if year == '2018':
        return import_track(src_location + '/Track/CoordinateCSVs/Track_2018.csv')

    elif year == '2019':
        return import_track(src_location + '/Track/CoordinateCSVs/Track_2019.csv')

    raise Exception('Could not find track for year ' + str(year))
