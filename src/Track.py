import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

class Track:
    def __init__(self, track):
        """

        :param track:

        Track is made of curve segments. These are of type:

            - Line:
                Straight line with parameters:
                    start: Coordinates of start point of line (3 element list)
                    end: Coordinates of end point of line (3 element list)

            - Arc:
                Constant radius arc with parameters:
                    start: Coordinates of start point of arc (3 element list)
                    end: Coordinates of end point of arc (3 element list)
                    centre: Coordinates of centre point of arc (3 element list)

        """

        # Environment attributes
        self.rho = 1.225    # Air density (g / cm^3)
        self.g = 9.81


        # Segments
        self.current_segment = 0
        self.track = track

        # Check segments are valid
        for segment in self.track:
            if segment['type'] == 'line':
                line(segment)

            elif segment['type'] == 'arc':
                if not arc(segment):
                    raise Exception('Arc not valid')


    def draw_coordinates(self):
        """"
        :return:
        """

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_aspect('equal')

        # Loop over each track segment and draw on 3D plot
        for index, segment in enumerate(self.track):
            if segment['type'] == 'line':

                x = [segment['start'][0], segment['end'][0]]
                y = [segment['start'][1], segment['end'][1]]
                z = [segment['start'][2], segment['end'][2]]

                ax.plot(x, y, z)
                print('line')

            elif segment['type'] == 'arc':

                x = []
                y = []
                z = []

                num_points = 20
                for i in range(num_points):
                    lambda_param = i / (num_points - 1)

                    P = self.position(index, lambda_param)

                    x.append(P[0])
                    y.append(P[1])
                    z.append(P[2])

                ax.plot(x, y, z)



        fig.show()

        return fig

    def gradient(self, segment, lambda_param):
        """
        For a given point on the track, get the gradient in radians
        :param segment: integer - Index of segment of track to use
        :param lambda_param: float between 0 and 1 - value of parameter lambda to get gradient of
        :return: float - Gradient of track in radians
        """

        # Get directional vector
        direction = self.direction(segment, lambda_param)

        # Get unit vec of xy plane components
        direction_xy = [direction[0], direction[1], 0]
        direction_xy_unit = np.multiply((1 / np.linalg.norm(direction_xy)), direction_xy)

        # Dot two vectors
        dot = np.dot(direction_xy_unit, direction)

        return np.arccos(dot)

    def direction(self, segment, lambda_param):
        """
        For a given point on the track, get the direction of travel a unit vector
        :param segment: integer - Index of segment of track to use
        :param lambda_param: float between 0 and 1 - value of parameter lambda to get direction of
        :return: 3 element list - Direction vector of track
        """

        # Get segment
        track_segment = self.track[segment]

        # Line segment
        if track_segment['type'] == 'line':
            return track_segment['direction']

        # Arc
        if track_segment['type'] == 'arc':

            A = track_segment['start']
            B = track_segment['end']
            C = track_segment['centre']

            P = self.position(segment, lambda_param)
            PC = np.subtract(C, P)
            PC_unit = np.multiply((1 / np.linalg.norm(PC)), PC)

            N = triNormal(A, B, C)

            return list(np.cross(PC_unit, N))

        return [0, 0, 0]

    def position(self, segment, lambda_param):
        """
        For a given point on the track, get the position of the vehicle in 3D space
        :param segment: integer - Index of segment of track to use
        :param lambda_param: float between 0 and 1 - value of parameter lambda to get position of
        :return: 3 element list - Position vector of vehicle
        """

        # Get segment
        track_segment = self.track[segment]

        # Line segment
        if track_segment['type'] == 'line':

            A = track_segment['start']
            B = track_segment['end']

            AB = np.subtract(B, A)
            return list(np.add(A, np.multiply(lambda_param, AB)))

        # Arc
        if track_segment['type'] == 'arc':

            # Get alpha
            alpha = lambda_param * track_segment['angle']

            A = track_segment['start']
            B = track_segment['end']
            C = track_segment['centre']

            return pointOnArc(A, B, C, alpha)

        return [0, 0, 0]




def line(line_dictionary):
    """
    Get information about line segment
    :param line_dictionary: line dictionary of form: {'type': 'line', 'start': [...], 'end': [...]},
    """

    # Unpack dictionary
    A = line_dictionary['start']
    B = line_dictionary['end']

    AB = np.subtract(B, A)
    line_dictionary['length'] = np.linalg.norm(AB)
    line_dictionary['direction'] = list(np.multiply((1 / line_dictionary['length']), AB))


def arc(arc_dictionary):
    """
    Test if arc segment is valid and get information about it
    :param arc_dictionary: arc dictionary of form: {'type': 'arc', 'start': [...], 'end': [...], 'centre': [...]}
    :return: Bool - is arc valid
    """

    # Unpack dictionary
    A = arc_dictionary['start']
    B = arc_dictionary['end']
    C = arc_dictionary['centre']

    # Get radius
    CA = np.subtract(A, C)
    CB = np.subtract(B, C)
    radius = np.linalg.norm(CA)

    if np.linalg.norm(CB) != radius:
        return False

    # Arc angle and length
    arc_angle = np.arccos(np.dot(CA, CB))
    arc_length = arc_angle * radius

    # Update arc dictionary - dictionary is mutable so can operate on it here without returning new dictionary
    arc_dictionary['radius'] = radius
    arc_dictionary['angle'] = arc_angle
    arc_dictionary['length'] = arc_length

    return True


def triNormal(A, B, C):
    """
    Finds a unit vector normal to a triangle of coordinates A, B and C. ABC are taken in anticlockwise order
    :param A: 3 element list - A coordinate
    :param B: 3 element list - B coordinate
    :param C: 3 element list - C coordinate
    :return:  3 element list - Unit vector normal to triangle
    """

    AB = np.subtract(B, A)
    AC = np.subtract(C, A)
    ABxAC = np.cross(AB, AC)

    return list(np.multiply((1 / np.linalg.norm(ABxAC)), ABxAC))


def pointOnArc(A, B, C, alpha):
    """
    Calculate point P on arc given start point, end point, centre and angle
    :param A: Start point of arc
    :param B: End point of arc
    :param C: Centre of arc
    :param alpha: angle made by vector CP to CA (radians)
    :return: P
    """

    # Create new coordinates in plane of ABC with coordinate vectors: CA, N cross CA
    # Get P in these new coordinates and transform back

    CA = np.subtract(A, C)
    R = np.linalg.norm(CA)
    CA_unit = np.multiply((1 / R), CA)
    N = triNormal(A, B, C)

    NxCA = np.cross(N, CA_unit)

    i_component = np.multiply(R * np.cos(alpha), CA_unit)
    j_component = np.multiply(R * np.sin(alpha), NxCA)

    P = list(np.add(C, np.add(i_component, j_component)))

    return P



if __name__ == '__main__':




    track_1 = [
        {'type': 'line', 'start': [0, 0, 0], 'end': [100, 0, 0]},
        {'type': 'arc', 'start': [100, 0, 0], 'end': [150, 50, 0], 'centre': [100, 50, 0]},
        {'type': 'arc', 'start': [150, 50, 0], 'end': [100, 100, 0], 'centre': [100, 50, 0]},
        {'type': 'line', 'start': [100, 100, 0], 'end': [0, 100, 0]},
        {'type': 'arc', 'start': [0, 100, 0], 'end': [-50, 50, 0], 'centre': [0, 50, 0]},
        {'type': 'arc', 'start': [-50, 50, 0], 'end': [0, 0, 0], 'centre': [0, 50, 0]}
    ]

    track = Track(track_1)
    track.draw()



    print(track.direction(0, 0.5))
