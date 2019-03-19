import numpy as np

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


    def draw_coordinates(self, num_arc_points=20):
        """
        Calculate 3D coordinates to draw track
        :param num_arc_points: Number of points used to discretise curve
        :return: tuple: x_coordinates, y_coordinates, z_coordinates
        """

        x = []
        y = []
        z = []


        # Loop over each track segment and draw on 3D plot
        for index, segment in enumerate(self.track):
            if segment['type'] == 'line':

                x += [segment['start'][0], segment['end'][0]]
                y += [segment['start'][1], segment['end'][1]]
                z += [segment['start'][2], segment['end'][2]]

            elif segment['type'] == 'arc':

                num_points = num_arc_points
                for i in range(num_points):
                    lambda_param = i / (num_points - 1)

                    P = self.position(index, lambda_param)

                    x.append(P[0])
                    y.append(P[1])
                    z.append(P[2])

        return x, y, z

    def gradient(self, segment, lambda_param):
        """
        For a given point on the track, get the gradient in radians
        :param segment: integer - Index of segment of track to use
        :param lambda_param: float between 0 and 1 - value of parameter lambda to get gradient of
        :return: float - Gradient of track in radians
        """

        # Get directional vector
        direction = self.direction(segment, lambda_param)

        # If flat
        if np.isclose(direction[2], 0):
            return 0

        # Get angle
        dot = np.dot(direction, [0, 0, 1])
        angle_to_vertical = np.arccos(dot)

        return (np.pi / 2) - angle_to_vertical

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

    # Allow 'start', 'end', 'radius' assuming flat
    if 'radius' in arc_dictionary.keys():
        arc_from_radius(arc_dictionary)
        return True


    # Unpack dictionary
    A = arc_dictionary['start']
    B = arc_dictionary['end']
    C = arc_dictionary['centre']

    # Get radius
    CA = np.subtract(A, C)
    CB = np.subtract(B, C)
    radius = np.linalg.norm(CA)

    if not np.isclose(np.linalg.norm(CB), radius):
        return False

    # Arc angle and length
    arc_angle = np.arccos(np.dot(CA, CB) / (np.linalg.norm(CA) * np.linalg.norm(CB)))
    arc_length = arc_angle * radius

    # Update arc dictionary - dictionary is mutable so can operate on it here without returning new dictionary
    arc_dictionary['radius'] = radius
    arc_dictionary['angle'] = arc_angle
    arc_dictionary['length'] = arc_length

    return True

def arc_from_radius(arc_dictionary):


    A = arc_dictionary['start']
    B = arc_dictionary['end']
    r = arc_dictionary['radius']

    arc_dictionary['centre'] = centre_from_two_points(A, B, r)

def centre_from_two_points(A, B, r):
    """

    :param A:
    :param B:
    :param r:
    :return:
    """

    AB = np.subtract(B, A)
    AB_mid = A + np.multiply(0.5, AB)


    n_vec = np.cross(AB, [0, 0, 1])
    n_vec_unit = (1 / np.linalg.norm(n_vec)) * n_vec

    C = AB_mid + np.multiply(r, n_vec_unit)

    return list(C)




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