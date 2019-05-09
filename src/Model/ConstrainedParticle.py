import numpy as np

from src.Model import Model

class ConstrainedParticle(Model.Model):

    def __init__(self):

        super(ConstrainedParticle, self).__init__()

    def initialise(self, initial_conditions, information_dictionary, **kwargs):

        # ------- Initialise particle on track ------- #
        self.track = kwargs['track']
        self.track_length = self.track.total_length()
        self.laps = 0
        self.net_distance = 0
        self.arc_length_n = 0

        # Optional kwargs
        optional_kwargs = {'lap_limit': 1}
        for keyword in optional_kwargs:
            if keyword not in kwargs:
                kwargs[keyword] = optional_kwargs[keyword]

        self.lap_limit = kwargs['lap_limit']
        self.total_distance = self.lap_limit * self.track.length
        self.lap_limit_reached = False

        # Update information dictionary
        starting_segment, segment_index, lambda_param = self.track.segment_lambda_from_arc_length(initial_conditions[0])
        information_dictionary['Velocity (m/s)'] = initial_conditions[1]
        information_dictionary['segment'] = segment_index
        information_dictionary['lambda_param'] = lambda_param

        self.post_initialisation(**kwargs)

    def post_initialisation(self, **kwargs):
        pass

    def end_condition(self):

        if not self.lap_limit_reached:
            return True

        return False

    def post_step(self, t_np1, y_np1, information_dictionary):

        # Unpack new state
        starting_segment, segment_index, lambda_param = self.track.segment_lambda_from_arc_length(y_np1[0])

        # Continuity
        if y_np1[0] > self.track.length:
            segment_index = 0
            y_np1[0] = 0

        elif y_np1[0] < 0:
            segment_index = len(self.track.segments) - 1
            y_np1[0] = self.track.length

        else:
            # Update net distance
            if y_np1[1] > 0:
                self.net_distance += y_np1[0] - self.arc_length_n

            elif y_np1[1] < 0:
                self.net_distance += y_np1[0] - self.arc_length_n

            if self.net_distance > self.total_distance:
                self.lap_limit_reached = True

        self.arc_length_n = y_np1[0]
        information_dictionary['Velocity (m/s)'] = y_np1[1]
        information_dictionary['segment'] = segment_index
        information_dictionary['lambda_param'] = lambda_param

    def weight_force(self, theta):
        """
        :param theta: Angle between direction of travel and horizontal. +ve is uphill

        :return: The force contribution due to the weight of the particle.
                 Value is signed, ie a -ve value is a force acting in the opposite direction to the velocity
        """

        return -1 * self.mg * np.sin(theta)

    def aerodynamic_drag_force(self, V):
        """
        :param segment_index:
        :param lambda_param:
        :param V:
        :return:
        """

        return self.aero_force * V * V

def direction_modifier(V):
    """
    :param V:
    :return:
    """
    if V > 0:
        return 1

    elif V < 0:
        return -1

    else:
        return 0