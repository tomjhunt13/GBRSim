import numpy as np

from src.Model import Model

class ConstrainedParticle(Model.Model):

    def __init__(self):

        super(ConstrainedParticle, self).__init__()


    def initialise(self, initial_conditions, information_dictionary, **kwargs):

        # Initialise particle on track
        starting_segment = int(np.floor(initial_conditions[0]))
        self.current_segment = starting_segment
        self.laps = 0
        self.track = kwargs['track']
        self.y = [initial_conditions]
        self.t = [0]
        self.highest_segment = starting_segment
        self.track_length = self.track.total_length()
        self.distance = 0

        # Optional kwargs
        optional_kwargs = {'lap_limit': 1}
        for keyword in optional_kwargs:
            if keyword not in kwargs:
                kwargs[keyword] = optional_kwargs[keyword]

        self.lap_limit = kwargs['lap_limit']

        # Update information dictionary
        information_dictionary['Velocity (m/s)'] = initial_conditions[1] * self.track.segments[starting_segment].length
        information_dictionary['segment'] = starting_segment
        information_dictionary['lambda_param'] = initial_conditions[0]

        self.post_initialisation(**kwargs)

    def post_initialisation(self):
        pass

    def end_condition(self):

        if self.number_of_laps() != self.lap_limit:
            return True

        return False

    def post_step(self, t_np1, y_np1, information_dictionary):

        y_n = self.y[-1]
        t_n = self.t[-1]
        segment_index = int(np.floor(y_np1[0]))
        lambda_param = y_np1[0] - segment_index

        # Handle changing segment
        if segment_index != self.current_segment:

            # Interpolate to find time of segment change
            y_0_intermediate = max(np.floor(y_np1[0]), np.floor(y_n[0]))
            interpolant_ratio = ((y_0_intermediate - y_n[0]) / (y_np1[0] - y_n[0]))
            t_intermediate = t_n + interpolant_ratio * (t_np1[0] - t_n)
            y_1_intermediate = y_n[1] + interpolant_ratio * (y_np1[1] - y_n[1])

            # Continuity
            if segment_index > len(self.track.segments) - 1:
                segment_index = 0
                y_0_intermediate = 0

            elif segment_index < 0:
                segment_index = len(self.track.segments) - 1
                y_0_intermediate = segment_index + 0.99999

            # Write solution
            self.current_segment = segment_index
            self.t.append(t_intermediate)
            self.y.append([y_0_intermediate, y_1_intermediate])

            # Update y_np1 and t_np1
            t_np1[0] = t_intermediate
            y_np1[0] = y_0_intermediate
            y_np1[1] = y_1_intermediate

        else:
            segment_index = int(np.floor(y_np1[0]))
            lambda_param = y_np1[0] - segment_index

        self.t.append(t_np1[0])
        self.y.append(y_np1)

        information_dictionary['segment'] = segment_index
        information_dictionary['lambda_param'] = lambda_param
        information_dictionary['t'] = t_np1[0]
        information_dictionary['y'] = y_np1

    def number_of_laps(self):
        """
        Number of laps car has done
        :return: Number of laps
        """
        return self.laps

    def _update_lap_counter(self, new_segment):

        # If first initial step
        if len(self.y) < 2:
            return

        # If segment incremented
        if (self.y[-1][0] < self.y[-2][0] and self.y[-1][1] > 0) or (np.floor(self.y[-1][0]) > np.floor(self.y[-2][0])):

            if new_segment < self.highest_segment:

                self.laps += 1
                self.highest_segment = new_segment

            else:

                self.highest_segment = new_segment

        # Else if decremented
        elif (self.y[-1][0] > self.y[-2][0] and self.y[-1][1] < 0) or (
                np.floor(self.y[-1][0]) < np.floor(self.y[-2][0])):

            self.highest_segment = new_segment

    def equation_of_motion(self, t, y, information_dictionary, **kwargs):
        pass