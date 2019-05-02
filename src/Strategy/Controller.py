class Controller:

    def demand(self, **kwargs):
        pass

    def initialise(self, **kwargs):
        pass

class ConstantPower(Controller):

    def demand(self, **kwargs):
        return 1

class CutoffSpeed(Controller):
    def __init__(self, cutoff_speed=10):
        self.cutoff_speed = [cutoff_speed]

    def demand(self, **kwargs):

        # Unpack args
        V = kwargs['V'] * 2.237
        theta = kwargs['theta']

        if V > self.cutoff_speed[0] or theta < 0:
            throttle_demand = 0

        else:
            throttle_demand = 1

        return throttle_demand

class BurnAndCoast_Velocity(Controller):
    def __init__(self, min_vel=5, max_vel=15):
        self.min_vel = [min_vel]
        self.max_vel = [max_vel]

        self.previous_throttle = 0

    def demand(self, **kwargs):

        # Unpack args
        V = kwargs['V'] * 2.237

        previous_throttle = int(self.previous_throttle)

        if V < self.min_vel[0]:
            self.previous_throttle = 1
            return 1

        if V > self.max_vel[0]:
            self.previous_throttle = 0
            return 0


        # At this point min_vel < V < max_vel
        if previous_throttle == 1:
            # Must still be burning
            self.previous_throttle = 1
            return 1

        # Else must be coasting
        self.previous_throttle = 0
        return 0


class BurnAndCoast(Controller):

    def __init__(self, number_of_burns=1, min_speed_mph=5, verbose=False):

        # Initialise evenly
        self.number_of_burns = number_of_burns
        number_of_segments = number_of_burns * 2 + 1
        spacing = 1 / number_of_segments

        self.location_spacings = [[spacing], [spacing]] * self.number_of_burns
        self.demands = [1] * self.number_of_burns

        self.min_speed = min_speed_mph / 2.237

        self.verbose = verbose

    def initialise(self, **kwargs):

        # Initialise evenly
        self.track = kwargs['track']
        self.track_length = float(len(self.track.segments))

    def demand(self, **kwargs):

        if kwargs['V'] < self.min_speed:
            return 1

        # Get distance around track
        lambda_param = kwargs['lambda_param']
        distance = lambda_param / self.track_length

        if self.verbose:
            print('Distance: ' + str(distance))

        total_spacing = 0
        demand = 0

        for index, spacing in enumerate(self.location_spacings):


            demand = 1 - demand

            total_spacing += spacing[0]

            if distance < total_spacing:
                return demand

        return demand


