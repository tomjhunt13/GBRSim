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

