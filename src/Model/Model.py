class Model:

    def __init__(self):

        pass

    def end_condition(self):

        return True

    def pre_step(self, t_n, y_n):

        pass

    def post_step(self, t_np1, y_np1, information_dictionary):

        pass

    def initialise(self, initial_conditions, information_dictionary, **kwargs):

        pass