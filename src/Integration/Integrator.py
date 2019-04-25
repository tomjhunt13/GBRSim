class Integrator:

    def __init__(self):
        pass

    def solve(self, model, model_update_function, model_kwargs, initial_conditions, dt=0.01, t_end=500):

        # Initialise state space
        self.f = model_update_function
        self.y = [initial_conditions]
        self.t = [0]
        self.dt = dt
        self.t_end = t_end
        self.information_dictionary = [{}]

        # Initialise model
        self.model = model
        self.model.initialise(initial_conditions, self.information_dictionary[0], **model_kwargs)

        # Simulation loop
        while self.end_condition():

            # Pre-step processing
            self.pre_step()

            # Step
            t_n = self.t[-1]
            y_n = self.y[-1]
            t_np1 = t_n + self.dt
            y_np1, dictionary_t_np1 = self.update(t_n, y_n)

            # Post-step processing
            self.model.post_step(t_np1, y_np1, dictionary_t_np1)

            # Update state
            self.t.append(t_np1)
            self.y.append(y_np1)
            self.information_dictionary.append(dictionary_t_np1)

        # Update info dict
        update_dictionary_keys(self.information_dictionary[1], self.information_dictionary[0])

        return self.information_dictionary

    def end_condition(self):

        if self.t[-1] >= self.t_end:
            return False

        return self.model.end_condition()

    def pre_step(self):
        pass


def update_dictionary_keys(full_dictionary, destination_dictionary):
    """
    Updates destination dictionary with keys its missing from full dictionary
    :param full_dictionary:
    :param destination_dictionary:
    :return:
    """

    for key in full_dictionary.keys():
        if key not in destination_dictionary.keys():

            destination_dictionary[key] = 0
