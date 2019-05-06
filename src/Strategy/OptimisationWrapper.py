def OptimiseTransmissionRatio(optimiser, powertrain, min_ratio=5, max_ratio=20):
    """
    Adds transmission ratio as variable to optimisation
    :param optimiser: The optimiser instance
    :param powertrain: Reference to the powertrain used
    :param min_ratio: Minimum value of transmission ratio
    :param max_ratio:
    :return:
    """
    optimiser.AddVariable('Transmission Ratio', powertrain.ratio, min_ratio, max_ratio)


def OptimiseBurnLocations(optimiser, controller):

    for index, spacing in enumerate(controller.location_spacings):

        name = 'dt_' + str(index)
        optimiser.AddVariable(name, spacing, 0.01, 0.5)


def OptimiseBurnDemands(optimiser, controller):

    for index, demand in enumerate(controller.demands):

        name = 'throttle_' + str(index)
        optimiser.AddVariable(name, demand, 0, 1)