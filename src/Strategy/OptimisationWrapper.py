def OptimiseTransmissionRatio(optimiser, powertrain, min_ratio=5, max_ratio=20):
    optimiser.AddVariable('Transmission Ratio', powertrain.ratio, min_ratio, max_ratio)

def OptimiseBurnLocations(optimiser, controller):

    for index, spacing in enumerate(controller.location_spacings):

        name = 'dt_' + str(index)
        optimiser.AddVariable(name, spacing, 0.01, 0.5)


