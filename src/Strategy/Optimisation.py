# Model
from src.Track import ImportTrack
from src.Model import SeparatedVehicleModel
from src.Model import PowertrainModel
from src.Model import BrushedDCMotor

# Strategy
from src.Strategy import SimulationWrapper
from src.Strategy import Controller
from src.Strategy import OptimisationWrapper

# Optimiser
from src.Optimisation import SA

# Temp
from src.Results import Results


# ------- Import Track ------- #

# 2019
track = ImportTrack.import_year('2019')
total_time = 39 * 60
laps = 11
desired_time = (total_time / laps) - 5


# ------- Create model ------- #
transmission_efficiency = 0.95
vehicle_mass = 80
Cd = 0.25

controller = Controller.BurnAndCoast(number_of_burns=3)

motor = BrushedDCMotor.MaxonRE65(dt=1e-3, verbose=False)
powertrain = PowertrainModel.FreeWheel(motor, 10, transmission_efficiency=transmission_efficiency, verbose=False)
car = SeparatedVehicleModel.SeparatedVehicleModel(powertrain, vehicle_parameters={'VehicleMass': vehicle_mass, 'Cd': Cd}, verbose=False)

sim = SimulationWrapper.SimulationWrapper(car, track, controller, desired_time)


# # ------- Create Optimisation ------- #
# optimiser = SA.SA()
# OptimisationWrapper.OptimiseTransmissionRatio(optimiser, powertrain)
# OptimisationWrapper.OptimiseBurnLocations(optimiser, controller)
#
# optimum = optimiser.optimise(sim.cost, max_iterations=1)


# ------- Run Optimum Solution ------- #
optimum = {'Transmission Ratio': 12.761215988102006, 'dt_0': 0.4871151343836191, 'dt_1': 0.2545449394626367, 'dt_2': 0.3727696060224981, 'dt_3': 0.3806168035520914, 'dt_4': 0.287938645987174, 'dt_5': 0.17669039230306138}

powertrain.ratio[0] = optimum['Transmission Ratio']
controller.location_spacings[0][0] = optimum['dt_0']
controller.location_spacings[1][0] = optimum['dt_1']
controller.location_spacings[2][0] = optimum['dt_2']
controller.location_spacings[3][0] = optimum['dt_3']
controller.location_spacings[4][0] = optimum['dt_4']
controller.location_spacings[5][0] = optimum['dt_5']

vehicle_results = sim.simulate()
Results.process_results(track, vehicle_results)
# : 11.848357931345003, 'dt_0': 0.03765359806812562, 'dt_1': 0.012218674553587041, 'dt_2': 0.07158429447431057, 'dt_3': 0.04889603508825501}






