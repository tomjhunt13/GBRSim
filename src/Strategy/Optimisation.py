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

controller = Controller.BurnAndCoast(number_of_burns=6)

motor = BrushedDCMotor.MaxonRE65(dt=1e-3, verbose=False)
powertrain = PowertrainModel.FreeWheel(motor, 10, transmission_efficiency=transmission_efficiency, verbose=False)
car = SeparatedVehicleModel.SeparatedVehicleModel(powertrain, vehicle_parameters={'VehicleMass': vehicle_mass, 'Cd': Cd}, verbose=False)

sim = SimulationWrapper.SimulationWrapper(car, track, controller, desired_time)


# ------- Create Optimisation ------- #
optimiser = SA.SA()
OptimisationWrapper.OptimiseTransmissionRatio(optimiser, powertrain)
OptimisationWrapper.OptimiseBurnLocations(optimiser, controller)

optimum = optimiser.optimise(sim.cost, max_iterations=1)

#
# # ------- Run Optimum Solution ------- #
# optimum = {'Transmission Ratio': 11.920413573272526, 'dt_0': 0.11374758604599532, 'dt_1': 0.1978071313552845, 'dt_2': 0.4874739444596916, 'dt_3': 0.4977957254989296, 'dt_4': 0.47032508069801215, 'dt_5': 0.3013538015229851, 'dt_6': 0.2332330583327169, 'dt_7': 0.1509738869667685, 'dt_8': 0.4422626476936329, 'dt_9': 0.2897778433017719}
#
# powertrain.ratio[0] = optimum['Transmission Ratio']
# controller.location_spacings[0][0] = optimum['dt_0']
# controller.location_spacings[1][0] = optimum['dt_1']
# controller.location_spacings[2][0] = optimum['dt_2']
# controller.location_spacings[3][0] = optimum['dt_3']
# controller.location_spacings[4][0] = optimum['dt_4']
# controller.location_spacings[5][0] = optimum['dt_5']
# controller.location_spacings[6][0] = optimum['dt_6']
# controller.location_spacings[7][0] = optimum['dt_7']
# controller.location_spacings[8][0] = optimum['dt_6']
# controller.location_spacings[9][0] = optimum['dt_7']
#
# vehicle_results = sim.simulate()
# Results.process_results(track, vehicle_results)






