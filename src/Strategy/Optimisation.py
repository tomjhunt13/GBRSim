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


"""
# ------- Run Optimum Solution ------- #
# optimum = {'Transmission Ratio': 11.848357931345003, 'dt_0': 0.03765359806812562, 'dt_1': 0.012218674553587041, 'dt_2': 0.07158429447431057, 'dt_3': 0.04889603508825501}
# optimum = {'Transmission Ratio': 12.761215988102006, 'dt_0': 0.4871151343836191, 'dt_1': 0.2545449394626367, 'dt_2': 0.3727696060224981, 'dt_3': 0.3806168035520914, 'dt_4': 0.287938645987174, 'dt_5': 0.17669039230306138}
# optimum = {'Transmission Ratio': 12.806907595401823, 'dt_0': 0.2841263945703105, 'dt_1': 0.23016085710340192, 'dt_2': 0.38105613321140686, 'dt_3': 0.4015312871645489, 'dt_4': 0.3954765219246022, 'dt_5': 0.18622417540329397, 'dt_6': 0.46833854188831014, 'dt_7': 0.34189831606281446}
# optimum = {'Transmission Ratio': 11.920413573272526, 'dt_0': 0.11374758604599532, 'dt_1': 0.1978071313552845, 'dt_2': 0.4874739444596916, 'dt_3': 0.4977957254989296, 'dt_4': 0.47032508069801215, 'dt_5': 0.3013538015229851, 'dt_6': 0.2332330583327169, 'dt_7': 0.1509738869667685, 'dt_8': 0.4422626476936329, 'dt_9': 0.2897778433017719}
# optimum = {'Transmission Ratio': 11.763085991795872, 'dt_0': 0.06368652684675301, 'dt_1': 0.02313972333562985, 'dt_2': 0.4073335227607784, 'dt_3': 0.3751150737522706, 'dt_4': 0.26139511866556253, 'dt_5': 0.10817833943353737, 'dt_6': 0.2505618630646209, 'dt_7': 0.02652961074297732, 'dt_8': 0.24676489890867925, 'dt_9': 0.3290532824634609, 'dt_10': 0.01097307535400959, 'dt_11': 0.20679935021387186}


# powertrain.ratio[0] = optimum['Transmission Ratio']
# controller.location_spacings[0][0] = optimum['dt_0']
# controller.location_spacings[1][0] = optimum['dt_1']
# controller.location_spacings[2][0] = optimum['dt_2']
# controller.location_spacings[3][0] = optimum['dt_3']
# controller.location_spacings[4][0] = optimum['dt_4']
# controller.location_spacings[5][0] = optimum['dt_5']
# controller.location_spacings[6][0] = optimum['dt_6']
# controller.location_spacings[7][0] = optimum['dt_7']
# controller.location_spacings[8][0] = optimum['dt_8']
# controller.location_spacings[9][0] = optimum['dt_9']
# controller.location_spacings[10][0] = optimum['dt_10']
# controller.location_spacings[11][0] = optimum['dt_11']

vehicle_results = sim.simulate()
Results.process_results(track, vehicle_results)
"""





