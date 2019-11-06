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


# ------- Create Optimisation ------- #
optimiser = SA.SA()
OptimisationWrapper.OptimiseTransmissionRatio(optimiser, powertrain)
OptimisationWrapper.OptimiseBurnLocations(optimiser, controller)

optimum = optimiser.optimise(sim.cost, max_iterations=1)






