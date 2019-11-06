import time
from src.Track import ImportTrack, Track, Line, Circle
from src.Strategy import Controller
from src.Model import SeparatedVehicleModel, PowertrainModel, BrushedDCMotor, IntegratedVehicleModel
from src.Results import Results
from src.Integration import DP45, Butcher, RK4, Euler, RKF45


# ------- Import Track ------- #
track = ImportTrack.import_year('2019')
total_time = 39 * 60
laps = 11
desired_time = (total_time / laps) - 5

# ------- Make Model ------- #
controller = Controller.ConstantPower()
transmission_ratio = 12

motor = BrushedDCMotor.MaxonRE65(dt=1e-3, verbose=False, battery_efficiency=0.9)
powertrain = PowertrainModel.DirectTransmission(motor, transmission_ratio, transmission_efficiency=0.8, verbose=False)

vehicle_parameters = {'Cd': 0.3, 'VehicleMass': 100}
car = SeparatedVehicleModel.SeparatedVehicleModel(powertrain, verbose=True, vehicle_parameters=vehicle_parameters)
t_s = time.time()

vehicle_results = car.simulate([1e-4, 1e-4], dt=1, t_end=300, verbose=True, track=track, controller=controller, solver=DP45.DP45)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)


