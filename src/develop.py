import time

from src.Track import ImportTrack
from src.Integration import RKF45, RK4, DP45, Butcher
from src.Strategy import Controller
from src.Model import VehicleModel, PowertrainModel, BrushedDCMotor
from src.Results import Results

track = ImportTrack.import_year('2019')
controller = Controller.BurnAndCoast(number_of_burns=3)
controller.location_spacings[0][0] = 0.5
# controller = Controller.BurnAndCoast_Velocity()

motor = BrushedDCMotor.MaxonRE65(solver=RK4.RK4, dt=1e-3, verbose=False)
powertrain = PowertrainModel.FreeWheel(motor, 10, transmission_efficiency=0.8, verbose=False)
car = VehicleModel.Vehicle(powertrain, verbose=True)

model_kwargs = {'track': track, 'controller': controller}
s = RK4.RK4()

t_s = time.time()
vehicle_results = s.solve(car, car.equation_of_motion, model_kwargs, [1e-4, 1e-4], dt=0.25, t_end=300, verbose=True)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)