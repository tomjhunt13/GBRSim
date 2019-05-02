import time

from src.Track import ImportTrack
from src.Integration import RKF45, RK4, DP45, Butcher
from src.Strategy import Controller
from src.Model import VehicleModel, PowertrainModel, BrushedDCMotor
from src.Results import Results

track = ImportTrack.import_year('2018')
controller = Controller.BurnAndCoast(number_of_burns=5)
# controller.location_spacings[0][0] = 0.3176088178248599
# controller.location_spacings[1][0] = 0.30494126006598354
# controller.location_spacings[2][0] = 0.2268444260115963
# controller.location_spacings[3][0] = 0.22971726714767973


motor = BrushedDCMotor.MaxonRE65(solver=RK4.RK4, dt=1e-3, verbose=False)
powertrain = PowertrainModel.FreeWheel(motor, 10.911135027116293, transmission_efficiency=0.8, verbose=False)
car = VehicleModel.Vehicle(powertrain, verbose=False)

model_kwargs = {'track': track, 'controller': controller}
s = RK4.RK4()

t_s = time.time()
vehicle_results = s.solve(car, car.equation_of_motion, model_kwargs, [1e-4, 1e-4], dt=0.25, t_end=300, verbose=True)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)