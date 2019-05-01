import time

from src.Track import ImportTrack
from src.Integration import RKF45, RK4, DP45, Butcher
from src import Control
from src.Model import VehicleModel, PowertrainModel, BrushedDCMotor
from src.Results import Results

track = ImportTrack.import_year('2019')
control = Control.BurnAndCoast_Velocity(min_vel=2)

motor = BrushedDCMotor.MaxonRE65(solver=Butcher.RK8, dt=0.5e-3, verbose=False)
powertrain = PowertrainModel.FreeWheel(motor, 4, transmission_efficiency=0.8, verbose=False)
car = VehicleModel.Vehicle(powertrain)

model_kwargs = {'track': track, 'control_function': control.demand}
s = Butcher.RK4()

t_s = time.time()
vehicle_results = s.solve(car, car.equation_of_motion, model_kwargs, [1e-4, 1e-4], dt=0.25, t_end=200, verbose=True)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)