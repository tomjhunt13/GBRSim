import time

from src.Track import ImportTrack
from src.Integration import RKF45, RK4, DP45, Butcher
from src import Control
from src.Model import VehicleModel, PowertrainModel, BrushedDCMotor
from src.Results import Results

track = ImportTrack.import_year('2018')
control = Control.ConstantPower()

motor = BrushedDCMotor.BrushedMotor()
powertrain = PowertrainModel.DirectTransmission(motor, 10, transmission_efficiency=0.8, verbose=True)
car = VehicleModel.Vehicle()

model_kwargs = {'track': track, 'control_function': control.demand}


# s = RKF45.RKF45()   # Elapsed time: 189.21647214889526
# s = RK4.RK4()       # Elapsed time: 183.0775592327118
# s = DP45.DP45()       # Elapsed time: Huuuge
s = Butcher.RK8()

t_s = time.time()
vehicle_results = s.solve(car, car.equation_of_motion, model_kwargs, [1e-4, 1e-4, 1e-4], dt=1, t_end=200, verbose=True)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)