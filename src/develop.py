import time

from src.Track import ImportTrack
from src.Integration import RKF45, RK4, DP45, Butcher
from src.Strategy import Controller
from src.Model import VehicleModel, PowertrainModel, BrushedDCMotor
from src.Results import Results



# # 2018, 3 burns
# track = ImportTrack.import_year('2018')
# controller = Controller.BurnAndCoast(number_of_burns=3)
# tr = 14.948379317600004
# controller.location_spacings = [[0.4411448913653955],
#                                 [0.4070995163788076],
#                                 [0.2841050295875881],
#                                 [0.334264360711445],
#                                 [0.14947800646632983],
#                                 [0.37957323617532296]]

# 2018, 2 burns
track = ImportTrack.import_year('2018')
controller = Controller.BurnAndCoast(number_of_burns=2)
tr = 11.562858266880488
controller.location_spacings = [[0.09810088786524852],
                                [0.36485885775760474],
                                [0.34170404681209077],
                                [0.14749639937281714]]



motor = BrushedDCMotor.MaxonRE65(solver=RK4.RK4, dt=1e-3, verbose=False)
powertrain = PowertrainModel.FreeWheel(motor, tr, transmission_efficiency=0.8, verbose=False)
car = VehicleModel.Vehicle(powertrain, verbose=False)

model_kwargs = {'track': track, 'controller': controller}
s = RK4.RK4()

t_s = time.time()
vehicle_results = s.solve(car, car.equation_of_motion, model_kwargs, [1e-4, 1e-4], dt=0.25, t_end=300, verbose=True)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)