import time
from src.Track import ImportTrack, Track, Line, Circle
from src.Integration import RKF45, RK4, DP45, Butcher, Euler
from src.Strategy import Controller
from src.Model import VehicleModel, PowertrainModel, BrushedDCMotor, IntegratedModel
from src.Results import Results

l = Line.Line([[0, 0, 0], [1000, 0, 0]])
l = Circle.Circle(300)
track = Track.Track([l])


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

# # 2018, 2 burns
# track = ImportTrack.import_year('2018')
# controller = Controller.BurnAndCoast(number_of_burns=2)
# tr = 11.562858266880488
# controller.location_spacings = [[0.09810088786524852],
#                                 [0.36485885775760474],
#                                 [0.34170404681209077],
#                                 [0.14749639937281714]]

# # 2018, 6 burns
# track = ImportTrack.import_year('2018')
# controller = Controller.BurnAndCoast(number_of_burns=6)
# tr = 14.731240558547322
# controller.location_spacings = [[0.267547362730309],
#                                 [0.40033148399078783],
#                                 [0.19291264702309055],
#                                 [0.21404195822957228],
#                                 [0.02157327090312653],
#                                 [0.22404043454019507],
#                                 [0.19630226193112],
#                                 [0.05181996418860302],
#                                 [0.223274216442926],
#                                 [0.45128993450618554],
#                                 [0.07632456462219304],
#                                 [0.32582525419810304]]


# # 2018, 5 burns
# track = ImportTrack.import_year('2018')
# controller = Controller.BurnAndCoast(number_of_burns=5, verbose=True)
# tr = 16.625476896762848
# controller.location_spacings = [[0.2325945767871258],
#                                 [0.2235767597303815],
#                                 [0.27717865129200847],
#                                 [0.450859423314137],
#                                 [0.3381125682936139],
#                                 [0.012665318369254877],
#                                 [0.20441195485078167],
#                                 [0.32473284386235557],
#                                 [0.07540777865509461],
#                                 [0.37483379897557567]]


controller = Controller.ConstantPower()
tr = 12
# track = ImportTrack.import_year('2019')

motor = BrushedDCMotor.MaxonRE65(dt=1e-3, verbose=False)
# powertrain = PowertrainModel.FreeWheel(motor, tr, transmission_efficiency=0.8, verbose=False)
powertrain = PowertrainModel.DirectTransmission(motor, tr, transmission_efficiency=0.8, verbose=False)
# car = VehicleModel.Vehicle(powertrain, verbose=True)
car = IntegratedModel.IntegratedModel()

model_kwargs = {'track': track, 'controller': controller}
s = RK4.RK4()

t_s = time.time()
vehicle_results = s.solve(car, car.equation_of_motion, model_kwargs, [1e-4, 1e-4, 1e-4], dt=0.001, t_end=10, verbose=True)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)


