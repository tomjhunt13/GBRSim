import time
from src.Track import ImportTrack, Track, Line, Circle
from src.Strategy import Controller
from src.Model import VehicleModel, PowertrainModel, BrushedDCMotor, IntegratedModel
from src.Results import Results

l = Line.Line([[0, 0, 0], [1000, 0, 0]])
# l = Circle.Circle(50)
# track = Track.Track([l])

t_s = time.time()
track = ImportTrack.import_year('2019')
print('Track import time: ' + str(time.time() - t_s))

controller = Controller.ConstantPower()
transmission_ratio = 12

motor = BrushedDCMotor.MaxonRE65(dt=1e-3, verbose=False)
powertrain = PowertrainModel.DirectTransmission(motor, transmission_ratio, transmission_efficiency=0.8, verbose=False)
car = VehicleModel.Vehicle(powertrain, verbose=True)



t_s = time.time()
vehicle_results = car.simulate([1e-4, 1e-4], dt=0.25, t_end=10, verbose=True, track=track, controller=controller)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)


