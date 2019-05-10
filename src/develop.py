import time
from src.Track import ImportTrack, Track, Line, Circle
from src.Strategy import Controller
from src.Model import VehicleModel, PowertrainModel, BrushedDCMotor, IntegratedModel
from src.Results import Results

"""
To do:

1) Make parent to optimiser which handles variable changes

2) Look for telemetry from previous GBR


"""


t_s = time.time()
track = ImportTrack.import_year('2018')
print('Track import time: ' + str(time.time() - t_s))

# controller = Controller.BurnAndCoast(number_of_burns=2, verbose=True)
controller = Controller.ConstantPower()
transmission_ratio = 12
# controller.location_spacings = [[0.3988230623101998], [0.01580746431742152], [0.2007107484300042], [0.18355568719861992]]
# transmission_ratio = 12

motor = BrushedDCMotor.MaxonRE65(dt=1e-3, verbose=False)
powertrain = PowertrainModel.DirectTransmission(motor, transmission_ratio, transmission_efficiency=0.8, verbose=False)

vehicle_parameters = {'Cd': 0.2}

# car = VehicleModel.Vehicle(powertrain, verbose=True)

car  = IntegratedModel.IntegratedModel({'transmission_ratio': transmission_ratio}, verbose=True)


t_s = time.time()
vehicle_results = car.simulate([1e-4, 1e-4, 1e-4], dt=0.001, t_end=300, verbose=True, track=track, controller=controller)
print('Elapsed time: ' + str(time.time() - t_s))

Results.process_results(track, vehicle_results)


