from src.Powertrain import *
from src.Vehicle.Vehicle import *
from src.Results import *
from src.Control import *
from src.Track import ImportTrack
from src.Optimisation.Optimisation import *



track = ImportTrack.import_year('2018')

transmission = Transmission(9.339752197265625, 0.9)
powertrain = MaxonRE65()

# control = CutoffSpeed(7.5)
# control = BurnAndCoast_Velocity(min_vel=2, max_vel=8)
control = ConstantPower()
v = Vehicle(powertrain, transmission)

# OptimiseTransmissionRatio(15, v, track, control.demand)


vehicle_results = v.simulate(track, 0, [0, 0], control_function=control.demand, time_step=0.01, time_limit=600)


Animate(track, vehicle_results)


