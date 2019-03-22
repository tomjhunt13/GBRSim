from src.Track_original import *
from src.Powertrain import *
from src.Vehicle import *
from src.Results import *
from src.Control import *
from src.Track import ImportTrack



track = ImportTrack.import_year('2018')


# Motor
motor_properties = {'torque_constant': 0.0631, 'motor_efficiency': 0.8, 'motor_speed_constant': 138, 'no_load_speed': 2690}
battery_properties = {'max_discharge_power': 3000, 'discharge_voltage': 48, 'battery_efficiency': 0.9}
transmission_properties = {'transmission_ratio': 50, 'transmission_efficiency': 0.95}

powertrain = BrushedMotor(motor_properties, battery_properties, transmission_properties, number_of_motors=10)

control = CutoffSpeed(7.5)
# control = BurnAndCoast_Velocity(min_vel=2, max_vel=8)
# control = ConstantPower()

# Vehicle attributes
mass = 150      # Total vehicle mass (kg)
Crr = 0.005     # Coefficient of rolling resistance
Cd = 0.27       # Coefficient of drag
A = 1.3         # Frontal area (m^2)
Cs = 0.3        # Cornering stiffness
PoweredWheelRadius = 0.2       # Radius of tyre for powered wheel

vehicle_parameters = {'Mass': mass, 'Crr': Crr, 'Cd': Cd, 'A': A, 'Cs': Cs, 'PoweredWheelRadius': PoweredWheelRadius}

v = Vehicle(vehicle_parameters, powertrain)
vehicle_results = v.simulate(track, 0, [0.5, 0], control_function=control.demand, time_step=0.01, time_limit=210)


Animate(track, vehicle_results)


