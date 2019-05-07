import numpy as np

from src.Model import Model

class VehicleRoot(Model.Model):

    def __init__(self, vehicle_parameters={}, verbose=False):

        # Default attributes
        default_vehicle_attributes = {
            # Model properties
            'Mass': 170,
            'Crr': 1.5 * 0.001,     # http://www.eshopsem.com/boutique/product.php?id_product=75
            'Cd': 0.3,
            'A': 1.26,
            'PoweredWheelRadius': 0.279,
            'LongitudinalCoG': 0.5,     # Assume even weight distribution
        }

        for attribute in default_vehicle_attributes.keys():
            if attribute not in vehicle_parameters:
                vehicle_parameters[attribute] = default_vehicle_attributes[attribute]

        # Unpack vehicle attributes
        self.m = vehicle_parameters['Mass']     # Total vehicle mass (kg)
        self.Crr = vehicle_parameters['Crr']    # Coefficient of rolling resistance
        self.Cd = vehicle_parameters['Cd']      # Coefficient of drag
        self.A = vehicle_parameters['A']        # Frontal area (m^2)
        self.PoweredWheelRadius = vehicle_parameters['PoweredWheelRadius']       # Radius of tyre for powered wheel
        self.longitudinal_CoG = vehicle_parameters['LongitudinalCoG']

        # Admin properties
        self.verbose = verbose

        super(VehicleRoot, self).__init__()

    def end_condition(self):

        if self.number_of_laps() != self.lap_limit:
            return True

        return False

    def post_step(self, t_np1, y_np1, information_dictionary):

        y_n = self.y[-1]
        t_n = self.t[-1]
        segment_index = int(np.floor(y_np1[0]))
        lambda_param = y_np1[0] - segment_index

        # Handle changing segment
        if segment_index != self.current_segment:

            # Interpolate to find time of segment change
            y_0_intermediate = max(np.floor(y_np1[0]), np.floor(y_n[0]))
            interpolant_ratio = ((y_0_intermediate - y_n[0]) / (y_np1[0] - y_n[0]))
            t_intermediate = t_n + interpolant_ratio * (t_np1[0] - t_n)
            y_1_intermediate = y_n[1] + interpolant_ratio * (y_np1[1] - y_n[1])

            # Continuity
            if segment_index > len(self.track.segments) - 1:
                segment_index = 0
                y_0_intermediate = 0

            elif segment_index < 0:
                segment_index = len(self.track.segments) - 1
                y_0_intermediate = segment_index + 0.99999

            # Write solution
            self.current_segment = segment_index
            self.t.append(t_intermediate)
            self.y.append([y_0_intermediate, y_1_intermediate])

            # Update y_np1 and t_np1
            t_np1[0] = t_intermediate
            y_np1[0] = y_0_intermediate
            y_np1[1] = y_1_intermediate

        else:
            segment_index = int(np.floor(y_np1[0]))
            lambda_param = y_np1[0] - segment_index

        self.t.append(t_np1[0])
        self.y.append(y_np1)

        information_dictionary['segment'] = segment_index
        information_dictionary['lambda_param'] = lambda_param
        information_dictionary['t'] = t_np1[0]
        information_dictionary['y'] = y_np1

    def initialise(self, initial_conditions, information_dictionary, **kwargs):

        # Initialise vehicle on track
        starting_segment = int(np.floor(initial_conditions[0]))
        self.current_segment = starting_segment
        self.laps = 0
        self.track = kwargs['track']
        self.y = [initial_conditions]
        self.t = [0]
        self.highest_segment = starting_segment
        self.track_length = self.track.total_length()
        self.distance = 0

        # Initialise controller
        self.controller = kwargs['controller']
        self.controller.initialise(track=self.track)
        self.control_function = self.controller.demand

        # Optional kwargs
        optional_kwargs = {'lap_limit': 1}
        for keyword in optional_kwargs:
            if keyword not in kwargs:
                kwargs[keyword] = optional_kwargs[keyword]

        self.lap_limit = kwargs['lap_limit']

        # Update information dictionary
        information_dictionary['Velocity (m/s)'] = initial_conditions[1] * self.track.segments[starting_segment].length
        information_dictionary['segment'] = starting_segment
        information_dictionary['lambda_param'] = initial_conditions[0]

        # Pre-calculate constants
        self.aero_force = 0.5 * self.Cd * self.A * self.track.rho
        self.mg = self.m * self.track.g
        self.rolling_resistance_force = self.mg * self.Crr

    def number_of_laps(self):
        """
        Number of laps car has done
        :return: Number of laps
        """
        return self.laps

    def _update_lap_counter(self, new_segment):

        # If first initial step
        if len(self.y) < 2:
            return

        # If segment incremented
        if (self.y[-1][0] < self.y[-2][0] and self.y[-1][1] > 0) or (np.floor(self.y[-1][0]) > np.floor(self.y[-2][0])):

            if new_segment < self.highest_segment:

                self.laps += 1
                self.highest_segment = new_segment

            else:

                self.highest_segment = new_segment

        # Else if decremented
        elif (self.y[-1][0] > self.y[-2][0] and self.y[-1][1] < 0) or (np.floor(self.y[-1][0]) < np.floor(self.y[-2][0])):

            self.highest_segment = new_segment

    def equation_of_motion(self, t, y, information_dictionary, **kwargs):

        if self.verbose:
            print('State: ' + str(y))

        # Get world variables from position
        segment_index = int(np.floor(y[0]))
        lambda_param = y[0] - segment_index
        segment = self.track.segments[segment_index]
        theta = segment.gradient(lambda_param)
        segment_scale = self.track.dx_dlambda(segment_index, lambda_param)
        velocity = y[1] * segment_scale


        # Get propulsive force
        throttle_demand = self.control_function(velocity=velocity, theta=theta, segment=theta, lambda_param=y[0])
        propulsive_force = self._propulsive_force(t, y, velocity, throttle_demand, information_dictionary)

        # Resistive forces
        Fw, Fa, Fc, Frr = self.resistive_forces(theta, velocity, segment_index, lambda_param)
        resistive_force = Fw + Fa + Fc + Frr

        # Net force
        net_force = (propulsive_force - resistive_force)
        acceleration = net_force / self.m

        # Calculate d^2lambda_dt^2 correction factor
        correction_factor = y[1] * self.track.d_dx_dlambda_dt(segment_index, [lambda_param, y[1]])

        # Build state vector
        dy_dt = [
            y[1],
            (1 / segment_scale) * (
                        acceleration - correction_factor)
        ]

        information_dictionary['Gradient'] = 100 * (theta / (np.pi / 4))
        information_dictionary['P'] = propulsive_force
        information_dictionary['Frr'] = Frr
        information_dictionary['Fw'] = Fw
        information_dictionary['Fa'] = Fa
        information_dictionary['Fc'] = Fc
        information_dictionary['Velocity (m/s)'] = velocity
        information_dictionary['Throttle Demand'] = throttle_demand
        information_dictionary['Segment Scale'] = segment_scale
        information_dictionary['Net Force'] = net_force
        information_dictionary['Acceleration'] = acceleration
        information_dictionary['Correction Factor'] = correction_factor

        self._further_calculations(y, dy_dt, velocity, throttle_demand, information_dictionary)

        return dy_dt

    def _further_calculations(self, y, dy_dt, velocity, throttle_demand, information_dictionary):
        pass

    def _propulsive_force(self, t_np1, y_n, velocity, demand, information_dictionary):

        return 0

    def resistive_forces(self, theta, V, segment_index, lambda_param):
        """
        :param theta:
        :param V:
        :param segment_index:
        :param lambda_param:
        :return:
        """

        direction_mod = direction_modifier(V)

        Fw = self._weight(theta)
        Fa = direction_mod * self._aerodynamic_drag(V)
        Fc = direction_mod * self._cornering_drag(V, segment_index, lambda_param)
        Frr = direction_mod * self._rolling_resistance()

        return Fw, Fa, Fc, Frr

    def _weight(self, theta):
        """
        :param theta:
        :return:
        """

        return self.mg * np.sin(theta)

    def _aerodynamic_drag(self, V):
        """
        :param segment_index:
        :param lambda_param:
        :param V:
        :return:
        """

        return self.aero_force * V * V

    def _rolling_resistance(self):
        """
        :param V:
        :param theta:
        :return:
        """

        return self.rolling_resistance_force

    def _cornering_drag(self, V, segment_index, lambda_param, alpha_deg=1):
        """
        :return:
        """

        # Assume max alpha = 3 deg
        alpha = alpha_deg * (np.pi / 180)

        # Track radius of curvature
        segment = self.track.segments[segment_index]
        R = segment.horizontal_radius_of_curvature(lambda_param)    # Longest resistive force calculation

        # Centripetal force
        Fy = (self.m * V * V) / (R)

        return Fy * np.tan(alpha)

def direction_modifier(V):
    """
    :param V:
    :return:
    """
    if V > 0:
        return 1

    elif V < 0:
        return -1

    else:
        return 0