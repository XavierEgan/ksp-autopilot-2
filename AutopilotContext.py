import math

import krpc
from krpc.services.spacecenter import Vessel, SpaceCenter
from Math import AxisController, Derivative, clamp, LatLong, Runway, PID

class Telemetry:
    def __init__(self):
        self.conn = krpc.connect(name = 'Full AutoPilot')

        space_center = self.conn.space_center

        if (space_center is None):
            print("Could not find space center.")
            quit()
        
        self.space_center: SpaceCenter = space_center
        
        self.vessel: Vessel = self.space_center.active_vessel

        self.yaw_rate = Derivative()
        self._yaw_prev_time: float | None = None

    def get_speed(self) -> float:
        return self.vessel.flight(self.vessel.orbit.body.reference_frame).speed
    
    def get_vertical_velocity(self) -> float:
        return self.vessel.flight(self.vessel.orbit.body.reference_frame).vertical_speed
    
    def get_time(self) -> float:
        return self.space_center.ut

    def get_pitch(self) -> float:
        return self.vessel.flight(self.vessel.surface_reference_frame).pitch

    def get_roll(self) -> float:
        return self.vessel.flight(self.vessel.surface_reference_frame).roll
    
    def get_heading(self) -> float:
        return self.vessel.flight(self.vessel.surface_reference_frame).heading
    
    def get_altitude(self) -> float:
        return self.vessel.flight().mean_altitude

    def get_predictive_thrust(self) -> float:
        drag_vec = self.vessel.flight(self.vessel.orbit.body.reference_frame).drag
        drag_magnitude = (drag_vec[0]**2 + drag_vec[1]**2 + drag_vec[2]**2) ** 0.5
        max_producable_thrust = self.vessel.available_thrust
        predictive_value: float = clamp(drag_magnitude / max_producable_thrust if max_producable_thrust != 0 else 0, 0, 1)

        return predictive_value
    
    def get_position(self) -> LatLong:
        return LatLong(self.vessel.flight().latitude, self.vessel.flight().longitude)
    
    def get_cross_track_error(self, runway: Runway, body_radius: float = 600_000) -> float:
        """
        Cross-track error from the extended runway centerline, in meters.
        Positive = right of centerline, negative = left.
        """
        position = self.get_position()

        lat1 = math.radians(runway.start_position.latitude)
        lon1 = math.radians(runway.start_position.longitude)
        lat2 = math.radians(position.latitude)
        lon2 = math.radians(position.longitude)

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        # Angular distance: runway threshold → aircraft
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        d13 = 2 * math.asin(math.sqrt(a))

        # Bearing: runway threshold → aircraft
        x = math.cos(lat2) * math.sin(dlon)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing_to_aircraft = math.atan2(x, y)

        runway_bearing = math.radians(runway.heading)

        return math.asin(math.sin(d13) * math.sin(bearing_to_aircraft - runway_bearing)) * body_radius

    def get_yaw_rate(self) -> float:
        t = self.get_time()
        dt = (t - self._yaw_prev_time) if self._yaw_prev_time is not None else 0.0
        self._yaw_prev_time = t
        self.yaw_rate.set_next(self.vessel.flight(self.vessel.surface_reference_frame).sideslip_angle, dt)
        return self.yaw_rate.get_current_derivative(dt)
    
    def get_density(self) -> float:
        return self.vessel.flight(self.vessel.orbit.body.reference_frame).atmosphere_density

class Control:
    def __init__(self):
        self.conn = krpc.connect(name = 'Full AutoPilot')

        if (self.conn.space_center is None):
            print("Could not find space center.")
            quit()
        
        self.vessel: Vessel = self.conn.space_center.active_vessel
    
    def set_throttle(self, value: float) -> None:
        self.vessel.control.throttle = value
    
    def set_pitch(self, value: float) -> None:
        self.vessel.control.pitch = value
    
    def set_roll(self, value: float) -> None:
        self.vessel.control.roll = value
    
    def set_yaw(self, value: float) -> None:
        self.vessel.control.yaw = value
    
    def set_steering(self, value: float) -> None:
        self.vessel.control.wheel_steering = -value

    
    def extend_gear(self) -> None:
        self.vessel.control.gear = True

    def retract_gear(self) -> None:
        self.vessel.control.gear = False
    
    def get_gear(self) -> bool:
        return self.vessel.control.gear
    
    def set_brakes(self) -> None:
        self.vessel.control.brakes = True
    
    def release_brakes(self) -> None:
        self.vessel.control.brakes = False
    
    def get_brakes(self) -> bool:
        return self.vessel.control.brakes

    def retract_flaps(self) -> None:
        self.vessel.control.set_action_group(1, False)

    def extend_flaps(self) -> None:
        self.vessel.control.set_action_group(1, True)

    def get_flaps(self) -> bool:
        return self.vessel.control.get_action_group(1)
    
    def enable_main_engine(self) -> None:
        self.vessel.control.set_action_group(2, True)
    
    def disable_main_engine(self) -> None:
        self.vessel.control.set_action_group(2, False)
    
    def get_main_engine(self) -> bool:
        return self.vessel.control.get_action_group(2)
    
    def enable_supersonic_engine(self) -> None:
        self.vessel.control.set_action_group(3, True)
    
    def disable_supersonic_engine(self) -> None:
        self.vessel.control.set_action_group(3, False)
    
    def get_supersonic_engine(self) -> bool:
        return self.vessel.control.get_action_group(3)

    def enable_thrust_reversers(self) -> None:
        self.vessel.control.set_action_group(4, True)

    def disable_thrust_reversers(self) -> None:
        self.vessel.control.set_action_group(4, False)

    def get_thrust_reversers(self) -> bool:
        return self.vessel.control.get_action_group(4)


class BlockControls:
    def __init__(self):
        tuning_p = 20000

        self.speed = PID(kp=1/20, ki=0, kd=1/20, kaw=0, min_output=-1, max_output=1)
        self.pitch = AxisController(PID(kp=1/10, ki=1/10000, kd=1/30, kaw=1, min_output=-0.25, max_output=1, log=False), context=None, q_ref=tuning_p)
        self.roll = AxisController(PID(kp=1/20, ki=0, kd=1/30, kaw=0, min_output=-1, max_output=1), context=None, q_ref=tuning_p)
        self.ground_heading = AxisController(PID(kp=1/50, ki=0, kd=1/100, kaw=0, min_output=-1, max_output=1), context=None, q_ref=tuning_p)

        self.vertical_velocity = AxisController(PID(kp=1/500, ki=1/200, kd=1/1000, kaw=1/1000, min_output=-1, max_output=1, log=True), context=None, q_ref=tuning_p)
        self.heading = AxisController(PID(kp=1/5, ki=0, kd=1/10, kaw=1/100, min_output=-1, max_output=1), context=None, q_ref=tuning_p)

        self.altitude = AxisController(PID(kp=1/1000, ki=0, kd=0, kaw=0, min_output=-0.2, max_output=1, log=False), context=None, q_ref=tuning_p)

        self.centerline_follow = AxisController(PID(kp=1/10, ki=0, kd=0, kaw=0, min_output=-10, max_output=10), context=None, q_ref=tuning_p)

        self.yaw_damp = 0.1

    def set_context(self, context: 'AutoPilotContext') -> None:
        self.pitch.context = context
        self.roll.context = context
        self.ground_heading.context = context
        self.vertical_velocity.context = context
        self.heading.context = context
        self.altitude.context = context
        self.centerline_follow.context = context

class FlightParams:
    def __init__(self):
        self.rotation_speed = 80.0

        self.cruise_altitude = 6000.0

        self.subsonic_cruise_speed = 200.0
        self.supersonic_cruise_speed = 1000.0

        self.max_vertical_velocity = 20
        self.max_pitch = 15.0
        self.max_bank = 30

        self.safe_altitude = 200.0

        self.takeoff_runway = Runway(
            90.40119171142578, 72.05415471561719, LatLong(-0.04860829593900807, -74.72329952750442)
        )

class AutoPilotContext:
    '''
    Single source of all information the autopilot may need
    '''
    def __init__(self, telemetry: Telemetry, controls: BlockControls, control: Control, flight_params: FlightParams):
        self.telemetry = telemetry
        self.controls = controls
        self.control = control
        self.flight_params = flight_params
        controls.set_context(self)