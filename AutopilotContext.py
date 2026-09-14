import math

import krpc
from krpc.services.spacecenter import Vessel, SpaceCenter
from Math import AxisController, Derivative, clamp, LatLong, Runway, PID


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