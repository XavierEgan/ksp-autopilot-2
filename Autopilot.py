import enum
from typing import Callable
from ControlBlocks import AltitudeControlBlock, CenterlineFollowBlock, ControlChain, CruiseManagerBlock, PreCruiseManagerBlock, SetSetpointBlock, GroundHeadingBlock, TakeoffRollManagerBlock, Mode, RollControlBlock, PitchControlBlock, SpeedControlBlock, HeadingControlBlock, VerticalVelocityControlBlock, YawDamperBlock, RotationManagerBlock
from AutopilotContext import Runway, LatLong
from AutopilotContext import AutoPilotContext, Control, FlightParams, Telemetry, BlockControls

class FlightState(enum.Enum):
    TAKEOFF_ROLL = 1
    ROTATION = 2
    PRE_CRUISE = 3
    CRUISE = 4
    POST_CRUISE = 5
    DESCENT = 6
    FINAL = 7
    FLARE = 8
    ROLLOUT = 9

class FlightStateMachine:
    def __init__(self, context: AutoPilotContext, mode: Mode, should_transition_fn: Callable[[], FlightState | None]):
        self.context = context
        self.mode = mode
        self.should_transition_fn = should_transition_fn
    
    def update(self, dt: float):
        self.mode.update(dt)

    def on_enter(self):
        self.mode.on_start()
    
    def check_transition(self) -> FlightState | None:
        return self.should_transition_fn()

def should_transition_to_rotation(context: AutoPilotContext) -> FlightState | None:
    if context.telemetry.get_speed() > context.flight_params.rotation_speed:
        return FlightState.ROTATION
    return None

def should_transition_to_precruise(context: AutoPilotContext) -> FlightState | None:
    if context.telemetry.get_altitude() > context.flight_params.safe_altitude:
        return FlightState.PRE_CRUISE
    return None

def should_transition_to_cruise(context: AutoPilotContext) -> FlightState | None:
    if context.telemetry.get_altitude() > context.flight_params.cruise_altitude * 0.95:
        return FlightState.CRUISE
    return None

if __name__ == "__main__":
    Runway(
        90.40119171142578, 72.05415471561719, LatLong(-0.04860829593900807, -74.72329952750442)
    )

    context = AutoPilotContext(Telemetry(), BlockControls(), Control(), FlightParams())

    rollout_mode = Mode("Takeoff Roll", [
        ControlChain([SetSetpointBlock(context, 0), RollControlBlock(context)]),
        ControlChain([SetSetpointBlock(context, context.telemetry.get_pitch()), PitchControlBlock(context)]),
        ControlChain([SetSetpointBlock(context, context.flight_params.subsonic_cruise_speed), SpeedControlBlock(context)]),
        ControlChain([CenterlineFollowBlock(context, context.flight_params.takeoff_runway), GroundHeadingBlock(context)]),
        ControlChain([TakeoffRollManagerBlock(context)])
        ]
    )

    rotation_mode = Mode("Rotation", [
        ControlChain([SetSetpointBlock(context, context.flight_params.max_pitch), PitchControlBlock(context)]),
        ControlChain([SetSetpointBlock(context, context.flight_params.subsonic_cruise_speed), SpeedControlBlock(context)]),
        ControlChain([SetSetpointBlock(context, context.flight_params.takeoff_runway.heading), HeadingControlBlock(context), RollControlBlock(context)]),
        ControlChain([YawDamperBlock(context)]),
        ControlChain([RotationManagerBlock(context)])
    ])

    # climb to cruise altitude the switch to cruise (which is supersonic)
    pre_cruise_mode = Mode("Pre-Cruise", [
        ControlChain([SetSetpointBlock(context, context.flight_params.cruise_altitude), AltitudeControlBlock(context), VerticalVelocityControlBlock(context), PitchControlBlock(context)]),
        ControlChain([SetSetpointBlock(context, context.flight_params.subsonic_cruise_speed), SpeedControlBlock(context)]),
        ControlChain([SetSetpointBlock(context, context.flight_params.takeoff_runway.heading), HeadingControlBlock(context), RollControlBlock(context)]),
        ControlChain([YawDamperBlock(context)]),
        ControlChain([PreCruiseManagerBlock(context)])
    ])

    cruise_mode = Mode("Cruise", [
        ControlChain([SetSetpointBlock(context, context.flight_params.cruise_altitude), AltitudeControlBlock(context), VerticalVelocityControlBlock(context), PitchControlBlock(context)]),
        ControlChain([SetSetpointBlock(context, context.flight_params.supersonic_cruise_speed), SpeedControlBlock(context)]),
        ControlChain([SetSetpointBlock(context, context.flight_params.takeoff_runway.heading), HeadingControlBlock(context), RollControlBlock(context)]),
        ControlChain([YawDamperBlock(context)]),
        ControlChain([CruiseManagerBlock(context)])
    ])

    rollout = FlightStateMachine(context, rollout_mode, lambda: should_transition_to_rotation(context))
    rotation = FlightStateMachine(context, rotation_mode, lambda: should_transition_to_precruise(context))
    pre_cruise = FlightStateMachine(context, pre_cruise_mode, lambda: should_transition_to_cruise(context))


    state_machines = {
        FlightState.TAKEOFF_ROLL: rollout,
        FlightState.ROTATION: rotation,
        FlightState.PRE_CRUISE: pre_cruise,
        FlightState.CRUISE: FlightStateMachine(context, cruise_mode, lambda: None)
    }

    state = FlightState.TAKEOFF_ROLL
    state_machine = state_machines[state]
    state_machine.on_enter()

    previous_time = None

    while True:
        current_time = context.telemetry.get_time()
        if previous_time is None:
            previous_time = current_time
            continue

        dt = current_time - previous_time
        previous_time = current_time

        state_machine.update(dt)

        new_state = state_machine.check_transition()
        if new_state is not None:
            print(f"Transitioning from {state} to {new_state}")
            state = new_state
            state_machine = state_machines[state]
            state_machine.on_enter()
