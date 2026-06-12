from abc import ABC, abstractmethod
import time
from AutopilotContext import AutoPilotContext, BlockControls, Control, FlightParams, Runway, Telemetry
from Math import clamp, circular_error

class Mode:
    def __init__(self, name: str, blocks: list['ControlChain']):
        self.name = name
        self.control_chains: list['ControlChain'] = blocks
    
    def update(self, dt: float) -> None:
        for chain in self.control_chains:
            chain.update(dt)
        
    def on_start(self) -> None:
        for chain in self.control_chains:
            chain.on_start()

class ControlChain:
    def __init__(self, blocks: list['ControlBlock']):
        self.blocks = blocks

        self.has_started = False
    
    def update(self, dt: float) -> None:
        if not self.has_started:
            for block in self.blocks:
                block.on_start()
            self.has_started = True

        for i, block in enumerate(self.blocks):
            below_block = self.blocks[i + 1] if i + 1 < len(self.blocks) else None
            block.update(dt, below_block)
    
    def on_start(self) -> None:
        for block in self.blocks:
            block.on_start()

# each control block is supposed to be above one or no other type of control block
class ControlBlock(ABC):
    context: AutoPilotContext
    below_block: 'ControlBlock | None' # the block directly below this one (lower level)
    setpoint: float

    def __init__(self, context: AutoPilotContext):
        self.context = context

    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        pass

    def on_start(self) -> None:
        pass


class SpeedControlBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0
    
    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        d = self.setpoint - self.context.telemetry.get_speed()

        a = self.context.controls.speed.update(d, dt)

        control = a + self.context.telemetry.get_predictive_thrust()

        self.context.control.set_throttle(control)

class PitchControlBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0.0
    
    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        d = self.setpoint - self.context.telemetry.get_pitch()

        control = self.context.controls.pitch.update(d, dt)

        self.context.control.set_pitch(control)

class RollControlBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0.0
    
    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        d = self.setpoint - self.context.telemetry.get_roll()

        control = self.context.controls.roll.update(d, dt)

        self.context.control.set_roll(control)

class HeadingControlBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0.0
    
    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        d = circular_error(self.setpoint, self.context.telemetry.get_heading())

        control = self.context.controls.heading.update(d, dt) * self.context.flight_params.max_bank

        if below_block is not None:
            below_block.setpoint = control

class VerticalSpeedControlBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0.0
    
    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        d = self.setpoint - self.context.telemetry.get_vertical_velocity()

        control = self.context.controls.vertical_velocity.update(d, dt) * self.context.flight_params.max_pitch

        if below_block is not None:
            below_block.setpoint = control

class AltitudeControlBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0
    
    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        d = self.setpoint - self.context.telemetry.get_altitude()

        control = self.context.controls.altitude.update(d, dt) * self.context.flight_params.max_pitch
        
        if below_block is not None:
            below_block.setpoint = control

class YawDamperBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0.0
    
    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        d = self.context.telemetry.get_yaw_rate()

        control = self.context.controls.yaw_damp * d

        self.context.control.set_yaw(control)

class SetSetpointBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext, setpoint: float):
        super().__init__(context)

        self.setpoint = setpoint
    
    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        if below_block is not None:
            below_block.setpoint = self.setpoint

class GroundHeadingBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)
        self.setpoint = 0.0
    
    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        d = circular_error(self.setpoint, self.context.telemetry.get_heading())

        control = self.context.controls.ground_heading.update(d, dt)
        
        self.context.control.set_steering(control)
        self.context.control.set_yaw(control)

class CenterlineFollowBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext, runway: Runway):
        super().__init__(context)
        self.runway = runway

        self.setpoint = 0.0
    
    def update(self, dt: float, below_block: 'ControlBlock | None') -> None:
        d = self.context.telemetry.get_cross_track_error(self.runway)

        control = self.runway.heading - self.context.controls.centerline_follow.update(d, dt)

        if below_block is not None:
            below_block.setpoint = control
        
class TakeoffRollManagerBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0.0
    
    def on_start(self):
        self.context.control.release_brakes()
        self.context.control.extend_gear()
        self.context.control.extend_flaps()
        self.context.control.enable_main_engine()
        self.context.control.disable_supersonic_engine()

class RotationManagerBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0.0
        
    def on_start(self):
        pass

class PreCruiseManagerBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0.0
        
    def on_start(self):
        self.context.control.retract_flaps()
        self.context.control.retract_gear()

class CruiseManagerBlock(ControlBlock):
    def __init__(self, context: AutoPilotContext):
        super().__init__(context)

        self.setpoint = 0.0
        
    def on_start(self):
        self.context.control.enable_supersonic_engine()

