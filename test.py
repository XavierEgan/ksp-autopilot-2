from AutopilotContext import AutoPilotContext, Telemetry, BlockControls, Control, FlightParams
from ControlBlocks import Mode, ControlChain, SetSetpointBlock, YawDamperBlock, RollControlBlock, PitchControlBlock, SpeedControlBlock, AltitudeControlBlock, HeadingControlBlock, VerticalVelocityControlBlock
import time

class Test:
    context: AutoPilotContext

    def __init__(self) -> None:
        self.reset_speed = 600
        self.reset_altitude = 3000

        self.context = AutoPilotContext(
            Telemetry(),
            BlockControls(),
            Control(), 
            FlightParams()
        )

        self.reset_mode = Mode("Reset Mode", [
            ControlChain([SetSetpointBlock(self.context, 90), HeadingControlBlock(self.context), RollControlBlock(self.context)]),
            ControlChain([SetSetpointBlock(self.context, self.reset_altitude), AltitudeControlBlock(self.context), VerticalVelocityControlBlock(self.context), PitchControlBlock(self.context)]),
            ControlChain([SetSetpointBlock(self.context, self.reset_speed), SpeedControlBlock(self.context)]),
            ControlChain([YawDamperBlock(self.context)]),
            ]
        )
    
    def reset(self):
        previous_time = time.time()

        while abs(self.context.telemetry.get_altitude() - self.reset_altitude) > 5 or abs(self.context.telemetry.get_speed() - self.reset_speed) > 1 or True:
            now = time.time()
            dt = now - previous_time
            previous_time = now

            self.reset_mode.update(dt)

if __name__ == "__main__":
    test = Test()

    test.reset()