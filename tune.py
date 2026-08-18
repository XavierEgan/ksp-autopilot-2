from AutopilotContext import AutoPilotContext, Telemetry, BlockControls, Control, FlightParams
from ControlBlocks import Mode, ControlChain, SetSetpointBlock, RollControlBlock, PitchControlBlock, SpeedControlBlock, YawDamperBlock
import time
import random

change_desired_pointing_time = 15

speed = 1000
roll_range = 0
pitch_min = -5
pitch_max = 20

context = AutoPilotContext(Telemetry(), BlockControls(), Control(), FlightParams())

tune_pitch_roll = Mode("Tune Pitch/Roll", [
    ControlChain([SetSetpointBlock(context, 0), RollControlBlock(context)]),
    ControlChain([SetSetpointBlock(context, 2), PitchControlBlock(context)]),
    ControlChain([SetSetpointBlock(context, speed), SpeedControlBlock(context)]),
    ControlChain([YawDamperBlock(context)]),
    ]
)

tune_altitude_heading = Mode("Tune Altitude/Heading", [
    ControlChain([SetSetpointBlock(context, 0), RollControlBlock(context)]),
    ControlChain([SetSetpointBlock(context, context.telemetry.get_pitch()), PitchControlBlock(context)]),
    ControlChain([SetSetpointBlock(context, speed), SpeedControlBlock(context)]),
    ControlChain([YawDamperBlock(context)]),
    ]
)

previous_time = context.telemetry.get_time()

timer = 0

while True:
    current_time = context.telemetry.get_time()
    dt = current_time - previous_time
    previous_time = current_time

    old_timer = timer
    timer = (timer + dt) % change_desired_pointing_time
    if timer < old_timer:
        new_pitch = random.randint(pitch_min, pitch_max)
        new_roll = random.randint(-roll_range, roll_range)

        tune_pitch_roll.control_chains[0].blocks[0].setpoint = new_roll
        tune_pitch_roll.control_chains[1].blocks[0].setpoint = new_pitch
    
    # print(f"roll error: {tune_pitch_roll.control_chains[0].blocks[0].setpoint - context.telemetry.get_roll(): 4.2}, pitch error: {tune_pitch_roll.control_chains[1].blocks[0].setpoint - context.telemetry.get_pitch(): 4.2}")

    tune_pitch_roll.update(dt)

    time.sleep(0.02)