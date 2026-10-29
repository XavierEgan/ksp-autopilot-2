from dataclasses import dataclass
from turtle import speed

import krpc
from krpc.client import Client
from krpc.services.spacecenter import Flight, Orbit, ReferenceFrame, SpaceCenter, Vessel
from krpc.stream import Stream

from kmath import Derivative


@dataclass
class TelemetryFrame:

    altitude_ground: float
    altitude_asl: float
    altitude_rate: float

    speed_tas: float

    # 0deg = north, 90deg = east
    heading: float
    heading_rate: float

    # +45 = pitching up 45 deg
    pitch: float
    pitch_rate: float

    # +45 = rolling right 45 deg
    roll: float
    roll_rate: float

class Telemetry:
    def __init__(self, conn: Client, vessel: Vessel):
        ref = vessel.orbit.body.reference_frame
        flight: Flight = vessel.flight(ref)

        self.altitude_asl: Stream = conn.add_stream(getattr, flight, "mean_altitude")
        self.altitude_ground: Stream = conn.add_stream(getattr, flight, "surface_altitude")
        self.altitude_der: Derivative = Derivative()

        self.speed_tas: Stream = conn.add_stream(getattr, flight, "speed")
    
    def __call__(self, time: float) -> TelemetryFrame:
        altitude_ground: float = self.altitude_ground()
        altitude_asl: float = self.altitude_asl()
        altitude_rate: float = self.altitude_der((time, altitude_asl))

        speed_tas = self.speed_tas()

        heading: = self.heading()

        pitch = self.pitch()

        roll = self.roll()

        return TelemetryFrame(
            altitude_ground=altitude_ground,
            altitude_asl=altitude_asl,
            altitude_rate=altitude_rate,

        )

if __name__ == "__main__":
    pass