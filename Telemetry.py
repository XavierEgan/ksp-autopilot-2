from dataclasses import dataclass

import krpc
from krpc.client import Client
from krpc.services.spacecenter import Flight, Orbit, ReferenceFrame, SpaceCenter, Vessel
from krpc.stream import Stream


@dataclass
class TelemetryFrame:

    altitude_ground: float
    altitude_asl: float

    velocity_tas: float

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
        self.altitude: Stream = conn.add_stream(getattr, flight, "mean_altitude")
    
    def __call__(self) -> TelemetryFrame:
        return TelemetryFrame()

if __name__ == "__main__":
    pass