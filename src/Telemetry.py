from abc import ABC, abstractmethod
from dataclasses import dataclass, replace
from math import atan2, degrees, hypot
from time import monotonic
from typing import override

from krpc.client import Client
from krpc.services.spacecenter import (
    Flight,
    ReferenceFrame,
    Resources,
    Vessel,
    VesselSituation,
)
from krpc.stream import Stream

from src.kmath import Derivative, LowPass


@dataclass(frozen=True)
class TelemetryFrame:
    time: float
    dt: float
    sample_age: float

    on_ground: bool

    latitude: float
    longitude: float
    altitude_msl: float
    altitude_agl: float
    terrain_height: float
    gear_agl: float

    pitch: float
    roll: float
    heading: float
    p: float
    q: float
    r: float

    tas: float
    vertical_speed: float
    track: float
    flight_path_angle: float
    tas_dot: float

    dynamic_pressure: float
    density: float
    mach: float
    angle_of_attack: float
    sideslip: float
    load_factor: float

    mass: float
    thrust: float
    available_thrust: float
    drag: float

    spec_energy: float
    spec_energy_rate: float
    energy_balance: float
    energy_balance_rate: float

    fuel: float


class TelemetryProvider(ABC):
    @abstractmethod
    def get_telemetry(self) -> TelemetryFrame: ...


GROUND_SITUATIONS = {VesselSituation.landed, VesselSituation.splashed, VesselSituation.pre_launch}
TAS_DOT_TAU = 0.5


class KRPCTelemetryProvider(TelemetryProvider):
    def __init__(self, conn: Client, vessel: Vessel):
        self._conn: Client = conn
        self._vessel: Vessel = vessel

        body = vessel.orbit.body
        HybridFrame = conn.space_center.ReferenceFrame.create_hybrid
        surface_frame: ReferenceFrame = HybridFrame(position=body.reference_frame, rotation=vessel.surface_reference_frame)
        body_frame: ReferenceFrame = HybridFrame(position=body.reference_frame, rotation=vessel.reference_frame)

        flight: Flight = vessel.flight(surface_frame)
        resources: Resources = vessel.resources

        self._gravity: float = body.surface_gravity
        self._gear_height: float = vessel.bounding_box(vessel.reference_frame).max[2]

        self._time: Stream = conn.add_stream(getattr, conn.space_center, "ut")
        self._situation: Stream = conn.add_stream(getattr, vessel, "situation")

        self._latitude: Stream = conn.add_stream(getattr, flight, "latitude")
        self._longitude: Stream = conn.add_stream(getattr, flight, "longitude")
        self._altitude_msl: Stream = conn.add_stream(getattr, flight, "mean_altitude")
        self._altitude_agl: Stream = conn.add_stream(getattr, flight, "surface_altitude")
        self._terrain_height: Stream = conn.add_stream(getattr, flight, "elevation")

        self._pitch: Stream = conn.add_stream(getattr, flight, "pitch")
        self._roll: Stream = conn.add_stream(getattr, flight, "roll")
        self._heading: Stream = conn.add_stream(getattr, flight, "heading")
        self._angular_velocity: Stream = conn.add_stream(vessel.angular_velocity, body_frame)

        self._tas: Stream = conn.add_stream(getattr, flight, "true_air_speed")
        self._vertical_speed: Stream = conn.add_stream(getattr, flight, "vertical_speed")
        self._velocity: Stream = conn.add_stream(getattr, flight, "velocity")
        self._tas_derivative: Derivative = Derivative()
        self._tas_dot_filter: LowPass = LowPass(TAS_DOT_TAU)

        self._dynamic_pressure: Stream = conn.add_stream(getattr, flight, "dynamic_pressure")
        self._density: Stream = conn.add_stream(getattr, flight, "atmosphere_density")
        self._mach: Stream = conn.add_stream(getattr, flight, "mach")
        self._angle_of_attack: Stream = conn.add_stream(getattr, flight, "angle_of_attack")
        self._sideslip: Stream = conn.add_stream(getattr, flight, "sideslip_angle")
        self._load_factor: Stream = conn.add_stream(getattr, flight, "g_force")

        self._mass: Stream = conn.add_stream(getattr, vessel, "mass")
        self._thrust: Stream = conn.add_stream(getattr, vessel, "thrust")
        self._available_thrust: Stream = conn.add_stream(getattr, vessel, "available_thrust")
        self._drag: Stream = conn.add_stream(getattr, flight, "drag")

        self._fuel: Stream = conn.add_stream(resources.amount, "LiquidFuel")

        self._last_frame: TelemetryFrame | None = None
        self._last_sample_wall: float = 0.0

    @override
    def get_telemetry(self) -> TelemetryFrame:
        time: float = self._time()
        now: float = monotonic()

        if self._last_frame is not None and time == self._last_frame.time:
            return replace(self._last_frame, sample_age=now - self._last_sample_wall)

        dt: float = 0.0 if self._last_frame is None else time - self._last_frame.time
        self._last_sample_wall = now

        altitude_msl: float = self._altitude_msl()
        altitude_agl: float = self._altitude_agl()

        w_right, w_forward, w_down = self._angular_velocity()

        tas: float = self._tas()
        vertical_speed: float = self._vertical_speed()
        v_up, v_north, v_east = self._velocity()
        tas_dot: float = self._tas_dot_filter((time, self._tas_derivative((time, tas))))

        potential: float = altitude_msl
        kinetic: float = tas**2 / (2 * self._gravity)
        potential_rate: float = vertical_speed
        kinetic_rate: float = tas * tas_dot / self._gravity

        frame = TelemetryFrame(
            time=time,
            dt=dt,
            sample_age=0.0,

            on_ground=self._situation() in GROUND_SITUATIONS,

            latitude=self._latitude(),
            longitude=self._longitude(),
            altitude_msl=altitude_msl,
            altitude_agl=altitude_agl,
            terrain_height=self._terrain_height(),
            gear_agl=altitude_agl - self._gear_height,

            pitch=self._pitch(),
            roll=self._roll(),
            heading=self._heading(),
            p=-degrees(w_forward),
            q=-degrees(w_right),
            r=-degrees(w_down),

            tas=tas,
            vertical_speed=vertical_speed,
            track=degrees(atan2(v_east, v_north)) % 360,
            flight_path_angle=degrees(atan2(v_up, hypot(v_north, v_east))),
            tas_dot=tas_dot,

            dynamic_pressure=self._dynamic_pressure(),
            density=self._density(),
            mach=self._mach(),
            angle_of_attack=self._angle_of_attack(),
            sideslip=self._sideslip(),
            load_factor=self._load_factor(),

            mass=self._mass(),
            thrust=self._thrust(),
            available_thrust=self._available_thrust(),
            drag=hypot(*self._drag()),

            spec_energy=potential + kinetic,
            spec_energy_rate=potential_rate + kinetic_rate,
            energy_balance=potential - kinetic,
            energy_balance_rate=potential_rate - kinetic_rate,

            fuel=self._fuel(),
        )

        self._last_frame = frame
        return frame
