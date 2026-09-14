import math
from dataclasses import dataclass
import sys
from typing import Callable, cast

import krpc
from krpc.services.spacecenter import SpaceCenter, Vessel

from Math import PID, AxisController, Derivative, LatLong, Runway, clamp

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


class TelemetryREFERENCEOLDDONTUSE:
    def __init__(self):
        self.conn = krpc.connect(name="Full AutoPilot")

        space_center = self.conn.space_center

        if space_center is None:
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

    def get_position(self) -> LatLong:
        return LatLong(self.vessel.flight().latitude, self.vessel.flight().longitude)

    def get_yaw_rate(self) -> float:
        t = self.get_time()
        dt = (t - self._yaw_prev_time) if self._yaw_prev_time is not None else 0.0
        self._yaw_prev_time = t
        self.yaw_rate.set_next(
            self.vessel.flight(self.vessel.surface_reference_frame).sideslip_angle, dt
        )
        return self.yaw_rate.get_current_derivative(dt)

    def get_density(self) -> float:
        return self.vessel.flight(
            self.vessel.orbit.body.reference_frame
        ).atmosphere_density


@dataclass
class TelemetryFrame:
    """
    Represents the telemetry data retrieved from a single frame in ksp
    """
    time: float = 0

    p: float = 0 # body roll rate (angular velocity around roll axis)
    q: float = 0 # body pitch rate (angular velocity around pitch axis)
    r: float = 0 # body yaw rate (angular velocity around yaw axis)

    phi: float = 0 # bank angle (rad)
    beta: float = 0 # sideslip angle (rad, positive = right side of aircraft hits air)

    psi: float = 0 # heading (rad, 0 = north, clockwise)
    psi_dot: float = 0 # heading rate (rad/s)

    V: float = 0 # true airspeed
    vertical_velocity: float = 0 # vertical speed

    altitude: float = 0 # altitude above sea level
    altitude_ground: float = 0 # altitude above ground level

class Telemetry:
    def __init__(self):
        self.conn = krpc.connect(name="AutoPilot")

        space_center = self.conn.space_center

        if space_center is None:
            print("could not find space center")
            sys.exit()

        self.space_center: SpaceCenter = space_center

        self.vessel: Vessel = self.space_center.active_vessel

        self.previous_frame: TelemetryFrame | None = None

        # set up streams
        self.time_stream = cast(
            Callable[[], float], self.conn.add_stream(getattr, self.space_center, "ut")
        )
        body_frame = self.vessel.orbit.body.reference_frame

        self.angular_velocity_stream = cast(
            Callable[[], Vec3], self.conn.add_stream(self.vessel.angular_velocity, body_frame)
        )
        self.rotation_stream = cast(
            Callable[[], Quat], self.conn.add_stream(self.vessel.rotation, body_frame)
        )
        self.velocity_stream = cast(
            Callable[[], Vec3], self.conn.add_stream(self.vessel.velocity, body_frame)
        )

        # pitch/heading/roll are relative to the horizon
        surface_flight = self.vessel.flight(self.vessel.surface_reference_frame)
        self.roll_stream = cast(
            Callable[[], float], self.conn.add_stream(getattr, surface_flight, "roll")
        )
        self.heading_stream = cast(
            Callable[[], float], self.conn.add_stream(getattr, surface_flight, "heading")
        )

    @staticmethod
    def _to_vessel_axes(v: Vec3, rotation: Quat) -> Vec3:
        """rotates a body frame vector into vessel axes (x right, y forward, z down)"""
        x, y, z, s = rotation

        t = (
            2 * (y * v[2] - z * v[1]),
            2 * (z * v[0] - x * v[2]),
            2 * (x * v[1] - y * v[0]),
        )
        return (
            v[0] - s * t[0] + (y * t[2] - z * t[1]),
            v[1] - s * t[1] + (z * t[0] - x * t[2]),
            v[2] - s * t[2] + (x * t[1] - y * t[0]),
        )

    def _body_rates(self, rotation: Quat) -> Vec3:
        """angular velocity in vessel axes, (p, q, r) in rad/s"""
        av = self._to_vessel_axes(self.angular_velocity_stream(), rotation)

        # negative signs are emperical from in game
        return -av[1], av[0], av[2]

    def _sideslip(self, rotation: Quat) -> float:
        """sideslip angle in rad, positive when moving to the right (right side hits air)"""
        v = self._to_vessel_axes(self.velocity_stream(), rotation)
        speed = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)

        # sideslip is meaningless when basically stationary
        if speed < 1.0:
            return 0.0

        return math.asin(clamp(v[0] / speed, -1.0, 1.0))

    def _heading_rate(self, time: float, psi: float) -> float:
        """heading rate in rad/s, finite difference against the previous frame"""
        prev = self.previous_frame
        if prev is None or time <= prev.time:
            return 0.0

        # wrap to [-pi, pi) so crossing north doesnt spike
        dpsi = (psi - prev.psi + math.pi) % math.tau - math.pi
        return dpsi / (time - prev.time)

    def is_frame_ready(self) -> bool:
        return self.previous_frame is None or self.previous_frame.time != self.time_stream()

    def get_frame(self) -> TelemetryFrame:
        if not self.is_frame_ready():
            assert self.previous_frame is not None
            return self.previous_frame

        time = self.time_stream()
        rotation = self.rotation_stream()

        p, q, r = self._body_rates(rotation)
        psi = math.radians(self.heading_stream())

        frame = TelemetryFrame(
            time=time,
            p=p,
            q=q,
            r=r,
            phi=math.radians(self.roll_stream()),
            beta=self._sideslip(rotation),
            psi=psi,
            psi_dot=self._heading_rate(time, psi),
        )

        self.previous_frame = frame
        return frame

if __name__ == "__main__":
    telem = Telemetry()
    while True:
        if not telem.is_frame_ready():
            continue

        frame = telem.get_frame()
        print(
            f"p {frame.p: .2f} q {frame.q: .2f} r {frame.r: .2f} | "
            f"phi {math.degrees(frame.phi): 6.1f} beta {math.degrees(frame.beta): 5.1f} | "
            f"psi {math.degrees(frame.psi): 5.1f} psi_dot {math.degrees(frame.psi_dot): 5.1f}"
        )