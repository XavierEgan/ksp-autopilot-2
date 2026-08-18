from __future__ import annotations
from typing import TYPE_CHECKING, Literal

if TYPE_CHECKING:
    from AutopilotContext import AutoPilotContext

def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))

def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t

def remap(value: float, from_min: float, from_max: float, to_min: float, to_max: float) -> float:
    t = (value - from_min) / (from_max - from_min)
    return lerp(to_min, to_max, t)

def circular_error(target: float, current: float) -> float:
    error = target - current
    while error > 180:
        error -= 360
    while error < -180:
        error += 360
    return error

class LatLong:
    def __init__(self, latitude: float, longitude: float):
        self.latitude = latitude
        self.longitude = longitude

class Runway:
    def __init__(self, heading: float, altitude: float, start_position: LatLong):
        self.heading = heading
        self.altitude = altitude
        self.start_position = start_position

class Derivative:
    def __init__(self):
        self.prev_value = 0.0
        self.curr_value = 0.0
        self.has_prev = False

    def set_next(self, value: float, dt: float):
        self.prev_value = self.curr_value
        self.curr_value = value
        if not self.has_prev:
            self.prev_value = value
            self.has_prev = True

    def get_current(self):
        return self.curr_value

    def get_current_derivative(self, dt: float):
        if dt <= 0.0:
            return 0.0
        return (self.curr_value - self.prev_value) / dt

class PID:
    # ki and kaw are passed in these units so the tuning numbers stay human-sized
    integral_scale = 1 / 10000

    def __init__(
            self,
            kp: float,
            ki: float,
            kd: float,
            kaw: float,
            min_output: float = -1.0,
            max_output: float = 1.0,
            log: bool = False
        ):

        self.kp = kp
        self.ki = ki * PID.integral_scale
        self.kd = kd
        self.kaw = kaw * PID.integral_scale

        self.min_output = min_output
        self.max_output = max_output

        self.min_acc = min_output * 0.5
        self.max_acc = max_output * 0.5

        self.accum: float = 0.0
        self.smoothing = Derivative()

        self.log = log
    
    @staticmethod
    def from_ziegler_nichols(
        type: Literal["p", "pi", "pd", "pid", "pessen_pid", "some_overshoot", "no_overshoot"],
        Ku: float,
        Tu: float,
        min_output: float = -1.0, 
        max_output: float = 1.0,
        log: bool = False
    ) -> 'PID':
            # Ziegler-Nichols produces true gains, so undo the constructor's
            # integral_scale to stop ki/kaw being scaled down twice
            def entry(kp: float, ki: float, kd: float) -> dict[str, float]:
                kaw = ki / kp if ki != 0.0 else 0.0
                return {
                    "kp": kp,
                    "ki": ki / PID.integral_scale,
                    "kd": kd,
                    "kaw": kaw / PID.integral_scale,
                }

            gains = {
                "p":              entry(0.5 * Ku,  0.0,                          0.0),
                "pi":             entry(0.45 * Ku, 0.45 * Ku / (Tu / 1.2),       0.0),
                "pd":             entry(0.8 * Ku,  0.0,                          0.8 * Ku * (Tu / 8.0)),
                "pid":            entry(0.6 * Ku,  0.6 * Ku / (Tu / 2.0),        0.6 * Ku * (Tu / 8.0)),
                "pessen_pid":     entry(0.7 * Ku,  0.7 * Ku / (Tu / 2.5),        0.7 * Ku * (3.0 * Tu / 20.0)),
                "some_overshoot": entry(0.33 * Ku, 0.33 * Ku / (Tu / 2.0),       0.33 * Ku * (Tu / 3.0)),
                "no_overshoot":   entry(0.2 * Ku,  0.2 * Ku / (Tu / 2.0),        0.2 * Ku * (Tu / 3.0)),
            }

            return PID(min_output=min_output, max_output=max_output, log=log, **gains[type])
    
    def update(self, error: float, dt: float) -> float:
        self.smoothing.set_next(error, dt)
        current_error = self.smoothing.get_current()

        # derivative
        derivative = self.smoothing.get_current_derivative(dt)

        # integral
        self.accum += current_error * dt * self.ki

        raw_output = (self.kp * current_error) + self.accum + (self.kd * derivative)
        output = clamp(raw_output, self.min_output, self.max_output)
        
        # anti-windup
        self.accum += (output - raw_output) * self.kaw * dt

        # clamp windup
        self.accum = clamp(self.accum, self.min_acc, self.max_acc)

        if self.log:
            print(f"Proportional: {current_error * self.kp: <6.2f} | Derivative: {derivative * self.kd: <6.2f} | Integral: {self.accum: <6.2f}")

        return output

    def reset(self):
        self.accum = 0.0

class AxisController:
    def __init__(self, pid: PID, context: AutoPilotContext | None, q_ref: float, q_min: float = 20000.0):
        self.pid = pid
        self.context = context
        self.q_ref = q_ref
        self.q_min = q_min
        self.original_kp = pid.kp
        self.original_kd = pid.kd

    def update(self, error: float, dt: float) -> float:
        if self.context is None:
            raise ValueError("Context must be provided for AxisController to update")

        v = self.context.telemetry.get_speed()
        rho = self.context.telemetry.get_density()
        q = max(0.5 * rho * v * v, self.q_min)

        scale = self.q_ref / q
        self.pid.kp = self.original_kp * scale
        self.pid.kd = self.original_kd * scale

        return self.pid.update(error, dt)