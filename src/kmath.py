
def clamp_angle(min: float, max: float, angle: float) -> float:
    if abs(min - max) != 360:
        raise ValueError("clamp_angle needs 360deg to clamp")
    
    if angle > max: return angle - 360
    elif angle < min: return angle + 360
    else: return angle

class Derivative:
    prev: tuple[float, float] | None
    def __init__(self) -> None:
        self.prev = None
    
    def __call__(self, pc: tuple[float, float]) -> float:
        if self.prev == None:
            self.prev = pc
            return 0.0

        dy: float = pc[1] - self.prev[1]
        dt: float = pc[0] - self.prev[0]

        self.prev = pc

        return dy/dt

class LowPass:
    prev: tuple[float, float] | None
    def __init__(self, tau: float) -> None:
        self.tau: float = tau
        self.prev = None

    def __call__(self, pc: tuple[float, float]) -> float:
        if self.prev is None:
            self.prev = pc
            return pc[1]

        dt: float = pc[0] - self.prev[0]
        alpha: float = dt / (self.tau + dt)
        y: float = self.prev[1] + alpha * (pc[1] - self.prev[1])

        self.prev = (pc[0], y)

        return y
