
def clamp_angle(min: float, max: float, angle: float) -> float:
    if abs(min - max) != 360:
        raise ValueError("clamp_angle needs 360deg to clamp")
    
    if angle > max: return angle - 360
    elif angle < min: return angle + 360
    else: return angle

