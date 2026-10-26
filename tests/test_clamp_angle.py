import pytest
from kmath import clamp_angle

def test_angle_in_range_is_unchanged() -> None:
    assert clamp_angle(-180, 180, 0) == 0

def test_angle_below_range_is_in_range() -> None:
    angle = clamp_angle(-180, 180, -190)
    
    assert angle > -180
    assert angle < 180

def test_angle_below_range_value_is_correct() -> None:
    angle = clamp_angle(-180, 180, -190)

    assert angle == 170