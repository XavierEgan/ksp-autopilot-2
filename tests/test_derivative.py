import pytest
from kmath import Derivative


def test_positive_derivative() -> None:
    der: Derivative = Derivative()

    assert(der((0, 0)) == 0)
    assert(der((1, 1)) == 1)
    assert(der((2, 1.5)) == 0.5)

def test_negative_y() -> None:
    der: Derivative = Derivative()

    assert(der((0, 0)) == 0)
    assert(der((1, -1)) == -1)