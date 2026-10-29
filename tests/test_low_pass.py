import pytest
from kmath import LowPass


def test_first_sample_passes_through() -> None:
    lp: LowPass = LowPass(1.0)

    assert(lp((0, 5)) == 5)

def test_step_moves_part_way() -> None:
    lp: LowPass = LowPass(1.0)

    assert(lp((0, 0)) == 0)
    assert(lp((1, 1)) == 0.5)
    assert(lp((2, 1)) == 0.75)

def test_converges_to_constant_input() -> None:
    lp: LowPass = LowPass(0.5)

    lp((0, 0))
    y: float = 0
    for i in range(1, 200):
        y = lp((i * 0.1, 3))

    assert(y == pytest.approx(3))
