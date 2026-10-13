import math

import pytest

from drawbot import kinematics as kin
from drawbot import shapes


@pytest.mark.parametrize('elbow_up', [False, True])
def test_inverse_round_trips_over_the_paper(elbow_up):
    for i in range(11):
        for j in range(11):
            x = kin.PAPER_X[0] + (kin.PAPER_X[1] - kin.PAPER_X[0]) * i / 10
            y = kin.PAPER_Y[0] + (kin.PAPER_Y[1] - kin.PAPER_Y[0]) * j / 10
            q = kin.inverse(x, y, elbow_up)
            assert q is not None, f'({x}, {y}) should be reachable'
            fx, fy = kin.forward(*q)
            assert math.isclose(fx, x, abs_tol=1e-9)
            assert math.isclose(fy, y, abs_tol=1e-9)


def test_out_of_reach_is_rejected():
    assert kin.inverse(kin.L1 + kin.L2 + 0.01, 0.0) is None
    assert kin.inverse(0.0, 0.0) is None


def test_quantize_snaps_to_microsteps():
    step = 2 * math.pi / (200 * 16)
    assert math.isclose(kin.quantize(step * 10.4), step * 10)
    assert kin.quantize(0.123, microsteps=0) == 0.123


def test_demo_fits_on_the_paper():
    for stroke in shapes.demo():
        for x, y in stroke:
            assert kin.PAPER_X[0] <= x <= kin.PAPER_X[1]
            assert kin.PAPER_Y[0] <= y <= kin.PAPER_Y[1]
