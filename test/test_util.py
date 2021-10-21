from pytest import approx

from naaims.util import *


def test_Coord():
    assert Coord(1.1, 2.2) == (1.1, 2.2)


def test_SpeedUpdate():
    assert SpeedUpdate(1.1, 2.2) == (1.1, 2.2)


def test_t_to_v():
    assert t_to_v(1, 2, 10) == 4.5
    assert t_to_v(10, -2, 1) == 4.5
    assert t_to_v(5.5, .5, 6.6) == approx(1.1/.5)
    assert t_to_v(6.6, -.5, 5.5) == approx(1.1/.5)


def test_x_constant_a():
    assert x_over_constant_a(10, 1, 2) == 22
    assert x_over_constant_a(10, -1, 2) == 18
    assert x_over_constant_a(10, 1, 1.1) == approx(11+1.21/2)
    assert x_over_constant_a(10, -1, 1.1) == approx(11-1.21/2)


def test_ff_exit():
    assert free_flow_exit(0, 1, 10, 10, 50, 50) == (10, 10)
    assert free_flow_exit(0, 1, 10, 10, 50, 10) == approx(
        (4.47213595499958, 4.47213595499958))
    assert free_flow_exit(0, 1, 10, 10, 50, 60) == (11, 10)
    t_odd = 5**.5-1
    assert free_flow_exit(1, 1, 3, 2, 4, 2) == (t_odd, 1+t_odd)
    assert free_flow_exit(1, 1, 3, 2, 4, 7) == (3, 3)
