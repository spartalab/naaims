from math import pi

from pytest import raises

from aimsim.vehicles import AutomatedVehicle
from aimsim.util import Coord
import aimsim.shared as SHARED

SHARED.SETTINGS.read()


def test_automated(a: AutomatedVehicle = AutomatedVehicle(0, 0)):

    # position change
    assert a.pos == Coord(0., 0.)
    a.pos = Coord(1., 1.)
    assert a.pos == Coord(1., 1.)

    # velocity change
    assert a.velocity == 0
    a.velocity = 1.5
    assert a.velocity == 1.5
    with raises(ValueError):
        a.velocity = -10.

    # acceleration change
    assert a.acceleration == 0
    a.acceleration = 1.5
    assert a.acceleration == 1.5
    with raises(ValueError):
        a.velocity = 0
        a.acceleration = -10.

    # heading change
    assert a.heading == 0
    a.heading = 1.5
    assert a.heading == 1.5
    with raises(ValueError):
        a.heading = -1.
    with raises(ValueError):
        a.heading = 3*pi
    with raises(ValueError):
        a.heading = 2*pi

    # bools
    assert not a.permission_to_enter_intersection
    a.permission_to_enter_intersection = True
    assert a.permission_to_enter_intersection
    assert not a.has_reservation
    a.has_reservation = True
    assert a.has_reservation

    # # test next movement
    # # not implemented in pathfinder yet
    # a.next_movements()


def test_cloning(a: AutomatedVehicle = AutomatedVehicle(0, 0)):
    a = AutomatedVehicle(0, 0)
    b = a.clone_for_request()
    a.vin == b.vin
