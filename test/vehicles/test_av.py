from math import pi

from pytest import raises, approx

from naaims.vehicles import AutomatedVehicle
from naaims.util import Coord
import naaims.shared as SHARED

SHARED.SETTINGS.load()


def test_position(vehicle: AutomatedVehicle):
    assert vehicle.pos == Coord(0., 0.)
    vehicle.pos = Coord(1., 1.)
    assert vehicle.pos == Coord(1., 1.)


def test_velocity(vehicle: AutomatedVehicle):
    assert vehicle.velocity == 0
    vehicle.velocity = 1.5
    assert vehicle.velocity == 1.5
    with raises(ValueError):
        vehicle.velocity = -10.


def test_acceleration(vehicle: AutomatedVehicle):
    # acceleration change
    assert vehicle.acceleration == 0
    vehicle.acceleration = 1.5
    assert vehicle.acceleration == 1.5
    with raises(ValueError):
        vehicle.velocity = 0
        vehicle.acceleration = -10.


def test_heading(vehicle: AutomatedVehicle):
    assert vehicle.heading == 0
    vehicle.heading = 1.5
    assert vehicle.heading == 1.5
    with raises(ValueError):
        vehicle.heading = -1.
    with raises(ValueError):
        vehicle.heading = 3*pi
    with raises(ValueError):
        vehicle.heading = 2*pi


def test_bools(vehicle: AutomatedVehicle):
    assert not vehicle.permission_to_enter_intersection
    vehicle.permission_to_enter_intersection = True
    assert vehicle.permission_to_enter_intersection
    assert not vehicle.has_reservation
    vehicle.has_reservation = True
    assert vehicle.has_reservation

# def test_next_movement in test_integration because it depends on several
# different components working together.


def test_outline(vehicle: AutomatedVehicle):
    vehicle.pos = Coord(0, 0)
    vehicle.heading = 0
    rectangle_right = vehicle.get_outline()
    assert rectangle_right == approx([
        Coord(vehicle.length/2, vehicle.width/2),
        Coord(vehicle.length/2, -vehicle.width/2),
        Coord(-vehicle.length/2, -vehicle.width/2),
        Coord(-vehicle.length/2, vehicle.width/2)])
    rectangle_right = vehicle.get_outline(static_buffer=.1)
    assert rectangle_right == approx([
        Coord(vehicle.length/2*1.1, vehicle.width/2*1.1),
        Coord(vehicle.length/2*1.1, -vehicle.width/2*1.1),
        Coord(-vehicle.length/2*1.1, -vehicle.width/2*1.1),
        Coord(-vehicle.length/2*1.1, vehicle.width/2*1.1)])

    vehicle.heading = pi/2
    rectangle_up = vehicle.get_outline()
    assert rectangle_up[0] == approx(Coord(-vehicle.width/2, vehicle.length/2))
    assert rectangle_up[1] == approx(Coord(vehicle.width/2, vehicle.length/2))
    assert rectangle_up[2] == approx(Coord(vehicle.width/2, -vehicle.length/2))
    assert rectangle_up[3] == approx(
        Coord(-vehicle.width/2, -vehicle.length/2))


def test_cloning(a: AutomatedVehicle = AutomatedVehicle(0, 0)):
    a = AutomatedVehicle(0, 0)
    b = a.clone_for_request()
    a.vin == b.vin
