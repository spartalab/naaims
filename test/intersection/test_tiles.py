from pytest import raises

from naaims.intersection.tilings.tiles import (Tile, DeterministicTile,
                                               StochasticTile)
from naaims.intersection.reservation import Reservation
from naaims.vehicles import Vehicle
from naaims.util import Coord, VehicleSection
from naaims.intersection.lane import IntersectionLane
from naaims.lane import ScheduledExit


def test_deterministic_tile(load_shared: None, vehicle: Vehicle,
                            vehicle2: Vehicle, il: IntersectionLane):

    def res(vehicle: Vehicle, tile: Tile, p: float):
        return Reservation(vehicle, Coord(0, 0), {0: {tile: p}}, il,
                           ScheduledExit(vehicle, VehicleSection.FRONT, 0, 0))

    tile: Tile = DeterministicTile(0, 0)
    r1 = res(vehicle, tile, 1)
    r2 = res(vehicle2, tile, 1)
    assert tile.will_reservation_work(r1) is True
    tile.confirm_reservation(r1)
    assert tile.will_reservation_work(r1) is True
    assert tile.will_reservation_work(r2) is False
    tile.confirm_reservation(r1, .5)
    assert tile.reserved_by[vehicle.vin] == 1


def test_stochastic_tile(load_shared: None, vehicle: Vehicle,
                         vehicle2: Vehicle, vehicle3: Vehicle,
                         il: IntersectionLane):

    with raises(ValueError):
        StochasticTile(0, 0, -1e-8)
    with raises(ValueError):
        StochasticTile(0, 0, 1+1e-9)

    def res(vehicle: Vehicle, tile: Tile, p: float):
        return Reservation(vehicle, Coord(0, 0), {0: {tile: p}}, il,
                           ScheduledExit(vehicle, VehicleSection.FRONT, 0, 0))

    tile = StochasticTile(0, 0, 1e-8)
    assert tile.threshold == 1e-8
    r1 = res(vehicle, tile, 1)
    r2 = res(vehicle2, tile, 1)
    assert tile.will_reservation_work(r1) is True
    tile.confirm_reservation(r1)
    assert tile.will_reservation_work(r1) is True
    assert tile.will_reservation_work(r2) is False

    tile = StochasticTile(0, 0, 1e-8)
    r7 = res(vehicle, tile, 1e-7)
    assert tile.will_reservation_work(r7, 1e-7) is True
    tile.confirm_reservation(r7, 1e-7)
    assert tile.will_reservation_work(r7, 1e-7) is True
    r6 = res(vehicle2, tile, 1e-6)
    assert tile.will_reservation_work(r6, 1e-6) is True
    tile.confirm_reservation(r6, 1e-6)
    assert tile.will_reservation_work(r6, 1e-6) is True
    r2 = res(vehicle3, tile, 1e-2)
    assert tile.will_reservation_work(r2, 1e-2) is False
    r39 = res(vehicle3, tile, 9e-3)
    assert tile.will_reservation_work(r39, 9e-3) is True

    assert tile.reserved_by[vehicle.vin] == 1e-7
    tile.confirm_reservation(r7, 1e-11)
    assert tile.reserved_by[vehicle.vin] == 1e-7
    tile.confirm_reservation(r7, 1)
    assert tile.reserved_by[vehicle.vin] == 1
