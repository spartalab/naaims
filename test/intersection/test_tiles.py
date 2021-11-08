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
    assert tile.reserved_by[r1] == 1


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

    assert tile.reserved_by[r7] == 1e-7
    tile.confirm_reservation(r7, 1e-11)
    assert tile.reserved_by[r7] == 1e-7
    tile.confirm_reservation(r7, 1)
    assert tile.reserved_by[r7] == 1


def test_potential_reservation(load_shared: None, vehicle: Vehicle,
                               vehicle2: Vehicle, vehicle3: Vehicle,
                               il: IntersectionLane):

    tile = DeterministicTile(0, 0)
    res1 = Reservation(vehicle, Coord(0, 0), {}, il,
                       ScheduledExit(vehicle, VehicleSection.FRONT, 0, 0))
    res2 = Reservation(vehicle2, Coord(0, 0), {}, il,
                       ScheduledExit(vehicle, VehicleSection.FRONT, 0, 0),
                       dependent_on=res1, predecessors=frozenset([res1]))
    res1.dependency = res2
    res3 = Reservation(vehicle3, Coord(0, 0), {}, il,
                       ScheduledExit(vehicle, VehicleSection.FRONT, 0, 0))

    tile_stochastic = StochasticTile(0, 0, 1e-8)
    with raises(ValueError):
        tile_stochastic.mark(res1, 1)

    tile.mark(res1)
    assert res1 in tile.potentials
    tile.mark(res2)
    assert res2 not in tile.potentials
    tile.mark(res3)
    assert res3 in tile.potentials

    tile.confirm(res1)
    assert res1 in tile.reserved_by
    assert tile.will_reservation_work(res2)
    tile.confirm(res2)
    assert res2 not in tile.reserved_by
    assert tile.will_reservation_work(res2)
    tile.confirm(res3)
    assert res3 in tile.reserved_by
