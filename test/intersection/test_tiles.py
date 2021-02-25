from pytest import raises
from pytest_mock import MockerFixture

from aimsim.intersection.tilings.tiles import DeterministicTile
from aimsim.intersection.reservation import Reservation
from aimsim.vehicles import AutomatedVehicle
from aimsim.util import Coord, VehicleSection
from aimsim.intersection.lane import IntersectionLane
from aimsim.lane import ScheduledExit


def test_tile(mocker: MockerFixture, read_config: None):

    t = DeterministicTile(0, 0)
    v1 = AutomatedVehicle(0, 0)
    v2 = AutomatedVehicle(1, 0)

    # Create a road object to feed to remover, skipping all the checks the
    # Intersection init does because they aren't in this test's scope.
    mocker.patch.object(IntersectionLane, '__init__', return_value=None)

    r1 = Reservation(
        v1,
        Coord(0, 0),
        {0: {t: 1}},
        IntersectionLane(),
        ScheduledExit(v1, VehicleSection.FRONT, 0, 0)
    )
    r2 = Reservation(
        v2,
        Coord(0, 0),
        {0: {t: 1}},
        IntersectionLane(),
        ScheduledExit(v2, VehicleSection.FRONT, 0, 0)
    )

    assert t.will_reservation_work(r1) is True
    t.confirm_reservation(r1)
    assert t.will_reservation_work(r1) is True
    assert t.will_reservation_work(r2) is False
