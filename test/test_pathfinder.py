from pytest import raises

from aimsim.pathfinder import Pathfinder
from aimsim.util import Coord


def test_hardcoded_pathfinder():

    lane_1 = Coord(0, 0)
    destination_1 = 0
    lane_destination_pair_1_target = [Coord(0, 1), Coord(1, 0)]

    p = Pathfinder([], [], {
        (lane_1, destination_1): lane_destination_pair_1_target
    })

    assert p.next_movements(coord=lane_1, destination=destination_1
                            ) == lane_destination_pair_1_target

    with raises(NotImplementedError):
        p.next_movements(coord=lane_1, destination=1)

    with raises(NotImplementedError):
        p.next_movements(coord=lane_1, destination=destination_1,
                         at_least_one=True)
