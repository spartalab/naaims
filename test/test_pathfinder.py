from pytest import raises

from naaims.pathfinder import Pathfinder
from naaims.util import Coord


def test_hardcoded_pathfinder_rome():
    """All roads (well, road lanes) lead to one destination"""

    lane_1 = Coord(0, 0)
    destination_1 = 0
    lane_destination_pair_1_target = [Coord(0, 1), Coord(1, 0)]

    p = Pathfinder([], [], {
        (lane_1, destination_1): lane_destination_pair_1_target
    })

    assert p.next_movements(enters_intersection_at=lane_1,
                            destination=destination_1
                            ) == lane_destination_pair_1_target


def test_hardcoded_pathfinder_options():

    lane_il = Coord(0, 12)
    lane_iu = Coord(12, 0)
    lane_or = Coord(24, 12)
    lane_ou = Coord(12, 24)
    destination_u = 0
    destination_r = 1

    p = Pathfinder([], [], {
        (lane_il, destination_u): [lane_ou],
        (lane_iu, destination_u): [lane_ou],
        (lane_il, destination_r): [lane_or],
        (lane_iu, destination_r): [lane_or]
    })

    assert p.next_movements(enters_intersection_at=lane_il,
                            destination=destination_u) == [lane_ou]

    assert p.next_movements(enters_intersection_at=lane_il,
                            destination=destination_r) == [lane_or]
