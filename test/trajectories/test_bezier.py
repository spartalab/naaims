from math import pi, isclose

from aimsim.trajectories.bezier import BezierTrajectory
from aimsim.util import Coord


def test_length():

    # Straight line
    assert BezierTrajectory(Coord(0, 0), Coord(1, 0), [Coord(0.5, 0)]
                            ).length == 1

    # Symmetrical right turn
    assert isclose(BezierTrajectory(Coord(0, 0), Coord(1, 1), [Coord(0, 1)]
                                    ).length, pi/2, rel_tol=0.05)


def test_position():

    a = BezierTrajectory(Coord(0, 0), Coord(1, 0), [Coord(0.5, 0)])

    assert a.get_position(0) == Coord(0, 0)
    assert a.get_position(1) == Coord(1, 0)
    assert a.get_position(.1) == Coord(.1, 0)


def test_connection():

    inferred = BezierTrajectory.as_intersection_connector(
        Coord(0, 0), pi/2, Coord(1, 1), pi)

    assert isclose(inferred.control_coord.x, 0)
    assert isclose(inferred.control_coord.y, 1)
