from math import pi, isclose

from naaims.trajectories.bezier import BezierTrajectory
from naaims.util import Coord


straight_trajectory = BezierTrajectory(Coord(0, 0), Coord(1, 0),
                                       [Coord(0.5, 0)])


def test_length():

    # Straight line
    assert straight_trajectory.length == 1

    # Symmetrical right turn
    assert isclose(BezierTrajectory(Coord(0, 0), Coord(1, 1), [Coord(0, 1)]
                                    ).length, pi/2, rel_tol=0.05)


def test_position():
    assert straight_trajectory.get_position(0) == Coord(0, 0)
    assert straight_trajectory.get_position(1) == Coord(1, 0)
    assert straight_trajectory.get_position(.1) == Coord(.1, 0)


def test_connection_90deg():

    inferred = BezierTrajectory.as_intersection_connector(
        Coord(0, 0), pi/2, Coord(1, 1), pi)

    assert isclose(inferred.control_coord.x, 0)
    assert isclose(inferred.control_coord.y, 1)
    assert inferred.get_position(.5) == Coord(0.25, 0.75)


def test_connection_vert():

    inferred = BezierTrajectory.as_intersection_connector(
        Coord(0, 0), pi/2, Coord(0, 1), pi/2)

    assert isclose(inferred.control_coord.x, 0)
    assert isclose(inferred.control_coord.y, .5)
    assert inferred.get_position(.5) == Coord(0, .5)


def test_connection_horizontal():

    inferred = BezierTrajectory.as_intersection_connector(
        Coord(0, 0), 0, Coord(1, 0), 0)

    assert isclose(inferred.control_coord.x, .5)
    assert isclose(inferred.control_coord.y, 0)
    assert inferred.get_position(.5) == Coord(.5, 0)


def test_clone():
    b = straight_trajectory.clone_with_offset(Coord(1, 1))

    assert isinstance(b, BezierTrajectory)
    assert b.start_coord == Coord(1, 1)
    assert b.reference_coords == [Coord(1.5, 1)]
    assert b.end_coord == Coord(2, 1)
    assert b.control_coord == Coord(1.5, 1)


def test_straight():
    assert BezierTrajectory(
        Coord(x=32.0, y=26.0),
        Coord(x=-4.898587196589413e-16, y=26.0),
        [Coord(x=2.7576594341012595e-05, y=26.000000000000004)]).straight
    assert BezierTrajectory(
        Coord(y=32.0, x=26.0),
        Coord(y=-4.898587196589413e-16, x=26.0),
        [Coord(y=2.7576594341012595e-05, x=26.000000000000004)]).straight
    assert not BezierTrajectory(Coord(x=32.0, y=26.0), Coord(x=-26.0, y=4.89),
                                [Coord(x=2.75, y=26.0)]).straight
