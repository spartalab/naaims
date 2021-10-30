from typing import List, Tuple, Type
from math import pi, ceil
from random import seed

from pytest import raises, fixture

import naaims.shared as SHARED
from naaims.util import CollisionError, Coord, VehicleSection
from naaims.trajectories import BezierTrajectory
from naaims.road import RoadLane
from naaims.intersection import IntersectionLane
from naaims.intersection.tilings import SquareTiling
from naaims.vehicles import Vehicle
from naaims.intersection.reservation import Reservation
from naaims.lane import ScheduledExit, VehicleProgress
from naaims.intersection.movement import (MovementModel, DeterministicModel,
                                          OneDrawStochasticModel)


def test_failing_init(load_shared: None):
    with raises(ValueError):
        SquareTiling({}, {}, (), {}, 0)


def test_simple_init(load_shared: None):
    coord_top_right = Coord(10, 10)
    rl_top_right = RoadLane(BezierTrajectory(
        coord_top_right, Coord(11, 11), [Coord(10.5, 10.5)]), 5, 30, .1, .1)
    coord_top_left = Coord(0, 10)
    rl_top_left = RoadLane(BezierTrajectory(
        Coord(-1, 11), coord_top_left, [Coord(-.5, 10.5)]), 5, 30, .1, .1)
    coord_bot_left = Coord(0, 0)
    rl_bot_left = RoadLane(BezierTrajectory(
        Coord(-1, -1), coord_bot_left, [Coord(-.5, -.5)]), 5, 30, .1, .1)
    coord_bot_right = Coord(10, 0)
    rl_bot_right = RoadLane(BezierTrajectory(
        coord_bot_right, Coord(11, -1), [Coord(10.5, -.5)]), 5, 30, .1, .1)

    il_down = IntersectionLane(rl_top_left, rl_bot_right, 30)
    il_up = IntersectionLane(rl_bot_left, rl_top_right, 30)

    sq = SquareTiling({coord_top_left: rl_top_left,
                       coord_bot_left: rl_bot_left},
                      {coord_top_right: rl_top_right,
                       coord_bot_right: rl_bot_right}, (il_down, il_up),
                      {(coord_top_left, coord_bot_right): il_down,
                       (coord_bot_left, coord_top_right): il_up},
                      2e-8,
                      misc_spec={'tile_width': 5})

    # Check SquareTiling-specific information
    assert sq.tile_width == 5
    assert sq.origin == Coord(0, 0)
    assert sq.min_x == 0
    assert sq.max_x == 10
    assert sq.min_y == 0
    assert sq.max_y == 10
    assert sq.x_tile_count == 2
    assert sq.y_tile_count == 2
    assert len(sq.buffer_tile_loc) == 4
    assert sq.threshold_registered
    assert sq.threshold == 2e-8


def test_slanted_init(load_shared: None):
    coord_top_right = Coord(9, 10)
    rl_top_right = RoadLane(BezierTrajectory(
        coord_top_right, Coord(11, 11), [Coord(10.5, 10.5)]), 5, 30, .1, .1)
    coord_top_left = Coord(0, 9)
    rl_top_left = RoadLane(BezierTrajectory(
        Coord(-1, 11), coord_top_left, [Coord(-.5, 10.5)]), 5, 30, .1, .1)
    coord_bot_left = Coord(1, 0)
    rl_bot_left = RoadLane(BezierTrajectory(
        Coord(-1, -1), coord_bot_left, [Coord(-.5, -.5)]), 5, 30, .1, .1)
    coord_bot_right = Coord(10, 1)
    rl_bot_right = RoadLane(BezierTrajectory(
        coord_bot_right, Coord(11, -1), [Coord(10.5, -.5)]), 5, 30, .1, .1)

    il_down = IntersectionLane(rl_top_left, rl_bot_right, 30)
    il_up = IntersectionLane(rl_bot_left, rl_top_right, 30)

    sq = SquareTiling({coord_top_left: rl_top_left,
                       coord_bot_left: rl_bot_left},
                      {coord_top_right: rl_top_right,
                       coord_bot_right: rl_bot_right}, (il_down, il_up),
                      {(coord_top_left, coord_bot_right): il_down,
                       (coord_bot_left, coord_top_right): il_up},
                      1e-6,
                      misc_spec={'tile_width': 5})

    # Check SquareTiling-specific information
    assert sq.tile_width == 5
    assert sq.origin == Coord(0, 0)
    assert sq.min_x == 0
    assert sq.max_x == 10
    assert sq.min_y == 0
    assert sq.max_y == 10
    assert sq.x_tile_count == 2
    assert sq.y_tile_count == 2
    assert len(sq.buffer_tile_loc) == 4
    assert sq.threshold_registered
    assert sq.threshold == 1e-6


def test_init_oblong_overtiled(load_shared: None):
    coord_top_right = Coord(9, 10)
    rl_top_right = RoadLane(BezierTrajectory(
        coord_top_right, Coord(11, 11), [Coord(10.5, 10.5)]), 5, 30, .1, .1)
    coord_top_left = Coord(-1, 9)
    rl_top_left = RoadLane(BezierTrajectory(
        Coord(-1, 11), coord_top_left, [Coord(-.5, 10.5)]), 5, 30, .1, .1)
    coord_bot_left = Coord(1, -2)
    rl_bot_left = RoadLane(BezierTrajectory(
        Coord(-1, -2), coord_bot_left, [Coord(-.5, -.5)]), 5, 30, .1, .1)
    coord_bot_right = Coord(10, 1)
    rl_bot_right = RoadLane(BezierTrajectory(
        coord_bot_right, Coord(11, -1), [Coord(10.5, -.5)]), 5, 30, .1, .1)

    il_down = IntersectionLane(rl_top_left, rl_bot_right, 30)
    il_up = IntersectionLane(rl_bot_left, rl_top_right, 30)

    sq = SquareTiling({coord_top_left: rl_top_left,
                       coord_bot_left: rl_bot_left},
                      {coord_top_right: rl_top_right,
                       coord_bot_right: rl_bot_right}, (il_down, il_up),
                      {(coord_top_left, coord_bot_right): il_down,
                       (coord_bot_left, coord_top_right): il_up},
                      0,
                      misc_spec={'tile_width': 11.5})

    # Check SquareTiling-specific information
    assert sq.tile_width == 11.5
    assert sq.origin == Coord(-1, -2)
    assert sq.min_x == -1
    assert sq.max_x == 10
    assert sq.min_y == -2
    assert sq.max_y == 10
    assert sq.x_tile_count == 1
    assert sq.y_tile_count == 2
    assert len(sq.buffer_tile_loc) == 4


def square_tiling_polygon(x_min: float, x_max: float, y_min: float,
                          y_max: float, tile_width: float,
                          speed_limit: int = 1, p_crash_total: float = 0,
                          movement_model: Type[MovementModel
                                               ] = DeterministicModel
                          ) -> SquareTiling:
    top_left = Coord(x_min, y_max)
    top_mid = Coord((x_max - x_min)/2 + x_min, y_max)
    top_right = Coord(x_max, y_max)
    rl_top = RoadLane(BezierTrajectory(
        top_left, top_right, [top_mid]), 0, speed_limit, 0, 0)
    bot_left = Coord(x_min, y_min)
    bot_mid = Coord((x_max - x_min)/2 + x_min, y_min)
    bot_right = Coord(x_max, y_min)
    rl_bot = RoadLane(BezierTrajectory(
        bot_left, bot_right, [bot_mid]), 0, speed_limit, 0, 0)

    il_down = IntersectionLane(rl_top, rl_bot, speed_limit, movement_model)
    il_up = IntersectionLane(rl_bot, rl_top, speed_limit, movement_model)

    return SquareTiling({top_right: rl_top,
                         bot_right: rl_bot},
                        {top_left: rl_top,
                         bot_left: rl_bot}, (il_down, il_up),
                        {(top_right, bot_left): il_down,
                         (bot_right, top_left): il_up},
                        p_crash_total,
                        misc_spec={'tile_width': tile_width})


@fixture
def sq():
    return square_tiling_polygon(0, 100, 0, 200, 1)


@fixture
def sq_stochastic():
    return square_tiling_polygon(0, 5, 0, 10, 1, p_crash_total=2e-8,
                                 movement_model=OneDrawStochasticModel)


def test_check_for_collisions(sq: SquareTiling, vehicle: Vehicle,
                              vehicle2: Vehicle, sq_stochastic: SquareTiling):
    sq.lanes[0].vehicles = [vehicle, vehicle2]
    sq_stochastic.lanes[0].vehicles = [vehicle, vehicle2]
    vehicle.pos = Coord(0, 0)
    vehicle2.pos = Coord(100, 100)
    sq.check_for_collisions()
    sq_stochastic.check_for_collisions()

    vehicle2.pos = Coord(4, 0)
    sq.check_for_collisions()
    with raises(CollisionError):
        sq_stochastic.check_for_collisions()


def check_line_range(sq: SquareTiling, start: Coord, end: Coord,
                     y_min_true: int, x_mins_true: List[int],
                     x_maxes_true: List[int]):
    y_min, x_mins, x_maxes = sq._line_to_tile_ranges(start, end)
    assert y_min == y_min_true
    assert x_mins == x_mins_true
    assert x_maxes == x_maxes_true


def test_line_to_range_down_right(load_shared: None, sq: SquareTiling):
    # Fully in
    check_line_range(sq, Coord(0.5, 1.5), Coord(2.5, .5), 0, [], [2, 1])
    check_line_range(sq, Coord(1, 4), Coord(3, 1), 1, [], [3, 2, 1, 1])

    # Starts at edge
    check_line_range(sq, Coord(5, 200), Coord(7, 199), 199, [], [7, 5])

    # Ends at edge
    check_line_range(sq, Coord(98, 150), Coord(100, 147), 147, [],
                     [100, 99, 98, 98])

    # Starts and ends at edge
    check_line_range(sq, Coord(98, 200), Coord(100, 197), 197, [],
                     [100, 99, 98, 98])


def test_line_to_range_down_left(load_shared: None, sq: SquareTiling):
    # Fully in
    check_line_range(sq, Coord(2.5, 1.5), Coord(.5, .5), 0, [], [1, 2])

    # Starts at edge
    check_line_range(sq, Coord(5, 200), Coord(3, 199), 199, [], [4, 5])

    # Ends at edge
    check_line_range(sq, Coord(2, 150), Coord(0, 147), 147, [],
                     [0, 1, 1, 2])

    # Starts and ends at edge
    check_line_range(sq, Coord(2, 200), Coord(0, 197), 197, [], [0, 1, 1, 2])

    # Fully in negative test
    check_line_range(sq, Coord(-1, -2), Coord(-5, -5), -5, [],
                     [-4, -3, -2, -1])


def test_line_to_range_up_left(load_shared: None, sq: SquareTiling):
    # Fully in
    check_line_range(sq, Coord(2.5, .5), Coord(.5, 1.5), 0, [1, 0], [])

    # Starts at edge
    check_line_range(sq, Coord(100, 147), Coord(98, 150), 147,
                     [99, 98, 98, 98], [])

    # Ends at edge
    check_line_range(sq, Coord(7, 199), Coord(5, 200), 199, [5, 5], [])

    # Starts and ends at edge
    check_line_range(sq, Coord(100, 197), Coord(98, 200), 197,
                     [99, 98, 98, 98], [])

    # Starts and ends at edge
    check_line_range(sq, Coord(-5, -7), Coord(-9, -5), -7, [-7, -9, -9], [])

    # Ends on x and y transition.
    check_line_range(sq, Coord(59, -2.446), Coord(51, 1), -3,
                     [57, 55, 53, 51, 51], [])

    # Non-1:1 tile to coordinate system
    start = Coord(x=-0.03999999997779238, y=2.099999999966693)
    end = Coord(x=-0.04000000002220129, y=2.699999999966693)
    sq5 = square_tiling_polygon(0, 0, 24, 24, 5)
    check_line_range(sq5, start, end, 2, [-1], [-1])


def test_line_to_range_up_right(load_shared: None, sq: SquareTiling):
    # Fully in
    check_line_range(sq, Coord(.5, .5), Coord(2.5, 1.5), 0, [0, 1], [])
    check_line_range(sq, Coord(51, 1), Coord(60, 3), 1, [51, 55, 59], [])

    # Starts at edge
    check_line_range(sq, Coord(3, 199), Coord(5, 200), 199, [3, 4], [])

    # Ends at edge
    check_line_range(sq, Coord(0, 147), Coord(2, 150), 147, [0, 0, 1, 1], [])

    # Starts and ends at edge
    check_line_range(sq, Coord(0, 197), Coord(2, 200), 197, [0, 0, 1, 1], [])

    # Weird case that made me add isclose to an if statement
    check_line_range(sq, Coord(-3, -3), Coord(10.1, 0.1), -3, [-3, 1, 5, 9],
                     [])

    # Ends inside at a tile xy border.
    check_line_range(sq, Coord(.5, .5), Coord(2, 1), 0, [0, 1], [])


def test_line_to_range_up(load_shared: None, sq: SquareTiling):
    check_line_range(sq, Coord(4, .5), Coord(4, 1.5), 0, [4, 4], [4, 4])
    check_line_range(sq, Coord(100, 0), Coord(100, 3.5), 0,
                     [100, 100, 100, 100], [100, 100, 100, 100])


def test_line_to_range_down(load_shared: None, sq: SquareTiling):
    check_line_range(sq, Coord(4, 1.5), Coord(4, .5), 0, [4, 4], [4, 4])
    check_line_range(sq, Coord(100, 200), Coord(100, 197.5), 197,
                     [100, 100, 100, 100], [100, 100, 100, 100])


def test_line_to_range_left(load_shared: None, sq: SquareTiling):
    check_line_range(sq, Coord(2.5, 1), Coord(3.5, 1), 1, [2], [3])
    check_line_range(sq, Coord(100, 200), Coord(98.5, 200), 200, [98], [100])
    # Potential floating point error
    check_line_range(sq, Coord(x=5.749999999999999, y=-18.0),
                     Coord(x=-16.75, y=-17.999999999999996), -18, [-17], [5])


def test_line_to_range_right(load_shared: None, sq: SquareTiling):
    check_line_range(sq, Coord(3.5, 1.5), Coord(2.5, 1.5), 1, [2], [3])
    check_line_range(sq, Coord(0, 200), Coord(2.5, 200), 200, [0], [2])


def compare_clip(sq: SquareTiling, y_min: int, x_mins: List[int],
                 x_maxes: List[int], y_min_true: int, x_mins_true: List[int],
                 x_maxes_true: List[int]):
    y_min, x_mins, x_maxes = sq._clip_tile_range(y_min, x_mins, x_maxes)
    assert y_min == y_min_true
    assert x_mins == x_mins_true
    assert x_maxes == x_maxes_true


def test_clip_range(load_shared: None, sq: SquareTiling):

    # No clip
    compare_clip(sq, 98, [5, 5, 5, 5], [5, 5, 5, 5], 98, [5, 5, 5, 5],
                 [5, 5, 5, 5])

    # Clip top
    compare_clip(sq, 198, [5, 5, 5, 5], [5, 5, 5, 5], 198, [5, 5], [5, 5])

    # Clip bottom
    compare_clip(sq, -2, [5, 5, 5, 5], [5, 5, 5, 5], 0, [5, 5], [5, 5])

    # Clip left
    compare_clip(sq, 98, [-5, 5, -5, 5], [5, 5, 5, 5], 98, [0, 5, 0, 5],
                 [5, 5, 5, 5])

    # Clip right
    compare_clip(sq, 98, [5, 5, 5, 5], [5, 222, 5, 222], 98, [5, 5, 5, 5],
                 [5, 99, 5, 99])

    # All clip
    compare_clip(sq, -3, [-100 for _ in range(207)], [234 for _ in range(207)],
                 0, [0 for _ in range(200)], [99 for _ in range(200)])


def check_tile_range(sq: SquareTiling, outline: Tuple[Coord, ...],
                     y_min_true: int, x_mins_true: List[int],
                     x_maxes_true: List[int]):
    y_min, x_mins, x_maxes = sq._outline_to_tile_range(outline)
    assert y_min == y_min_true
    assert x_mins == x_mins_true
    assert x_maxes == x_maxes_true


def test_outline_to_range_below(load_shared: None, sq: SquareTiling):

    # Left
    check_tile_range(sq, (Coord(-1, -1), Coord(-1, -3), Coord(-3, -3)),
                     0, [], [])

    # Below
    check_tile_range(sq, (Coord(10, -1), Coord(10, -3), Coord(7, -3)),
                     0, [], [])

    # Right
    check_tile_range(sq, (Coord(200, -1), Coord(200, -3), Coord(101, -3)),
                     0, [], [])

    # Just touching left
    check_tile_range(sq, (Coord(0, 0), Coord(0, -3), Coord(-3, -3)),
                     0, [0], [0])

    # Just touching below
    check_tile_range(sq, (Coord(10.1, 0.1), Coord(13, 0.2), Coord(10, -3)),
                     0, [10], [13])

    # Just touching right
    check_tile_range(sq, (Coord(100, 0), Coord(101, 0), Coord(100, -3)),
                     0, [], [])
    check_tile_range(sq, (Coord(99.9, 0), Coord(101, 0), Coord(99.9, -3)),
                     0, [99], [99])


def test_outline_to_range_above(load_shared: None, sq: SquareTiling):

    # Left
    check_tile_range(sq, (Coord(-1, 201), Coord(-1, 203), Coord(-3, 203)),
                     199, [], [])

    # Above
    check_tile_range(sq, (Coord(10, 201), Coord(10, 203), Coord(7, 203)),
                     199, [], [])

    # Right
    check_tile_range(sq, (Coord(200, 201), Coord(200, 203), Coord(101, 203)),
                     199, [], [])

    # Just touching left
    check_tile_range(sq, (Coord(0, 200), Coord(0, 203), Coord(-3, 203)),
                     199, [], [])
    check_tile_range(sq, (Coord(0, 199.5), Coord(-3, 203), Coord(0, 203)),
                     199, [0], [0])

    # Just touching above
    check_tile_range(sq, (Coord(10.1, 200.1), Coord(10, 203), Coord(13, 200.2)
                          ), 199, [], [])
    check_tile_range(sq, (Coord(10.1, 199.1), Coord(10, 203), Coord(13, 199.2)
                          ), 199, [10], [13])

    # Just touching right
    check_tile_range(sq, (Coord(100, 200), Coord(100, 203), Coord(101, 200)),
                     199, [], [])
    check_tile_range(sq, (Coord(99.9, 199), Coord(99.9, 203), Coord(101, 199)),
                     199, [99], [99])


def test_outline_to_range_left(load_shared: None, sq: SquareTiling):

    # Left
    check_tile_range(sq, (Coord(-100, 10), Coord(-90, 20), Coord(-110, 5)),
                     5, [], [])

    # Just touching left
    check_tile_range(sq, (Coord(-100, 10), Coord(0.2, 12.1), Coord(0.1, 9.6)),
                     9, [0, 0, 0, 0], [0, 0, 0, 0])


def test_outline_to_range_right(load_shared: None, sq: SquareTiling):

    # Right
    check_tile_range(sq, (Coord(200, 10), Coord(201, 23), Coord(200, 3)),
                     3, [], [])

    # Just touching left
    check_tile_range(sq, (Coord(100, 10.5), Coord(100, 12.1), Coord(110, 9.6)),
                     9, [], [])
    check_tile_range(sq, (Coord(99, 10.5), Coord(99, 12.1), Coord(110, 9.6)),
                     10, [99, 99, 99], [99, 99, 99])


def test_outline_to_range_normal(load_shared: None, sq: SquareTiling):

    # Poke out top left
    check_tile_range(sq, (Coord(-1, 201), Coord(2.5, 199.7), Coord(.1, 196.4)),
                     196, [0, 0, 0, 0], [0, 1, 1, 2])

    # Poke out top
    check_tile_range(sq, (Coord(4.1, 201.6), Coord(10.8, 201.7),
                          Coord(7, 198.8)), 198, [6, 5], [7, 8])

    # Poke out top right
    check_tile_range(sq, (Coord(98.1, 199.6), Coord(100.8, 200.7),
                          Coord(102.4, 199.8), Coord(98.7, 197.2)), 197,
                     [98, 98, 98], [99, 99, 99])

    # Poke out left
    check_tile_range(sq, (Coord(-1.1, 101.6), Coord(-1.1, 103.7),
                          Coord(2.2, 103.7), Coord(2.2, 101.6)), 101,
                     [0, 0, 0], [2, 2, 2])

    # Fully inside
    check_tile_range(sq, (Coord(50.1, 101.6), Coord(50.1, 103.7),
                          Coord(52.2, 103.7), Coord(52.2, 101.6)), 101,
                     [50, 50, 50], [52, 52, 52])
    check_tile_range(sq, (Coord(50.1, 53.6), Coord(53.2, 55.7),
                          Coord(54.5, 53.6), Coord(51.4, 51.5)), 51,
                     [51, 50, 50, 50, 52], [52, 53, 54, 54, 53])

    # Poke out right
    check_tile_range(sq, (Coord(98.1, 154.4), Coord(102.2, 153.7),
                          Coord(100.1, 151.2)), 151, [99, 98, 98, 98],
                     [99, 99, 99, 99])

    # Poke out bottom left
    check_tile_range(sq, (Coord(-1.9, .6), Coord(1.8, 1.7),
                          Coord(3.4, .6), Coord(-.3, -.6)), 0,
                     [0, 0], [3, 2])

    # Poke out bottom
    check_tile_range(sq, (Coord(51, 1), Coord(60, 3), Coord(59, -2.5)),
                     0, [51, 51, 55, 59], [59, 59, 59, 60])

    # Poke out bottom right
    check_tile_range(sq, (Coord(97.1, -.8), Coord(101.3, 2.4),
                          Coord(102.6, -1.7)), 0, [98, 99], [99, 99])


def test_pos_to_tile(load_shared: None, sq: SquareTiling,
                     vehicle: Vehicle, vehicle2: Vehicle):
    SHARED.t = 1
    assert len(sq.tiles) == 0
    i_lane0 = sq.lanes[0]
    coord0 = i_lane0.trajectory.start_coord
    res = Reservation(vehicle, coord0, {}, i_lane0, ScheduledExit(
        vehicle, VehicleSection.FRONT, 1, 10))

    # Can't reserve for the prior or current timestep
    with raises(ValueError):
        assert sq.pos_to_tiles(i_lane0, 0, vehicle, res)
    with raises(ValueError):
        assert sq.pos_to_tiles(i_lane0, 1, vehicle, res)

    # Prepending a reservation for the timestep just after the current one
    res.its_exit = ScheduledExit(vehicle, VehicleSection.FRONT, 0, 10)
    tiles = sq.pos_to_tiles(i_lane0, 6, vehicle, res)
    assert tiles is not None
    assert len(tiles) == 6
    for x in range(3):
        for y in range(2):
            tile = sq.tiles[4][sq._tile_loc_to_id((x, y))]
            assert tile in tiles
            assert tiles[tile] == 1
    assert len(sq.tiles) == 5

    # Diagonal heading, inside
    vehicle.pos = Coord(50.1, 100.5)
    vehicle.heading = 1.1
    tiles = sq.pos_to_tiles(i_lane0, 5, vehicle, res)
    assert tiles is not None
    x_mins = [49, 47, 47, 47, 48, 49, 49]
    x_maxes = [50, 51, 51, 52, 52, 52, 50]
    counter = 0
    for i, y in enumerate(range(97, 97+7)):
        for x in range(x_mins[i], x_maxes[i]+1):
            tile = sq.tiles[3][sq._tile_loc_to_id((x, y))]
            assert tile in tiles
            assert tiles[tile] == 1
            counter += 1
    assert len(tiles) == counter
    assert len(sq.tiles) == 5

    # Rejection test
    competing_res = Reservation(vehicle2, coord0, {}, i_lane0, ScheduledExit(
        vehicle2, VehicleSection.FRONT, 4, 10))
    reserved_tile_idx = sq._tile_loc_to_id((5, 5))
    sq.tiles[4][reserved_tile_idx].confirm_reservation(competing_res)
    vehicle.pos = Coord(5.5, 5.5)
    assert sq.pos_to_tiles(i_lane0, 6, vehicle, res) is None

    # Test adjusted tiling size and origin
    sq_1p1_2p1 = square_tiling_polygon(1.1, 10, 2.1, 10, .2)
    vehicle.pos = Coord(1, 1)
    vehicle.heading = pi*.9
    res_1p1 = Reservation(vehicle, sq.lanes[0].trajectory.start_coord, {},
                          sq.lanes[0],
                          ScheduledExit(vehicle, VehicleSection.FRONT, 1, 10))
    assert len(sq_1p1_2p1.tiles) == 0
    tiles = sq_1p1_2p1.pos_to_tiles(sq.lanes[0], 3, vehicle, res_1p1)
    assert tiles is not None
    assert len(sq_1p1_2p1.tiles) == 2
    x_maxes = [9, 6, 3, 0]
    counter = 0
    for y in range(4):
        for x in range(0, x_maxes[y]+1):
            tile = sq_1p1_2p1.tiles[1][sq_1p1_2p1._tile_loc_to_id((x, y))]
            assert tile in tiles
            assert tiles[tile] == 1
            counter += 1
    assert len(tiles) == counter


def test_loc_to_coord(load_shared: None, sq_stochastic: SquareTiling):
    assert sq_stochastic._tile_loc_to_coord((0, 0)) == Coord(0, 0)
    assert sq_stochastic._tile_loc_to_coord(
        (1, 0)) == Coord(sq_stochastic.tile_width, 0)
    assert sq_stochastic._tile_loc_to_coord(
        (0, 1)) == Coord(0, sq_stochastic.tile_width)


def test_pos_to_tiles_stochastic(load_shared: None,
                                 sq_stochastic: SquareTiling,
                                 h_vehicle: Vehicle):
    sq = sq_stochastic
    vehicle = h_vehicle

    SHARED.t = 1
    assert len(sq.tiles) == 0
    i_lane0 = sq.lanes[0]
    coord0 = i_lane0.trajectory.start_coord
    res = Reservation(vehicle, coord0, {}, i_lane0, ScheduledExit(
        vehicle, VehicleSection.FRONT, 1, 10))

    # Load vehicle onto lane
    seed(0)
    v0 = .1
    ts0 = 2
    en = ScheduledExit(h_vehicle, VehicleSection.FRONT, ts0, v0)
    v_max = 15
    i_lane0.vehicle_progress[h_vehicle] = VehicleProgress(.6, .6, .6)
    assert i_lane0.movement_model.prepend_probabilities(h_vehicle, en, v_max
                                                        ) == [1]
    SHARED.t = 3
    ts = 100
    vf = 10
    res.its_exit = ScheduledExit(vehicle, VehicleSection.FRONT, ts, vf)

    # Diagonal heading, inside
    vehicle.pos = Coord(2.6, 5.1)
    vehicle.heading = pi/4
    tiles = sq.pos_to_tiles(i_lane0, ts, vehicle, res)
    assert tiles is not None
    t_obs = ts-SHARED.t-1
    assert len(tiles) == len(sq.tiles[t_obs])
    assert set(tiles.keys()) == set(sq.tiles[t_obs])
    assert sum(tiles.values()) > 0
    tile_center = sq.tiles[t_obs][sq._tile_loc_to_id((4, 5))]
    tile_other = sq.tiles[t_obs][sq._tile_loc_to_id((0, 0))]
    assert tiles[tile_center] > .5 > tiles[tile_other]


def test_io_buffer(load_shared: None, sq: SquareTiling,
                   vehicle: Vehicle, vehicle2: Vehicle):

    SHARED.t = 0
    assert len(sq.tiles) == 0
    i_lane0 = sq.lanes[0]
    coord0 = i_lane0.trajectory.start_coord
    i_lane_start_tiles_idx = sq.buffer_tile_loc[coord0]
    i_lane_end_tiles_idx = sq.buffer_tile_loc[i_lane0.trajectory.end_coord]
    res = Reservation(vehicle, coord0, {}, i_lane0, ScheduledExit(
        vehicle, VehicleSection.FRONT, 0, 10))

    # Prepending a reservation before or at the current timestep
    with raises(ValueError):
        assert sq.io_tile_buffer(i_lane0, 0, vehicle, res, True)

    # Prepending a reservation for the timestep just after the current one
    res.its_exit = ScheduledExit(vehicle, VehicleSection.FRONT, 1, 10)
    assert sq.io_tile_buffer(i_lane0, 1, vehicle, res, True) == {}

    # Normal case accepted future prepend reservation
    prepended = sq.io_tile_buffer(i_lane0, 2, vehicle, res, True)
    assert prepended is not None
    assert len(sq.tiles) == 1
    assert len(prepended) == 1
    assert 1 in prepended
    assert prepended[1] == {sq.tiles[0][i_lane_start_tiles_idx]: 1}

    # Normal case rejected future prepend reservation
    competing_res = Reservation(vehicle2, coord0, {}, i_lane0, ScheduledExit(
        vehicle2, VehicleSection.FRONT, 0, 10))
    sq.tiles[0][i_lane_start_tiles_idx].confirm_reservation(competing_res)
    assert sq.io_tile_buffer(i_lane0, 2, vehicle, res, True) is None

    # Normal case accepted future postpend reservation
    postpended = sq.io_tile_buffer(i_lane0, 1, vehicle, res, False)
    assert postpended is not None
    steps_forward = ceil(.3 * SHARED.SETTINGS.steps_per_second)
    assert len(sq.tiles) == steps_forward + 1
    assert len(postpended) == steps_forward
    for i in range(5):
        assert postpended[i+2] == {sq.tiles[i+1][i_lane_end_tiles_idx]: 1}

    # Normal case rejected future postpend reservation
    competing_res = Reservation(vehicle2, coord0, {}, i_lane0, ScheduledExit(
        vehicle2, VehicleSection.FRONT, 4, 10))
    sq.tiles[4][i_lane_end_tiles_idx].confirm_reservation(competing_res)
    assert sq.io_tile_buffer(i_lane0, 1, vehicle, res, False) is None


def test_tile_loc_to_id(load_shared: None, sq: SquareTiling):
    assert sq._tile_loc_to_id((0, 0)) == 0
    assert sq._tile_loc_to_id((0, 1)) == 100
    assert sq._tile_loc_to_id((1, 0)) == 1
    assert sq._tile_loc_to_id((27, 138)) == 13_827
    assert sq._tile_loc_to_id((0, 199)) == 19_900
    assert sq._tile_loc_to_id((99, 199)) == 19_999


def test_new_layer(load_shared: None, sq_stochastic: SquareTiling):

    SHARED.t = 0
    # Recall that that the tiles are called on after all the movements in the
    # current timestep are completed, so the first entry in the tile stack
    # represents the current SHARED.t + 1.
    assert len(sq_stochastic.tiles) == 0
    sq_stochastic._add_new_layer()
    assert len(sq_stochastic.tiles) == 1
    assert len(sq_stochastic.tiles[0]) == 50
    assert hash(sq_stochastic.tiles[0][22]) == hash((22, 1))
    assert sq_stochastic.tiles[0][22].threshold == \
        sq_stochastic.threshold
    sq_stochastic._add_new_layer()
    assert len(sq_stochastic.tiles) == 2
    assert len(sq_stochastic.tiles[1]) == 50
    assert hash(sq_stochastic.tiles[1][22]) == hash((22, 2))
    assert sq_stochastic.tiles[0][22].threshold == \
        sq_stochastic.threshold

    # Mock next timestep
    SHARED.t += 1
    sq_stochastic.tiles.pop(0)
    assert len(sq_stochastic.tiles) == 1
    assert hash(sq_stochastic.tiles[0][22]) == hash((22, 2))
    sq_stochastic._add_new_layer()
    assert hash(sq_stochastic.tiles[0][22]) == hash((22, 2))
    assert hash(sq_stochastic.tiles[1][22]) == hash((22, 3))
    assert sq_stochastic.tiles[0][22].threshold == \
        sq_stochastic.threshold


def test_coord_to_tile(load_shared: None, sq: SquareTiling):
    assert sq._io_coord_to_tile_id(Coord(1, 1)) == 101
    assert sq._io_coord_to_tile_id(Coord(100, 200)) == 19_999
    assert sq._io_coord_to_tile_id(Coord(0, 11.5)) == 1_100
    assert sq._io_coord_to_tile_id(Coord(100, 11.5)) == 1_199
    assert sq._io_coord_to_tile_id(Coord(67.7, 0)) == 67
    assert sq._io_coord_to_tile_id(Coord(67.7, 200)) == 19_967


def test_io_stochastic(load_shared_clean: None, sq_stochastic: SquareTiling,
                       h_vehicle: Vehicle):

    SHARED.t = 0
    assert len(sq_stochastic.tiles) == 0
    i_lane0 = sq_stochastic.lanes[0]
    coord0 = i_lane0.trajectory.start_coord
    i_lane_start_tiles_idx = sq_stochastic.buffer_tile_loc[coord0]
    i_lane_end_tiles_idx = sq_stochastic.buffer_tile_loc[
        i_lane0.trajectory.end_coord]
    res = Reservation(h_vehicle, coord0, {}, i_lane0, ScheduledExit(
        h_vehicle, VehicleSection.FRONT, 0, 10))
    res.its_exit = ScheduledExit(h_vehicle, VehicleSection.FRONT, 1, 10)

    sq = sq_stochastic
    vehicle = h_vehicle
    # Prepending a reservation before or at the current timestep
    with raises(ValueError):
        assert sq.io_tile_buffer(i_lane0, 0, vehicle, res, True)

    # Prepending a reservation for the timestep just after the current one
    res.its_exit = ScheduledExit(vehicle, VehicleSection.FRONT, 1, 10)
    assert sq.io_tile_buffer(i_lane0, 1, vehicle, res, True) == {}

    # Normal case accepted future prepend reservation
    prepended = sq.io_tile_buffer(i_lane0, 2, vehicle, res, True)
    assert prepended is not None
    assert len(sq.tiles) == 1
    assert len(prepended) == 1
    assert 1 in prepended
    assert prepended[1] == {sq.tiles[0][i_lane_start_tiles_idx]: 1}

    # Load vehicle onto lane
    seed(0)
    v0 = .1
    ts0 = 2
    en = ScheduledExit(h_vehicle, VehicleSection.FRONT, ts0, v0)
    v_max = 15
    assert i_lane0.movement_model.prepend_probabilities(h_vehicle, en, v_max
                                                        ) == [1]
    SHARED.t = 169
    ts = 170
    vf = 10
    res.its_exit = ScheduledExit(vehicle, VehicleSection.FRONT, ts, vf)

    # Normal case accepted future postpend reservation
    vehicle.velocity = 15.
    postpended = sq.io_tile_buffer(i_lane0, ts, vehicle, res, False)
    assert postpended is not None
    steps_forward = ceil(.3 * SHARED.SETTINGS.steps_per_second)
    assert len(sq.tiles) - 1 == len(postpended) > steps_forward
    seed(0)
    assert i_lane0.movement_model.prepend_probabilities(h_vehicle, en, v_max
                                                        ) == [1]
    p_post = i_lane0.movement_model.postpend_probabilities(
        h_vehicle, sq._exit_res_timesteps_forward(vf), ts)
    for i in range(len(postpended)):
        assert postpended[SHARED.t+i+2] == {sq.tiles[i+1]
                                            [i_lane_end_tiles_idx]: p_post[i]}
