from typing import List, Tuple
from pytest import raises, approx, fixture

from aimsim.util import Coord
from aimsim.trajectories import BezierTrajectory
from aimsim.road import RoadLane
from aimsim.intersection import IntersectionLane
from aimsim.intersection.tilings import SquareTiling


def test_failing_init(read_config: None):
    with raises(ValueError):
        SquareTiling({}, {}, (), {})


def test_simple_init(read_config: None):
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


def test_slanted_init(read_config: None):
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


def test_init_oblong_overtiled(read_config: None):
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
                          y_max: float, tile_width: float) -> SquareTiling:
    top_left = Coord(x_min, y_max)
    top_mid = Coord((x_max - x_min)/2 + x_min, y_max)
    top_right = Coord(x_max, y_max)
    rl_top = RoadLane(BezierTrajectory(
        top_left, top_right, [top_mid]), 0, 1, 0, 0)
    bot_left = Coord(x_min, y_min)
    bot_mid = Coord((x_max - x_min)/2 + x_min, y_min)
    bot_right = Coord(x_max, y_min)
    rl_bot = RoadLane(BezierTrajectory(
        bot_left, bot_right, [bot_mid]), 0, 1, 0, 0)

    il_down = IntersectionLane(rl_top, rl_bot, 1)
    il_up = IntersectionLane(rl_bot, rl_top, 1)

    return SquareTiling({top_right: rl_top,
                         bot_right: rl_bot},
                        {top_left: rl_top,
                         bot_left: rl_bot}, (il_down, il_up),
                        {(top_right, bot_left): il_down,
                         (bot_right, top_left): il_up},
                        misc_spec={'tile_width': tile_width})


@fixture
def sq():
    return square_tiling_polygon(0, 100, 0, 200, 1)


def check_line_range(sq: SquareTiling, start: Coord, end: Coord,
                     y_min_true: int, x_mins_true: List[int],
                     x_maxes_true: List[int]):
    y_min, x_mins, x_maxes = sq._line_to_tile_ranges(start, end)
    assert y_min == y_min_true
    assert x_mins == x_mins_true
    assert x_maxes == x_maxes_true


def test_line_to_range_down_right(read_config: None, sq: SquareTiling):
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


def test_line_to_range_down_left(read_config: None, sq: SquareTiling):
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


def test_line_to_range_up_left(read_config: None, sq: SquareTiling):
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


def test_line_to_range_up_right(read_config: None, sq: SquareTiling):
    # Fully in
    check_line_range(sq, Coord(.5, .5), Coord(2.5, 1.5), 0, [0, 1], [])

    # Starts at edge
    check_line_range(sq, Coord(3, 199), Coord(5, 200), 199, [3, 4], [])

    # Ends at edge
    check_line_range(sq, Coord(0, 147), Coord(2, 150), 147, [0, 0, 1, 1], [])

    # Starts and ends at edge
    check_line_range(sq, Coord(0, 197), Coord(2, 200), 197, [0, 0, 1, 1], [])


def test_line_to_range_up(read_config: None, sq: SquareTiling):
    check_line_range(sq, Coord(4, .5), Coord(4, 1.5), 0, [4, 4], [4, 4])
    check_line_range(sq, Coord(100, 0), Coord(100, 3.5), 0,
                     [100, 100, 100, 100], [100, 100, 100, 100])


def test_line_to_range_down(read_config: None, sq: SquareTiling):
    check_line_range(sq, Coord(4, 1.5), Coord(4, .5), 0, [4, 4], [4, 4])
    check_line_range(sq, Coord(100, 200), Coord(100, 197.5), 197,
                     [100, 100, 100, 100], [100, 100, 100, 100])


def test_line_to_range_left(read_config: None, sq: SquareTiling):
    check_line_range(sq, Coord(2.5, 1), Coord(3.5, 1), 1, [2], [3])
    check_line_range(sq, Coord(100, 200), Coord(98.5, 200), 200, [98], [100])


def test_line_to_range_right(read_config: None, sq: SquareTiling):
    check_line_range(sq, Coord(3.5, 1.5), Coord(2.5, 1.5), 1, [2], [3])
    check_line_range(sq, Coord(0, 200), Coord(2.5, 200), 200, [0], [2])


def compare_clip(sq: SquareTiling, y_min: int, x_mins: List[int],
                 x_maxes: List[int], y_min_true: int, x_mins_true: List[int],
                 x_maxes_true: List[int]):
    y_min, x_mins, x_maxes = sq._clip_tile_range(y_min, x_mins, x_maxes)
    assert y_min == y_min_true
    assert x_mins == x_mins_true
    assert x_maxes == x_maxes_true


def test_clip_range(read_config: None, sq: SquareTiling):

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
