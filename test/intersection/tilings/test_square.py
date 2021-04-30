from typing import Tuple
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


def test_first_on_grid_normal(read_config: None, sq: SquareTiling):
    # Starts and finishes in
    assert sq._first_point_on_grid(Coord(2.1, 2), Coord(2.2, 3), 1/.1, -19
                                   ) == Coord(2.1, 2)

    # Starts in and finishes out
    assert sq._first_point_on_grid(Coord(99, 99), Coord(101, 101), 1, 0
                                   ) == Coord(99, 99)


def test_first_on_grid_out_in(read_config: None, sq: SquareTiling):
    # Starts out and finishes in
    assert sq._first_point_on_grid(Coord(-1, -1), Coord(1, 1), 1, 0
                                   ) == Coord(0, 0)
    assert sq._first_point_on_grid(Coord(101, 101), Coord(99, 99), 1, 0
                                   ) == Coord(100, 100)
    assert sq._first_point_on_grid(Coord(49, 201), Coord(51, 199), -1, 250
                                   ) == Coord(50, 200)
    assert sq._first_point_on_grid(Coord(-1, 52), Coord(1, 48), -2, 50
                                   ) == Coord(0, 50)
    assert sq._first_point_on_grid(Coord(26, -3), Coord(23, 6), -3, 75
                                   ) == Coord(25, 0)
    assert sq._first_point_on_grid(Coord(101, 201), Coord(0, 100), 1, 100
                                   ) == Coord(100, 200)


def test_first_on_grid_crosses(read_config: None, sq: SquareTiling):
    # Starts out, crosses in, finishes out
    assert sq._first_point_on_grid(Coord(99, 201), Coord(101, 99), -1, 300
                                   ) == Coord(100, 200)
    assert sq._first_point_on_grid(Coord(98, 201), Coord(101, 98), -1, 299
                                   ) == Coord(99, 200)
    assert sq._first_point_on_grid(Coord(-1, 199), Coord(1, 201), 1, 200
                                   ) == Coord(0, 200)
    assert sq._first_point_on_grid(Coord(-1, 198), Coord(2, 201), 1, 199
                                   ) == Coord(0, 199)
    assert sq._first_point_on_grid(Coord(99, -1), Coord(101, 1), 1, -100
                                   ) == Coord(100, 0)
    assert sq._first_point_on_grid(Coord(98, -1), Coord(101, 2), 1, -99
                                   ) == Coord(99, 0)
    assert sq._first_point_on_grid(Coord(1, -1), Coord(-1, 1), -1, 0
                                   ) == Coord(0, 0)
    assert sq._first_point_on_grid(Coord(2, -1), Coord(1, 2), -1, 1
                                   ) == Coord(1, 0)


def test_first_on_grid_out(read_config: None, sq: SquareTiling):
    # Starts out and finishes out without ever going in
    assert sq._first_point_on_grid(Coord(9000, 201), Coord(
        9000, 100), float('inf'), float('inf')) is None
    assert sq._first_point_on_grid(Coord(-9000, 201), Coord(
        -9000, 100), float('inf'), float('inf')) is None
    assert sq._first_point_on_grid(Coord(-9000, 2001), Coord(
        -9000, 1000), float('inf'), float('inf')) is None
    assert sq._first_point_on_grid(Coord(50, 9000), Coord(
        51, 9000), 0, 9000) is None
    assert sq._first_point_on_grid(Coord(50, -9000), Coord(
        51, -9000), 0, -9000) is None
    assert sq._first_point_on_grid(Coord(500, 9000), Coord(
        501, 9000), 0, 9000) is None


def test_first_on_grid_horizontal_vertical(read_config: None,
                                           sq: SquareTiling):
    # Horizontals and verticals
    assert sq._first_point_on_grid(Coord(-1, 1), Coord(1, 1), 0, 1
                                   ) == Coord(0, 1)
    assert sq._first_point_on_grid(Coord(101, 1), Coord(99, 1), 0, 1
                                   ) == Coord(100, 1)
    assert sq._first_point_on_grid(Coord(1, -1), Coord(1, 1), float('inf'),
                                   float('inf')) == Coord(1, 0)
    assert sq._first_point_on_grid(Coord(1, 201), Coord(1, 199), float('inf'),
                                   float('inf')) == Coord(1, 200)


def compare_projected_outline(proj_outline: Tuple[Coord, ...],
                              true_outline: Tuple[Coord, ...]):
    for i in range(len(proj_outline)):
        assert proj_outline[i] == approx(true_outline[i])


def test_project_to_grid_normal(read_config: None, sq: SquareTiling):
    compare_projected_outline(sq._project_onto_grid((Coord(1, 1), Coord(2, 2),
                                                     Coord(3, 3))),
                              (Coord(1, 1), Coord(2, 2), Coord(3, 3)))


def test_project_to_grid_bot_left(read_config: None, sq: SquareTiling):
    compare_projected_outline(sq._project_onto_grid(
        (Coord(-2, -2), Coord(1, 4), Coord(4, 2))),
        (Coord(0, 2), Coord(1, 4), Coord(4, 2), Coord(1, 0), Coord(0, 0)))
    compare_projected_outline(sq._project_onto_grid(
        (Coord(4, 2), Coord(-2, -2), Coord(1, 4))),
        (Coord(4, 2), Coord(1, 0), Coord(0, 0), Coord(0, 2), Coord(1, 4)))
    compare_projected_outline(sq._project_onto_grid(
        (Coord(1, 4), Coord(4, 2), Coord(-2, -2))),
        (Coord(1, 4), Coord(4, 2), Coord(1, 0), Coord(0, 0), Coord(0, 2)))


def test_project_to_grid_top_left(read_config: None, sq: SquareTiling):
    compare_projected_outline(sq._project_onto_grid(
        (Coord(-2, 203), Coord(2, 199), Coord(-2, 197))),
        (Coord(1, 200), Coord(2, 199), Coord(0, 198), Coord(0, 200)))
    compare_projected_outline(sq._project_onto_grid(
        (Coord(-2, 197), Coord(-2, 203), Coord(2, 199))),
        (Coord(1, 200), Coord(2, 199), Coord(0, 198), Coord(0, 200)))
    compare_projected_outline(sq._project_onto_grid(
        (Coord(2, 199), Coord(-2, 197), Coord(-2, 203))),
        (Coord(2, 199), Coord(0, 198), Coord(0, 200), Coord(1, 200)))


def test_project_to_grid_top_right(read_config: None, sq: SquareTiling):
    compare_projected_outline(sq._project_onto_grid(
        (Coord(97, 198), Coord(100, 204), Coord(103, 200))),
        (Coord(97, 198), Coord(98, 200), Coord(100, 200), Coord(100, 199)))
    compare_projected_outline(sq._project_onto_grid(
        (Coord(103, 200), Coord(97, 198), Coord(100, 204))),
        (Coord(100, 199), Coord(97, 198), Coord(98, 200), Coord(100, 200)))
    compare_projected_outline(sq._project_onto_grid(
        (Coord(100, 204), Coord(103, 200), Coord(97, 198))),
        (Coord(100, 199), Coord(97, 198), Coord(98, 200), Coord(100, 200)))


def test_project_to_grid_bot_right(read_config: None, sq: SquareTiling):
    compare_projected_outline(sq._project_onto_grid(
        (Coord(98, 1), Coord(102, 3), Coord(102, -3))),
        (Coord(98, 1), Coord(100, 2), Coord(100, 0), Coord(99, 0)))
    compare_projected_outline(sq._project_onto_grid(
        (Coord(102, -3), Coord(98, 1), Coord(102, 3))),
        (Coord(99, 0), Coord(98, 1), Coord(100, 2), Coord(100, 0)))
    compare_projected_outline(sq._project_onto_grid(
        (Coord(102, 3), Coord(102, -3), Coord(98, 1))),
        (Coord(99, 0), Coord(98, 1), Coord(100, 2), Coord(100, 0)))


def test_project_to_grid_cut_left(read_config: None, sq: SquareTiling):
    compare_projected_outline(
        sq._project_onto_grid(
            (Coord(-1, 196), Coord(1, 196), Coord(-1, 193))),
        (Coord(0, 196), Coord(1, 196), Coord(0, 194.5)))


def test_project_to_grid_cut_top(read_config: None, sq: SquareTiling):
    compare_projected_outline(
        sq._project_onto_grid(
            (Coord(2, 201), Coord(5, 202), Coord(7, 201), Coord(7, 199),
             Coord(4, 199))),
        (Coord(7, 200), Coord(7, 199), Coord(4, 199), Coord(3, 200)))


def test_project_to_grid_cut_right(read_config: None, sq: SquareTiling):
    compare_projected_outline(
        sq._project_onto_grid(
            (Coord(99, 4), Coord(101, 4), Coord(101, 1), Coord(99, 1))),
        (Coord(99, 4), Coord(100, 4), Coord(100, 1), Coord(99, 1)))


def test_project_to_grid_cut_bot(read_config: None, sq: SquareTiling):
    compare_projected_outline(
        sq._project_onto_grid((Coord(5, 1), Coord(7, -1), Coord(3, -1))),
        (Coord(5, 1), Coord(6, 0), Coord(4, 0)))


def test_project_to_grid_overlay(read_config: None):
    # Covers over and past the entire tiling grid except for a little of the
    # bottom right corner.
    sq = square_tiling_polygon(0, 3, 0, 5, 1)
    compare_projected_outline(sq._project_onto_grid(
        (Coord(-2, 3), Coord(2, -1), Coord(4, -1), Coord(4, 7), Coord(-2, 7))
    ), (Coord(0, 1), Coord(1, 0), Coord(3, 0), Coord(3, 5), Coord(0, 5)))


def test_project_to_grid_in_n_out(read_config: None):
    sq = square_tiling_polygon(0, 5, 0, 5, 1)

    # Goes straight through the bottom right corner without leaving a vertex in
    # the shape.
    compare_projected_outline(sq._project_onto_grid(
        (Coord(3, -1), Coord(6, 2), Coord(7, 1), Coord(4, -2))),
        (Coord(4, 0), Coord(5, 1), Coord(5, 0)))

    # Like above, but does it twice such that it doesn't actually cover the
    # bottom right corner.
    compare_projected_outline(sq._project_onto_grid(
        (Coord(1, -1), Coord(6, 4), Coord(7, 3), Coord(2, -2))),
        (Coord(2, 0), Coord(5, 3), Coord(5, 1), Coord(4, 0)))
