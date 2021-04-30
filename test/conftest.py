from importlib import reload
from typing import List, Tuple, Dict, Type
from math import pi

from pytest import fixture

import aimsim.shared as SHARED
from aimsim.vehicles import AutomatedVehicle
from aimsim.road import Road
from aimsim.road.lane import RoadLane
from aimsim.intersection import Intersection
from aimsim.intersection.lane import IntersectionLane
from aimsim.intersection.managers import StopSignManager
from aimsim.intersection.tilings import SquareTiling
from aimsim.intersection.tilings.tiles import Tile, DeterministicTile
from aimsim.trajectories import BezierTrajectory
from aimsim.util import Coord

from test.test_lane import straight_trajectory


@fixture(scope="session")
def read_config():
    try:
        SHARED.SETTINGS.read()
    except RuntimeError:
        pass


@fixture
def clean_config():
    reload(SHARED)


@fixture
def vehicle(read_config: None):
    return AutomatedVehicle(0, 0)


@fixture
def vehicle2(read_config: None):
    return AutomatedVehicle(1, 0)


@fixture
def vehicle3(read_config: None):
    return AutomatedVehicle(2, 0)


@fixture
def rl():
    return RoadLane(straight_trajectory, 5, 30, 300, 300,
                    downstream_is_remover=True)


@fixture
def il():
    width = 5
    speed_limit = SHARED.SETTINGS.speed_limit
    rl_start = RoadLane(
        BezierTrajectory(Coord(-10, 0), Coord(0, 0), [Coord(-5, 0)]),
        width, speed_limit, .2, .45
    )
    rl_end = RoadLane(
        BezierTrajectory(Coord(100, 0), Coord(110, 0), [Coord(105, 0)]),
        width, speed_limit, .2, .45
    )
    return IntersectionLane(rl_start, rl_end, speed_limit)


@fixture
def intersection(read_config: None) -> Intersection:

    trajectory1 = BezierTrajectory(Coord(-100, 0), Coord(0, 0),
                                   [Coord(-50, 0)])
    trajectory2 = BezierTrajectory(Coord(100, 0), Coord(200, 0),
                                   [Coord(150, 0)])

    # Create IO roads
    road_in = Road(trajectory1, .2*trajectory1.length,
                   SHARED.SETTINGS.speed_limit,
                   upstream_is_spawner=True, downstream_is_remover=True,
                   num_lanes=2, lane_offset_angle=pi/2,
                   len_approach_region=.7*trajectory1.length)
    road_out = Road(trajectory2, .2*trajectory2.length,
                    SHARED.SETTINGS.speed_limit,
                    upstream_is_spawner=True, downstream_is_remover=True,
                    num_lanes=2, lane_offset_angle=pi/2,
                    len_approach_region=.7*trajectory1.length)

    intersection = Intersection([road_in], [road_out],
                                [(road_in, road_out, True)],
                                manager_type=StopSignManager,
                                manager_spec={
                                    'tiling_type': SquareTiling,
                                    'tiling_spec': {
                                        'tile_type': DeterministicTile,
                                        'misc_spec': {
                                            'tile_width': 5
                                        }
                                    }
    },
        speed_limit=SHARED.SETTINGS.speed_limit)

    return intersection


@fixture
def manager_setup(intersection: Intersection) -> \
    Tuple[Dict[Coord, RoadLane], Dict[Coord, RoadLane],
          Tuple[IntersectionLane, ...], Dict[Tuple[Coord, Coord],
                                             IntersectionLane],
          Type[Tile]]:
    return (
        intersection.incoming_road_lane_by_coord,
        intersection.outgoing_road_lane_by_coord,
        intersection.lanes,
        intersection.lanes_by_endpoints,
        intersection.manager.tiling.tile_type
    )
