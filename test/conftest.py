from typing import List, Tuple, Dict, Type, Any
from importlib import reload

from pytest import fixture

import naaims.shared as SHARED
from naaims.util import Coord
from naaims.trajectories import BezierTrajectory
from naaims.intersection.tilings.tiles import DeterministicTile
from naaims.intersection.tilings import SquareTiling
from naaims.vehicles import AutomatedVehicle
from naaims.road import Road
from naaims.road.lane import RoadLane
from naaims.intersection import Intersection
from naaims.intersection.lane import IntersectionLane
from naaims.intersection.managers import IntersectionManager, StopSignManager
from naaims.pathfinder import Pathfinder
from naaims.vehicles.human_guided import HumanGuidedVehicle
from naaims.intersection.movement import (MovementModel, DeterministicModel,
                                          OneDrawStochasticModel)
from test.test_lane import straight_trajectory


@fixture(scope="session")
def load_shared():
    try:
        SHARED.SETTINGS.load()
    except RuntimeError:
        pass


@fixture
def load_shared_clean():
    """For this test only, reset and reload the shared settings."""
    reload(SHARED)
    SHARED.SETTINGS.load()

    yield

    reload(SHARED)
    SHARED.SETTINGS.load()


@fixture
def clean_shared():
    """For this test only, reset as if the shared settings weren't loaded."""
    reload(SHARED)

    yield

    reload(SHARED)
    SHARED.SETTINGS.load()


@fixture
def vehicle(load_shared: None):
    return AutomatedVehicle(0, 0)


@fixture
def vehicle2(load_shared: None):
    return AutomatedVehicle(1, 0)


@fixture
def vehicle3(load_shared: None):
    return AutomatedVehicle(2, 0)


@fixture
def h_vehicle(load_shared: None):
    return HumanGuidedVehicle(0, 0, throttle_mn=.01, throttle_sd=.01,
                              tracking_mn=.01, tracking_sd=.01)


@fixture
def h_vehicle2(load_shared: None):
    return HumanGuidedVehicle(0, 0, throttle_mn=.02, throttle_sd=.02,
                              tracking_mn=.02, tracking_sd=.02)


@fixture
def rl():
    return RoadLane(straight_trajectory, 5, 30, 300, 300,
                    downstream_is_remover=True)


def il_pattern(movement_model: Type[MovementModel] = DeterministicModel):
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
    return IntersectionLane(rl_start, rl_end, speed_limit,
                            movement_model)


@fixture
def il():
    return il_pattern()


@fixture
def il_stochastic():
    return il_pattern(OneDrawStochasticModel)


def intersection(manager: Type[IntersectionManager] = StopSignManager,
                 lanes: int = 1, turns: bool = False) -> Intersection:

    # Create IO roads
    traj_i_lr = BezierTrajectory(Coord(-100, 12), Coord(0, 12),
                                 [Coord(-50, 12)])
    road_i_lr = Road(traj_i_lr, .09*traj_i_lr.length,
                     SHARED.SETTINGS.speed_limit, upstream_is_spawner=True,
                     downstream_is_remover=False, num_lanes=lanes,
                     len_approach_region=.9*traj_i_lr.length)
    traj_i_up = BezierTrajectory(Coord(12, -100), Coord(12, 0),
                                 [Coord(12, -50)])
    road_i_up = Road(traj_i_up, .09*traj_i_up.length,
                     SHARED.SETTINGS.speed_limit, upstream_is_spawner=True,
                     downstream_is_remover=False, num_lanes=lanes,
                     len_approach_region=.9*traj_i_up.length)
    traj_o_lr = BezierTrajectory(Coord(24, 12), Coord(124, 12),
                                 [Coord(74, 12)])
    road_o_lr = Road(traj_o_lr, .9*traj_o_lr.length,
                     SHARED.SETTINGS.speed_limit, upstream_is_spawner=True,
                     downstream_is_remover=True, num_lanes=lanes,
                     len_approach_region=.09*traj_o_lr.length)
    traj_o_up = BezierTrajectory(Coord(12, 24), Coord(12, 124),
                                 [Coord(12, 74)])
    road_o_up = Road(traj_o_up, .9*traj_o_up.length,
                     SHARED.SETTINGS.speed_limit, upstream_is_spawner=True,
                     downstream_is_remover=True, num_lanes=lanes,
                     len_approach_region=.09*traj_o_up.length)

    connectivity: List[Tuple[Road, Road, bool]] = [
        (road_i_lr, road_o_lr, True), (road_i_up, road_o_up, True)]

    if turns:
        connectivity.extend([(road_i_lr, road_o_up, False),
                             (road_i_up, road_o_lr, False)])

    intersection = Intersection(
        [road_i_lr, road_i_up], [road_o_lr, road_o_up], connectivity,
        manager_type=manager, manager_spec={
            'tiling_type': SquareTiling,
            'tiling_spec': {
                'tile_type': DeterministicTile,
                'crash_probability_tolerance': 0,
                'misc_spec': {
                    'tile_width': 5
                }
            },
            'misc_spec': {
                'cycle': (
                    ({(Coord(0, 12), Coord(24, 12)),
                      (Coord(0, 12), Coord(12, 24))}, 600),
                    ({(Coord(12, 0), Coord(12, 24)),
                      (Coord(12, 0), Coord(24, 12))}, 600))
            }
        }, speed_limit=SHARED.SETTINGS.speed_limit)

    # Prepare Pathfinder
    od_pair: Dict[Tuple[Coord, int], List[Coord]] = {
        (Coord(0, 12), 0): [Coord(24, 12)],
        (Coord(12, 0), 1): [Coord(12, 24)],
    }
    SHARED.SETTINGS.pathfinder = Pathfinder([], [], od_pair)

    return intersection


@fixture
def incoming_road_lane_by_coord(intersection: Intersection
                                ) -> Dict[Coord, RoadLane]:
    return intersection.incoming_road_lane_by_coord


@fixture
def outgoing_road_lane_by_coord(intersection: Intersection
                                ) -> Dict[Coord, RoadLane]:
    return intersection.outgoing_road_lane_by_coord


@fixture
def lanes(intersection: Intersection) -> Tuple[IntersectionLane, ...]:
    return intersection.lanes


@fixture
def lanes_by_endpoints(intersection: Intersection) -> Dict[Tuple[Coord, Coord],
                                                           IntersectionLane]:
    return intersection.lanes_by_endpoints


@fixture
def square_tiling_spec() -> Dict[str, Any]:
    return {'tile_type': DeterministicTile, 'crash_probability_tolerance': 0.,
            'misc_spec': {'tile_width': 5}}
