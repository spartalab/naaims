from importlib import reload

from pytest import fixture

import aimsim.shared as SHARED
from aimsim.vehicles import AutomatedVehicle
from aimsim.road.lane import RoadLane
from aimsim.intersection.lane import IntersectionLane
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
