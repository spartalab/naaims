from importlib import reload

from pytest import fixture

import aimsim.shared as SHARED
from aimsim.vehicles import AutomatedVehicle
from aimsim.road.lane import RoadLane

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
def rl():
    return RoadLane(straight_trajectory, 5, 30, 300, 300,
                    downstream_is_remover=True)
