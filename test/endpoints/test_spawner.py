from __future__ import annotations
from aimsim.endpoints.factories.gaussian import GaussianVehicleFactory

from pytest import raises
from pytest_mock import MockerFixture

from aimsim.vehicles import AutomatedVehicle
from aimsim.endpoints import VehicleSpawner
from aimsim.util import VehicleTransfer, VehicleSection, Coord
from aimsim.road import Road

from test.endpoints.test_vehicle_factories import gvf_test_spec


def test_spawner(mocker: MockerFixture, read_config: None):

    # Create a road object to feed to remover, skipping all the checks Road's
    # init does because those aren't in the scope of this unit test.
    mocker.patch.object(Road, '__init__', return_value=None)
    mock_road = Road()
    mock_road.lanes = ()
    spawner = VehicleSpawner(mock_road, 1, [GaussianVehicleFactory],
                             [gvf_test_spec], [1])
    assert spawner.spawn_probability == 1/60**2
    spawner.spawn_probability = 1
    with raises(RuntimeError):
        spawner.step_vehicles()
