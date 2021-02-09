from __future__ import annotations
from typing import Type

from pytest import raises
from pytest_mock import MockerFixture

from aimsim.vehicles import AutomatedVehicle
from aimsim.endpoints.factories import VehicleFactory, GaussianVehicleFactory
import aimsim.shared as SHARED


def test_gvf_creation(mocker: MockerFixture):
    if not SHARED.SETTINGS.config_file_already_read:
        SHARED.SETTINGS.read()

    gvf = GaussianVehicleFactory(vehicle_types=[AutomatedVehicle],
                                 destinations=100_000,
                                 max_accel_mn=3,
                                 max_accel_sd=0)
    veh0 = gvf.create_vehicle()
    veh1 = gvf.create_vehicle()
    assert veh0.vin != veh1.vin
    assert veh0.max_acceleration == veh1.max_acceleration == 3
    assert 0 <= veh0.destination < 100_000

    # 1 in 100,000 chance this test fails even if working correctly lol
    assert veh0.destination != veh1.destination

    gvf_varied = GaussianVehicleFactory(vehicle_types=[AutomatedVehicle],
                                        destinations=1,
                                        max_accel_mn=3,
                                        max_accel_sd=.5)
    assert gvf_varied.create_vehicle().max_acceleration != 3


def test_vehicle_types(mocker: MockerFixture,
                       f: Type[VehicleFactory] = GaussianVehicleFactory):
    if not SHARED.SETTINGS.config_file_already_read:
        SHARED.SETTINGS.read()

    f(vehicle_types=[AutomatedVehicle], destinations=1,
      destination_probabilities=[1.]).create_vehicle()

    # Breaks when provided probabilities exceeds one
    with raises(ValueError):
        f(vehicle_types=[AutomatedVehicle], destinations=1,
          destination_probabilities=[1, 1]).destination_probabilities

    # Breaks when provided probabilities sum to less than one
    with raises(ValueError):
        f(vehicle_types=[AutomatedVehicle], destinations=1,
          destination_probabilities=[0, 0.1]).destination_probabilities

    # Infers destination probabilities correctly
    assert f(vehicle_types=[AutomatedVehicle], destinations=1
             ).destination_probabilities == [1.]

    assert f(vehicle_types=[AutomatedVehicle], destinations=2
             ).destination_probabilities == [.5, .5]


def test_mismatched_vehicle_types_and_probs(
    mocker: MockerFixture,
        f: Type[VehicleFactory] = GaussianVehicleFactory):
    if not SHARED.SETTINGS.config_file_already_read:
        SHARED.SETTINGS.read()

    with raises(ValueError):
        f(vehicle_types=[AutomatedVehicle], destinations=1,
          destination_probabilities=[]).create_vehicle()

    with raises(ValueError):
        f(vehicle_types=[AutomatedVehicle], destinations=2,
          destination_probabilities=[1.]).create_vehicle()


def test_destination_matching(
    mocker: MockerFixture,
        f: Type[VehicleFactory] = GaussianVehicleFactory):
    if not SHARED.SETTINGS.config_file_already_read:
        SHARED.SETTINGS.read()

    # Breaks when no possible destination due to source exclusion
    with raises(ValueError):
        f(vehicle_types=[AutomatedVehicle], destinations=1,
          destination_probabilities=[1],
          source_node_id=0)

    # Breaks when provided probabilities exceeds one
    with raises(ValueError):
        f(vehicle_types=[AutomatedVehicle], destinations=1,
          destination_probabilities=[2],
          source_node_id=0)

    # Breaks when provided probabilities sum to less than one
    with raises(ValueError):
        f(vehicle_types=[AutomatedVehicle], destinations=1,
          destination_probabilities=[.5],
          source_node_id=0)

    # Picks correct destination
    assert f(vehicle_types=[AutomatedVehicle], destinations=2,
             destination_probabilities=[0, 1], source_node_id=0
             ).create_vehicle().destination == 1

    # Calculates equal probabilites correctly
    assert f(vehicle_types=[AutomatedVehicle], destinations=2
             ).destination_probabilities == [.5, .5]

    # Calculates equal probabilites correctly after source exclusion
    assert f(vehicle_types=[AutomatedVehicle], destinations=2,
             source_node_id=0).destination_probabilities == [0, 1]
