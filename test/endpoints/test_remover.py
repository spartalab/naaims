from __future__ import annotations

from pytest import raises
from pytest_mock import MockerFixture

from aimsim.vehicles import AutomatedVehicle
from aimsim.endpoints.remover import VehicleRemover
from aimsim.util import VehicleTransfer, VehicleSection, Coord
import aimsim.shared as SHARED


def test_remover(mocker: MockerFixture):

    SHARED.SETTINGS.read()
    mock_road = mocker.patch('aimsim.road.Road')
    remover = VehicleRemover(mock_road)

    transferring_vehicle = AutomatedVehicle(0, 0)
    another_transferring_vehicle = AutomatedVehicle(1, 0)
    third_transferring_vehicle = AutomatedVehicle(3, 0)
    transfer_front = VehicleTransfer(transferring_vehicle,
                                     VehicleSection.FRONT, 0, Coord(0, 0))
    transfer_center = VehicleTransfer(transferring_vehicle,
                                      VehicleSection.CENTER, 0, Coord(0, 0))
    transfer_another = VehicleTransfer(another_transferring_vehicle,
                                       VehicleSection.FRONT, 0, Coord(0, 0))
    transfer_third = VehicleTransfer(third_transferring_vehicle,
                                     VehicleSection.CENTER, 0, Coord(0, 0))
    remover.transfer_vehicle(transfer_front)
    remover.transfer_vehicle(transfer_center)
    remover.transfer_vehicle(transfer_another)
    remover.transfer_vehicle(transfer_third)
    assert len(remover.entering_vehicle_buffer) == 4
    exited = remover.process_transfers()
    assert len(exited) == 2
    assert exited[0].vin == 0
    assert exited[1].vin == 3
    assert len(remover.entering_vehicle_buffer) == 0
