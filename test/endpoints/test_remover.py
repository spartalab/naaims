from __future__ import annotations

from pytest import raises
from pytest_mock import MockerFixture

from naaims.vehicles import AutomatedVehicle
from naaims.endpoints import VehicleRemover
from naaims.util import VehicleTransfer, VehicleSection, Coord
from naaims.road import Road


def test_remover(mocker: MockerFixture, load_shared: None):

    # Create a road object to feed to remover, skipping all the checks Road's
    # init does because those aren't in the scope of this unit test.
    mocker.patch.object(Road, '__init__', return_value=None)
    remover = VehicleRemover(Road())

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
