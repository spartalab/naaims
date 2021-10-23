from pytest import raises, fixture

from naaims.intersection.movement import DeterministicModel
from naaims.lane import ScheduledExit, VehicleProgress
from naaims.util import Coord, VehicleSection, VehicleTransfer
from naaims.vehicles import Vehicle

from test.test_lane import straight_trajectory


@fixture
def model(p: float = 1e-8):
    model = DeterministicModel(straight_trajectory)
    model.register_rejection_threshold(p)
    return model


def test_init():
    m = DeterministicModel(straight_trajectory)
    with raises(RuntimeError):
        m.rejection_threshold

    m.register_rejection_threshold(0.1)
    assert m.rejection_threshold == 0.1


def test_deterministic(model: DeterministicModel, vehicle: Vehicle):
    model.init_lateral_deviation(vehicle)
    model.init_throttle_deviation(vehicle, VehicleTransfer(
        vehicle, VehicleSection.FRONT, 0, Coord(0, 0)), 15.)
    model.remove_vehicle(vehicle)
    assert model.fetch_lateral_deviation(vehicle, .5) == 0
    assert model.fetch_throttle_deviation(
        vehicle, VehicleSection.FRONT, .5) == 0
    assert model.find_probability_of_usage(
        vehicle, VehicleProgress(.1, .2, .3), Coord(0, 0), 5, 2) == 1
    its_exit = ScheduledExit(vehicle, VehicleSection.FRONT, 2, 5.0)
    assert model.prepend_probabilities(
        vehicle, its_exit, 15.) == [1]
    assert model.postpend_probabilities(
        vehicle, 5, 3) == [1 for _ in range(5)]


def test_clone(model: DeterministicModel):
    assert type(model.reset_for_requests()) is DeterministicModel
