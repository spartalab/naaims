from importlib import reload
from math import ceil

from pytest import fixture, approx

import naaims.shared as SHARED
from naaims.lane import ScheduledExit, VehicleProgress
from naaims.intersection import Intersection
from naaims.intersection.managers import SignalManager
from naaims.util import VehicleSection
from naaims.vehicles import Vehicle
from test.conftest import intersection as intersec


@fixture
def intersection(load_shared: None, lanes: int = 1, turns: bool = True):
    yield intersec(SignalManager, lanes, turns)

    # Reset shared pathfinder to default of nothing.
    reload(SHARED)
    SHARED.SETTINGS.load()


def test_init(intersection: Intersection):
    assert type(intersection.manager) is SignalManager
    assert intersection.manager.cycle_ts == 1200
    assert len(intersection.manager.cycle) == 2


def test_phase(intersection: Intersection):
    assert type(intersection.manager) is SignalManager
    first_phase, ts_left = intersection.manager.get_phase()
    assert ts_left == 600
    SHARED.t = 1
    phase, ts_left = intersection.manager.get_phase()
    assert phase is first_phase
    assert ts_left == 599
    SHARED.t = 600
    second_phase, ts_left = intersection.manager.get_phase()
    assert second_phase is not first_phase
    assert ts_left == 600
    SHARED.t += 1
    phase, ts_left = intersection.manager.get_phase()
    assert phase is second_phase
    assert ts_left == 599
    SHARED.t = 1200
    phase, ts_left = intersection.manager.get_phase()
    assert phase is first_phase
    assert ts_left == 600


def test_rear(vehicle: Vehicle):

    # Already at v_max
    assert SignalManager.entrance_rear(
        ScheduledExit(vehicle, VehicleSection.FRONT, 0, 15), 3, 15, 0) == \
        ScheduledExit(vehicle, VehicleSection.REAR, ceil(
            vehicle.length_buffered/15 * SHARED.SETTINGS.steps_per_second), 15)

    # Accelerating from 0
    t_accel = (2*vehicle.length_buffered/3)**.5
    assert SignalManager.entrance_rear(
        ScheduledExit(vehicle, VehicleSection.FRONT, 10, 0), 3, 15, 10) == \
        approx(ScheduledExit(vehicle, VehicleSection.REAR, ceil(
            t_accel * SHARED.SETTINGS.steps_per_second)+10, 3*t_accel))

    # Hits v_max while crossing front to rear
    t_to_v_max = 1/3
    x_to_v_max = 14*t_to_v_max + .5*3*t_to_v_max**2
    assert SignalManager.entrance_rear(
        ScheduledExit(vehicle, VehicleSection.FRONT, 5, 14), 3, 15,
        t_to_v_max) == approx(ScheduledExit(vehicle, VehicleSection.REAR, ceil(
            (t_to_v_max + (vehicle.length_buffered - x_to_v_max)/15) *
            SHARED.SETTINGS.steps_per_second)+5, 15))


def test_process_green(intersection: Intersection, vehicle: Vehicle,
                       vehicle2: Vehicle, vehicle3: Vehicle):
    assert type(intersection.manager) is SignalManager

    lane = next(iter(intersection.incoming_road_lane_by_coord.values()))
    lane.vehicles = [vehicle, vehicle2, vehicle3]
    lane.vehicle_progress[vehicle] = VehicleProgress(.99, .98, .97)
    lane.vehicle_progress[vehicle2] = VehicleProgress(.96, .95, .94)
    lane.vehicle_progress[vehicle3] = VehicleProgress(.93, .92, .91)
    assert vehicle.permission_to_enter_intersection is False
    assert vehicle2.permission_to_enter_intersection is False
    assert vehicle3.permission_to_enter_intersection is False
    assert vehicle.has_reservation is False
    assert vehicle2.has_reservation is False
    assert vehicle3.has_reservation is False

    intersection.manager.process_requests()
    assert vehicle.permission_to_enter_intersection is True
    assert vehicle2.permission_to_enter_intersection is True
    assert vehicle3.permission_to_enter_intersection is True
    assert vehicle.has_reservation is False
    assert vehicle2.has_reservation is False
    assert vehicle3.has_reservation is False

    vehicle.permission_to_enter_intersection = \
        vehicle2.permission_to_enter_intersection = \
        vehicle3.permission_to_enter_intersection = False


def test_process_too_late(intersection: Intersection, vehicle: Vehicle,
                          vehicle2: Vehicle, vehicle3: Vehicle):
    assert type(intersection.manager) is SignalManager

    lane = next(iter(intersection.incoming_road_lane_by_coord.values()))
    lane.vehicles = [vehicle, vehicle2, vehicle3]
    lane.vehicle_progress[vehicle] = VehicleProgress(.99, .98, .97)
    lane.vehicle_progress[vehicle2] = VehicleProgress(.96, .95, .94)
    lane.vehicle_progress[vehicle3] = VehicleProgress(.02, .01, 0)
    intersection.manager.process_requests()
    assert vehicle.permission_to_enter_intersection is True
    assert vehicle2.permission_to_enter_intersection is True
    assert vehicle3.permission_to_enter_intersection is False
    assert vehicle.has_reservation is False
    assert vehicle2.has_reservation is False
    assert vehicle3.has_reservation is False


def test_process_red(intersection: Intersection, vehicle: Vehicle,
                     vehicle2: Vehicle, vehicle3: Vehicle):
    assert type(intersection.manager) is SignalManager

    lane = next(iter(intersection.incoming_road_lane_by_coord.values()))
    lane.vehicles = [vehicle, vehicle2, vehicle3]
    lane.vehicle_progress[vehicle] = VehicleProgress(.99, .98, .97)
    lane.vehicle_progress[vehicle2] = VehicleProgress(.96, .95, .94)
    lane.vehicle_progress[vehicle3] = VehicleProgress(.02, .01, 0)
    SHARED.t = 700
    intersection.manager.process_requests()
    assert vehicle.permission_to_enter_intersection is False
    assert vehicle2.permission_to_enter_intersection is False
    assert vehicle3.permission_to_enter_intersection is False
    assert vehicle.has_reservation is False
    assert vehicle2.has_reservation is False
    assert vehicle3.has_reservation is False
