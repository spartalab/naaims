from importlib import reload

from pytest import fixture

import naaims.shared as SHARED
from naaims.intersection.managers import FCFSManager
from naaims.lane import VehicleProgress
from naaims.intersection import Intersection
from naaims.vehicles import Vehicle
from test.conftest import intersection as intersec


@fixture
def intersection(load_shared: None, lanes: int = 1, turns: bool = False):
    yield intersec(FCFSManager, lanes, turns)

    # Reset shared pathfinder to default of nothing.
    reload(SHARED)
    SHARED.SETTINGS.load()


def test_nonconflict_sequential(intersection: Intersection, vehicle: Vehicle,
                                vehicle2: Vehicle):
    fcfs: FCFSManager = intersection.manager  # type: ignore
    assert isinstance(fcfs, FCFSManager)

    road_lane_lr = list(intersection.incoming_road_lane_by_coord.values())[0]
    road_lane_up = list(intersection.incoming_road_lane_by_coord.values())[1]

    # Vehicle far back on incoming road lane from the left
    assert not vehicle.permission_to_enter_intersection
    road_lane_lr.vehicles.append(vehicle)
    road_lane_lr.vehicle_progress[vehicle] = VehicleProgress(.2, .15, .1)
    vehicle.pos = road_lane_lr.trajectory.get_position(.15)
    vehicle.heading = road_lane_lr.trajectory.get_heading(.15)
    fcfs.process_requests()
    assert vehicle.permission_to_enter_intersection

    # Vehicle very far forward on incoming road lane from below
    assert not vehicle2.permission_to_enter_intersection
    vehicle2._Vehicle__destination = 1  # type: ignore
    road_lane_up.vehicles.append(vehicle2)
    road_lane_up.vehicle_progress[vehicle2] = VehicleProgress(.95, .9, .85)
    vehicle2.pos = road_lane_lr.trajectory.get_position(.9)
    vehicle2.heading = road_lane_lr.trajectory.get_heading(.9)
    fcfs.process_requests()
    assert vehicle2.permission_to_enter_intersection


def test_nonconflict_simultaneous(intersection: Intersection, vehicle: Vehicle,
                                  vehicle2: Vehicle):
    fcfs: FCFSManager = intersection.manager  # type: ignore
    assert isinstance(fcfs, FCFSManager)

    road_lane_lr = list(intersection.incoming_road_lane_by_coord.values())[0]
    road_lane_up = list(intersection.incoming_road_lane_by_coord.values())[1]

    # Vehicle far back on incoming road lane from the left
    assert not vehicle.permission_to_enter_intersection
    road_lane_lr.vehicles.append(vehicle)
    road_lane_lr.vehicle_progress[vehicle] = VehicleProgress(.2, .15, .1)
    vehicle.pos = road_lane_lr.trajectory.get_position(.15)
    vehicle.heading = road_lane_lr.trajectory.get_heading(.15)

    # Vehicle very far forward on incoming road lane from below
    assert not vehicle2.permission_to_enter_intersection
    vehicle2._Vehicle__destination = 1  # type: ignore
    road_lane_up.vehicles.append(vehicle2)
    road_lane_up.vehicle_progress[vehicle2] = VehicleProgress(.95, .9, .85)
    vehicle2.pos = road_lane_lr.trajectory.get_position(.9)
    vehicle2.heading = road_lane_lr.trajectory.get_heading(.9)

    # Run FCFS once only
    fcfs.process_requests()
    assert vehicle.permission_to_enter_intersection
    assert vehicle2.permission_to_enter_intersection


def test_conflict_sequential(intersection: Intersection, vehicle: Vehicle,
                             vehicle2: Vehicle):
    fcfs: FCFSManager = intersection.manager  # type: ignore
    assert isinstance(fcfs, FCFSManager)

    road_lane_lr = list(intersection.incoming_road_lane_by_coord.values())[0]
    road_lane_up = list(intersection.incoming_road_lane_by_coord.values())[1]

    # Vehicle on incoming road lane from the left
    assert not vehicle.permission_to_enter_intersection
    road_lane_lr.vehicles.append(vehicle)
    road_lane_lr.vehicle_progress[vehicle] = VehicleProgress(.2, .15, .1)
    vehicle.pos = road_lane_lr.trajectory.get_position(.15)
    vehicle.heading = road_lane_lr.trajectory.get_heading(.15)
    fcfs.process_requests()
    assert vehicle.permission_to_enter_intersection

    # Vehicle on incoming road lane from below at the same time
    assert not vehicle2.permission_to_enter_intersection
    vehicle2._Vehicle__destination = 1  # type: ignore
    road_lane_up.vehicles.append(vehicle2)
    road_lane_up.vehicle_progress[vehicle2] = VehicleProgress(.2, .15, .1)
    vehicle2.pos = road_lane_lr.trajectory.get_position(.15)
    vehicle2.heading = road_lane_lr.trajectory.get_heading(.15)
    fcfs.process_requests()
    assert not vehicle2.permission_to_enter_intersection


def test_conflict_simultaneous(intersection: Intersection, vehicle: Vehicle,
                               vehicle2: Vehicle):
    fcfs: FCFSManager = intersection.manager  # type: ignore
    assert isinstance(fcfs, FCFSManager)

    road_lane_lr = list(intersection.incoming_road_lane_by_coord.values())[0]
    road_lane_up = list(intersection.incoming_road_lane_by_coord.values())[1]

    # Vehicle on incoming road lane from the left
    assert not vehicle.permission_to_enter_intersection
    road_lane_lr.vehicles.append(vehicle)
    road_lane_lr.vehicle_progress[vehicle] = VehicleProgress(.2, .15, .1)
    vehicle.pos = road_lane_lr.trajectory.get_position(.15)
    vehicle.heading = road_lane_lr.trajectory.get_heading(.15)

    # Vehicle on incoming road lane from below at the same time
    assert not vehicle2.permission_to_enter_intersection
    vehicle2._Vehicle__destination = 1  # type: ignore
    road_lane_up.vehicles.append(vehicle2)
    road_lane_up.vehicle_progress[vehicle2] = VehicleProgress(.2, .15, .1)
    vehicle2.pos = road_lane_lr.trajectory.get_position(.15)
    vehicle2.heading = road_lane_lr.trajectory.get_heading(.15)

    # Run FCFS once only
    fcfs.process_requests()
    assert vehicle.permission_to_enter_intersection != \
        vehicle2.permission_to_enter_intersection
