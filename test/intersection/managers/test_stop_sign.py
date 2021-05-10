from typing import Dict, Tuple, Any

from pytest import fixture

from aimsim.intersection.managers import StopSignManager
from aimsim.util import Coord
from aimsim.intersection.tilings import SquareTiling
from aimsim.road import RoadLane
from aimsim.lane import VehicleProgress
from aimsim.intersection import Intersection, IntersectionLane
from aimsim.vehicles import Vehicle


def test_init(incoming_road_lane_by_coord: Dict[Coord, RoadLane],
              outgoing_road_lane_by_coord: Dict[Coord, RoadLane],
              lanes: Tuple[IntersectionLane, ...],
              lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                       IntersectionLane],
              square_tiling_spec: Dict[str, Any]):

    ssm = StopSignManager(incoming_road_lane_by_coord,
                          outgoing_road_lane_by_coord, lanes,
                          lanes_by_endpoints, SquareTiling, square_tiling_spec)
    assert len(ssm.queue) == 0
    assert ssm.intersection_is_empty
    assert ssm.tol_closeness < 1e-3


def test_process_requests(intersection: Intersection, vehicle: Vehicle,
                          vehicle2: Vehicle):
    ssm: StopSignManager = intersection.manager
    assert type(ssm) is StopSignManager

    road_lane_0 = list(intersection.incoming_road_lane_by_coord.values())[0]
    road_lane_1 = list(intersection.incoming_road_lane_by_coord.values())[1]

    # Vehicle not at intersection yet, so nothing happens
    road_lane_0.vehicles.append(vehicle)
    road_lane_0.vehicle_progress[vehicle] = VehicleProgress(0, 0, 0)
    ssm.process_requests()
    assert len(ssm.queue) == 0
    assert not vehicle.permission_to_enter_intersection
    assert ssm.intersection_is_empty

    # Vehicle at intersection but intersection is full, so queued
    ssm.intersection_is_empty = False
    road_lane_0.vehicle_progress[vehicle] = VehicleProgress(1, 0, 0)
    ssm.process_requests()
    assert len(ssm.queue) == 1
    assert ssm.queue[0] is road_lane_0
    assert not vehicle.permission_to_enter_intersection
    assert not ssm.intersection_is_empty

    # Another vehicle arrives but the intersection is still full, so queued
    road_lane_1.vehicles.append(vehicle2)
    road_lane_1.vehicle_progress[vehicle2] = VehicleProgress(1, 0, 0)
    ssm.process_requests()
    assert len(ssm.queue) == 2
    assert ssm.queue[0] is road_lane_0
    assert ssm.queue[1] is road_lane_1
    assert not vehicle.permission_to_enter_intersection
    assert not vehicle2.permission_to_enter_intersection
    assert not ssm.intersection_is_empty

    # Intersection has emptied so first queued vehicle is allowed to enter
    ssm.intersection_is_empty = True
    ssm.process_requests()
    assert len(ssm.queue) == 1
    assert ssm.queue[0] is road_lane_1
    assert vehicle.permission_to_enter_intersection
    assert not vehicle2.permission_to_enter_intersection
    assert not ssm.intersection_is_empty

    # Two vehicles reach a full intersection at the same time, so both are
    # added to the queue
    vehicle.permission_to_enter_intersection = False
    ssm.queue = []
    ssm.process_requests()
    assert len(ssm.queue) == 2
    assert (road_lane_0 in ssm.queue) and (road_lane_1 in ssm.queue)
    assert not vehicle.permission_to_enter_intersection
    assert not vehicle2.permission_to_enter_intersection
    assert not ssm.intersection_is_empty

    # Two vehicles reach an empty intersection at the same time, so only one
    # is added to the queue and the other is allowed into the intersection
    ssm.intersection_is_empty = True
    ssm.queue = []
    ssm.process_requests()
    assert len(ssm.queue) == 1
    assert vehicle.permission_to_enter_intersection != \
        vehicle2.permission_to_enter_intersection
    assert ssm.queue[0] is (road_lane_1
                            if vehicle.permission_to_enter_intersection
                            else road_lane_0)
    assert not ssm.intersection_is_empty


def test_finish_exiting(intersection: Intersection, vehicle: Vehicle):
    ssm: StopSignManager = intersection.manager
    assert type(ssm) is StopSignManager

    vehicle.permission_to_enter_intersection = True
    ssm.intersection_is_empty = False
    ssm.finish_exiting(vehicle)
    assert not vehicle.has_reservation
    assert not vehicle.permission_to_enter_intersection
    assert ssm.intersection_is_empty
