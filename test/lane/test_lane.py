from pytest import approx
from pytest_mock import MockerFixture

import aimsim.shared as SHARED
from aimsim.lane import Lane, VehicleProgress
from aimsim.road import RoadLane
from aimsim.intersection import IntersectionLane
from aimsim.trajectories import BezierTrajectory
from aimsim.util import Coord, SpeedUpdate, VehicleTransfer, VehicleSection
from aimsim.vehicles import Vehicle, AutomatedVehicle


straight_trajectory = BezierTrajectory(
    Coord(0, 0), Coord(1000, 0), [Coord(500, 0)])


def test_road_lane(mocker: MockerFixture):
    rl = RoadLane(straight_trajectory, 5, 30, .2*straight_trajectory.length,
                  .45*straight_trajectory.length)
    assert rl.entrance_end == .2
    assert rl.lcregion_end == .55
    assert rl.room_to_enter() == .2*straight_trajectory.length
    assert hash(rl) == hash(straight_trajectory)


def test_intersection_lane(mocker: MockerFixture, read_config: None):
    width = 5
    speed_limit = 30
    rl_start = RoadLane(
        BezierTrajectory(Coord(0, -1), Coord(0, 0), [Coord(0, -.5)]),
        width, speed_limit, .2, .45
    )
    rl_end = RoadLane(
        BezierTrajectory(Coord(1, 1), Coord(2, 1), [Coord(1.5, 1)]),
        width, speed_limit, .2, .45
    )
    il = IntersectionLane(rl_start, rl_end, speed_limit)
    assert hash(il.trajectory) == hash(
        BezierTrajectory(Coord(0, 0), Coord(1, 1), [Coord(0, 1)]))


def helper_forward_movement(lane: Lane, vehicle: Vehicle):

    # Test forward movement of vehicle
    updates = lane.get_new_speeds()
    vehicle.velocity = updates[vehicle].velocity
    vehicle.acceleration = updates[vehicle].acceleration
    lane.step_vehicles()
    rear_progress = lane.vehicle_progress[vehicle].rear
    assert rear_progress is not None
    assert lane.stopping_distance_to_last_vehicle() == \
        vehicle.stopping_distance() + rear_progress * lane.trajectory.length
    assert lane.effective_speed_limit(rear_progress,
                                      vehicle) == lane.speed_limit
    assert vehicle.acceleration == vehicle.max_acceleration
    assert vehicle.velocity == vehicle.max_acceleration * 1/60


def test_vehicle_entry_from_spawner(mocker: MockerFixture, read_config: None):

    vehicle_test = AutomatedVehicle(0, 0)
    rl = RoadLane(straight_trajectory, 5, 30, .2, .45,
                  upstream_is_spawner=True, downstream_is_remover=True)

    # Add the vehicle's front section to the lane
    rl.enter_vehicle_section(VehicleTransfer(
        vehicle_test, VehicleSection.FRONT, None, rl.trajectory.start_coord
    ))
    assert len(rl.vehicles) == 1
    assert len(rl.vehicle_progress) == 1
    assert rl.vehicles[0] == vehicle_test
    vp_test = rl.vehicle_progress[vehicle_test]
    assert vp_test.front == vehicle_test.length * \
        (1 + 2*SHARED.SETTINGS.length_buffer_factor) / rl.trajectory.length
    assert vp_test.center == vp_test.rear
    assert vp_test.center is None

    # Add the rest of the vehicle sections
    rl.enter_vehicle_section(VehicleTransfer(
        vehicle_test, VehicleSection.CENTER, None, rl.trajectory.start_coord
    ))
    rl.enter_vehicle_section(VehicleTransfer(
        vehicle_test, VehicleSection.REAR, None, rl.trajectory.start_coord
    ))
    assert len(rl.vehicles) == 1
    assert len(rl.vehicle_progress) == 1
    vp_test = rl.vehicle_progress[vehicle_test]
    assert vp_test.front == vehicle_test.length * \
        (1 + 2*SHARED.SETTINGS.length_buffer_factor) / rl.trajectory.length
    assert vp_test.center == vehicle_test.length * \
        (.5 + SHARED.SETTINGS.length_buffer_factor) / rl.trajectory.length
    assert vp_test.rear == 0

    # Test forward movement of vehicle
    helper_forward_movement(rl, vehicle_test)
    assert vehicle_test.pos.x == approx(
        vehicle_test.length*.6 + vehicle_test.max_acceleration * 1/60**2)


def test_vehicle_entry_from_facility(mocker: MockerFixture, read_config: None):

    vehicle_test = AutomatedVehicle(0, 0)
    il = IntersectionLane(RoadLane(
        BezierTrajectory(Coord(0, -1), Coord(0, 0), [Coord(0, -.5)]),
        0, 0, 0, 0),
        RoadLane(BezierTrajectory(Coord(100, 100), Coord(
            101, 100), [Coord(100.5, 100)]), 0, 0, 0, 0),
        30)

    # Add the vehicle's front section to the lane
    length_front = vehicle_test.length * \
        (1 + 2*.1)
    il.enter_vehicle_section(VehicleTransfer(
        vehicle_test, VehicleSection.FRONT, None, il.trajectory.start_coord
    ))
    assert len(il.vehicles) == 1
    assert len(il.vehicle_progress) == 1
    assert il.vehicles[0] == vehicle_test
    vp_test = il.vehicle_progress[vehicle_test]
    assert vp_test.front == length_front / il.trajectory.length
    assert vp_test.center == vp_test.rear
    assert vp_test.center is None

    # Add the rest of the vehicle sections
    length_center = vehicle_test.length * \
        (.5 + .1)
    length_rear = 0
    il.enter_vehicle_section(VehicleTransfer(
        vehicle_test, VehicleSection.CENTER, length_center,
        il.trajectory.start_coord
    ))
    il.enter_vehicle_section(VehicleTransfer(
        vehicle_test, VehicleSection.REAR, length_rear,
        il.trajectory.start_coord
    ))
    assert len(il.vehicles) == 1
    assert len(il.vehicle_progress) == 1
    vp_test = il.vehicle_progress[vehicle_test]
    assert vp_test.front == approx(length_front / il.trajectory.length)
    assert vp_test.center == length_center / il.trajectory.length
    assert vp_test.rear == length_rear

    # Test forward movement of vehicle
    helper_forward_movement(il, vehicle_test)
    post_movement_center_progress = il.vehicle_progress[vehicle_test].center
    assert post_movement_center_progress is not None
    assert vehicle_test.pos.x == il.trajectory.get_position(
        post_movement_center_progress
    ).x


def test_vehicle_control(mocker: MockerFixture, read_config: None):

    vehicle = AutomatedVehicle(0, 0)
    rl_in = RoadLane(
        BezierTrajectory(Coord(0, -1), Coord(0, 0), [Coord(0, -.5)]),
        0, 0, 0, 0, upstream_is_spawner=True)
    rl_out = RoadLane(BezierTrajectory(Coord(100, 100), Coord(101, 100),
                                       [Coord(100.5, 100)]),
                      0, 0, 0, 0, downstream_is_remover=True)
    il = IntersectionLane(rl_in, rl_out, 30)

    # front in intersection, center and rear in road
    rl_in.vehicle_progress[vehicle] = VehicleProgress(None, 1, 0.9)
    il.vehicle_progress[vehicle] = VehicleProgress(0, None, None)
    assert not rl_in.controls_this_speed(vehicle)[0]
    assert il.controls_this_speed(vehicle) == (True, 0, VehicleSection.FRONT)
    del rl_in.vehicle_progress[vehicle]
    del il.vehicle_progress[vehicle]

    # front and center in intersection, rear in road
    rl_in.vehicle_progress[vehicle] = VehicleProgress(None, None, 1)
    il.vehicle_progress[vehicle] = VehicleProgress(0.1, 0, None)
    assert not rl_in.controls_this_speed(vehicle)[0]
    assert il.controls_this_speed(vehicle) == (True, 0.1, VehicleSection.FRONT)
    del rl_in.vehicle_progress[vehicle]
    del il.vehicle_progress[vehicle]

    # totally in intersection
    il.vehicle_progress[vehicle] = VehicleProgress(0.2, 0.1, 0)
    assert il.controls_this_speed(vehicle) == (True, 0.2, VehicleSection.FRONT)
    del il.vehicle_progress[vehicle]

    # front in road, center and rear in intersection
    rl_out.vehicle_progress[vehicle] = VehicleProgress(0, None, None)
    il.vehicle_progress[vehicle] = VehicleProgress(None, 1, 0.9)
    assert not rl_out.controls_this_speed(vehicle)[0]
    assert il.controls_this_speed(vehicle) == (True, 0.9, VehicleSection.REAR)
    del rl_out.vehicle_progress[vehicle]
    del il.vehicle_progress[vehicle]

    # front and center in road, rear in intersection
    rl_out.vehicle_progress[vehicle] = VehicleProgress(0.1, 0, None)
    il.vehicle_progress[vehicle] = VehicleProgress(None, None, 1)
    assert not rl_out.controls_this_speed(vehicle)[0]
    assert il.controls_this_speed(vehicle) == (True, 1, VehicleSection.REAR)
    del rl_out.vehicle_progress[vehicle]
    del il.vehicle_progress[vehicle]

    # totally in road
    rl_out.vehicle_progress[vehicle] = VehicleProgress(0.2, 0.1, 0)
    assert rl_out.controls_this_speed(vehicle) == (True, 0.2,
                                                   VehicleSection.FRONT)
    del rl_out.vehicle_progress[vehicle]

    # front in remover, center and rear in intersection
    rl_out.vehicle_progress[vehicle] = VehicleProgress(None, 1, 0.9)
    assert rl_out.controls_this_speed(vehicle) == (True, 0.9,
                                                   VehicleSection.REAR)
    del rl_out.vehicle_progress[vehicle]

    # front and center in remover, rear in intersection
    # (not testing this since if both front and center are in remover the
    #  vehicle entry gets deleted)


def test_consecutive_vehicles(mocker: MockerFixture, read_config: None):
    vehA = AutomatedVehicle(0, 0)
    vehB = AutomatedVehicle(1, 0)
    veh_length = vehA.length*(1 + 2*SHARED.SETTINGS.length_buffer_factor)
    rl = RoadLane(BezierTrajectory(
        Coord(0, 0), Coord(veh_length*100, 0), [Coord(veh_length*50, 0)]
    ), 5, SHARED.SETTINGS.speed_limit, .2, .45,
        upstream_is_spawner=True, downstream_is_remover=True)

    # Add vehicles to lane
    rl.vehicles.extend((vehA, vehB))

    # Too close, should brake
    vehA.velocity = SHARED.SETTINGS.speed_limit - 0.001
    vehB.velocity = SHARED.SETTINGS.speed_limit
    rl.vehicle_progress[vehA] = VehicleProgress(0.02, 0.015, 0.01)
    rl.vehicle_progress[vehB] = VehicleProgress(0.011, 0.0051, 0.001)
    new_speeds = rl.get_new_speeds()
    assert new_speeds[vehA] == SpeedUpdate(SHARED.SETTINGS.speed_limit,
                                           vehA.max_acceleration)
    assert new_speeds[vehB].velocity < SHARED.SETTINGS.speed_limit
    assert new_speeds[vehB].acceleration == vehB.max_braking

    # Nice and far, should accelerate
    vehA.velocity = SHARED.SETTINGS.speed_limit
    vehB.velocity = 0
    rl.vehicle_progress[vehA] = VehicleProgress(0.99, 0.985, 0.98)
    rl.vehicle_progress[vehB] = VehicleProgress(0.011, 0.0051, 0.001)
    new_speeds = rl.get_new_speeds()
    assert new_speeds[vehA] == SpeedUpdate(SHARED.SETTINGS.speed_limit, 0)
    assert new_speeds[vehB].velocity > 0
    assert new_speeds[vehB].acceleration == vehB.max_acceleration

    # Tight, but should still accelerate
    vehA.velocity = SHARED.SETTINGS.speed_limit
    vehB.velocity = SHARED.SETTINGS.speed_limit - 1
    rl.vehicle_progress[vehA] = VehicleProgress(0.02, 0.015, 0.01)
    rl.vehicle_progress[vehB] = VehicleProgress(0.01, 0.005, 0)
    new_speeds = rl.get_new_speeds()
    assert new_speeds[vehA] == SpeedUpdate(SHARED.SETTINGS.speed_limit, 0)
    assert new_speeds[vehB].velocity > SHARED.SETTINGS.speed_limit - 1
    assert new_speeds[vehB].acceleration == vehB.max_acceleration

    # Exactly tight, maintain speed
    vehA.velocity = SHARED.SETTINGS.speed_limit
    vehB.velocity = SHARED.SETTINGS.speed_limit
    rl.vehicle_progress[vehA] = VehicleProgress(0.02, 0.015, 0.01)
    rl.vehicle_progress[vehB] = VehicleProgress(0.01, 0.005, 0)
    new_speeds = rl.get_new_speeds()
    assert new_speeds[vehA] == SpeedUpdate(SHARED.SETTINGS.speed_limit, 0)
    assert new_speeds[vehB] == SpeedUpdate(SHARED.SETTINGS.speed_limit, 0)
