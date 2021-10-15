from math import pi

from pytest import raises, approx
from pytest_mock import MockerFixture

import naaims.shared as SHARED
from naaims.lane import Lane, VehicleProgress
from naaims.road import RoadLane
from naaims.intersection import IntersectionLane
from naaims.trajectories import BezierTrajectory
from naaims.util import Coord, SpeedUpdate, VehicleTransfer, VehicleSection
from naaims.vehicles import Vehicle, AutomatedVehicle

straight_trajectory = BezierTrajectory(
    Coord(0, 0), Coord(1000, 0), [Coord(500, 0)])


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
    assert vehicle.acceleration == SHARED.SETTINGS.min_acceleration
    assert vehicle.velocity == SHARED.SETTINGS.min_acceleration * 1/60


def test_vehicle_entry_from_spawner(mocker: MockerFixture, load_shared: None):

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
        vehicle_test.length*.6 + 3/2 *
        SHARED.SETTINGS.min_acceleration*SHARED.SETTINGS.TIMESTEP_LENGTH**2)


def test_vehicle_entry_from_facility(mocker: MockerFixture, load_shared: None):

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


def test_vehicle_control(mocker: MockerFixture, load_shared: None):

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


def test_consecutive_vehicles(mocker: MockerFixture, load_shared: None):
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
                                           SHARED.SETTINGS.min_acceleration)
    assert new_speeds[vehB].velocity < SHARED.SETTINGS.speed_limit
    assert new_speeds[vehB].acceleration == SHARED.SETTINGS.min_braking

    # Nice and far, should accelerate
    vehA.velocity = SHARED.SETTINGS.speed_limit
    vehB.velocity = 0
    rl.vehicle_progress[vehA] = VehicleProgress(0.99, 0.985, 0.98)
    rl.vehicle_progress[vehB] = VehicleProgress(0.011, 0.0051, 0.001)
    new_speeds = rl.get_new_speeds()
    assert new_speeds[vehA] == SpeedUpdate(SHARED.SETTINGS.speed_limit, 0)
    assert new_speeds[vehB].velocity > 0
    assert new_speeds[vehB].acceleration == SHARED.SETTINGS.min_acceleration

    # Tight, but should still accelerate
    vehA.velocity = SHARED.SETTINGS.speed_limit
    vehB.velocity = SHARED.SETTINGS.speed_limit - 1
    rl.vehicle_progress[vehA] = VehicleProgress(0.02, 0.015, 0.01)
    rl.vehicle_progress[vehB] = VehicleProgress(0.01, 0.005, 0)
    new_speeds = rl.get_new_speeds()
    assert new_speeds[vehA] == SpeedUpdate(SHARED.SETTINGS.speed_limit, 0)
    assert new_speeds[vehB].velocity > SHARED.SETTINGS.speed_limit - 1
    assert new_speeds[vehB].acceleration == SHARED.SETTINGS.min_acceleration

    # Exactly tight, but tolerance is too close, brake
    vehA.velocity = SHARED.SETTINGS.speed_limit
    vehB.velocity = SHARED.SETTINGS.speed_limit
    rl.vehicle_progress[vehA] = VehicleProgress(0.02, 0.015, 0.01)
    rl.vehicle_progress[vehB] = VehicleProgress(0.01, 0.005, 0)
    new_speeds = rl.get_new_speeds()
    assert new_speeds[vehA] == SpeedUpdate(SHARED.SETTINGS.speed_limit, 0)
    assert new_speeds[vehB] == SpeedUpdate(
        SHARED.SETTINGS.speed_limit +
        SHARED.SETTINGS.TIMESTEP_LENGTH*SHARED.SETTINGS.min_braking,
        SHARED.SETTINGS.min_braking)

    # A little less than exactly tight, maintain speed
    vehA.velocity = SHARED.SETTINGS.speed_limit
    vehB.velocity = SHARED.SETTINGS.speed_limit
    rl.vehicle_progress[vehA] = VehicleProgress(0.02, 0.015, 0.01)
    rl.vehicle_progress[vehB] = VehicleProgress(0.005, 0, None)
    new_speeds = rl.get_new_speeds()
    assert new_speeds[vehA] == SpeedUpdate(SHARED.SETTINGS.speed_limit, 0)
    assert new_speeds[vehB] == SpeedUpdate(SHARED.SETTINGS.speed_limit, 0)


def test_t_to_v():
    assert RoadLane.t_to_v(1, 2, 10) == 4.5
    assert RoadLane.t_to_v(10, -2, 1) == 4.5
    assert RoadLane.t_to_v(5.5, .5, 6.6) == approx(1.1/.5)
    assert RoadLane.t_to_v(6.6, -.5, 5.5) == approx(1.1/.5)


def test_x_constant_a():
    assert RoadLane.x_over_constant_a(10, 1, 2) == 22
    assert RoadLane.x_over_constant_a(10, -1, 2) == 18
    assert RoadLane.x_over_constant_a(10, 1, 1.1) == approx(11+1.21/2)
    assert RoadLane.x_over_constant_a(10, -1, 1.1) == approx(11-1.21/2)


def test_effective_speed_limit(rl: RoadLane, vehicle: AutomatedVehicle):
    assert rl.effective_speed_limit(0, vehicle) == rl.speed_limit
    assert rl.effective_speed_limit(0.5, vehicle) == rl.speed_limit
    assert rl.effective_speed_limit(1, vehicle) == rl.speed_limit


def test_effective_stopping_distance(rl: RoadLane):
    assert rl.available_stopping_distance(0, 0, 10) == 10
    assert rl.available_stopping_distance(0.1, 0, 10) == 110


def test_sd_to_last(rl: RoadLane, vehicle: AutomatedVehicle,
                    vehicle2: AutomatedVehicle):
    # No vehicles, no distance
    assert rl.stopping_distance_to_last_vehicle() is None

    # One vehicle, still entering
    rl.vehicles.append(vehicle)
    rl.vehicle_progress[vehicle] = VehicleProgress(0.02, 0.02, None)
    assert rl.stopping_distance_to_last_vehicle() is None

    # One vehicle, entered
    rl.vehicle_progress[vehicle] = VehicleProgress(0.02, 0.02, 0.02)
    vehicle.velocity = 10
    assert rl.stopping_distance_to_last_vehicle() == 20 + \
        vehicle.stopping_distance()

    # Two vehicles, one entering
    rl.vehicles.append(vehicle2)
    rl.vehicle_progress[vehicle2] = VehicleProgress(0.01, 0.01, None)
    assert rl.stopping_distance_to_last_vehicle() == 10 + \
        vehicle.stopping_distance()


def test_accel_uncontested(rl: RoadLane, vehicle: AutomatedVehicle):
    vehicle.velocity = 0
    assert rl.accel_update_uncontested(vehicle, .5) == \
        SHARED.SETTINGS.min_acceleration
    assert rl.accel_update_uncontested(vehicle, 0) == \
        SHARED.SETTINGS.min_acceleration
    assert rl.accel_update_uncontested(vehicle, 1) == \
        SHARED.SETTINGS.min_acceleration

    vehicle.velocity = rl.speed_limit
    assert rl.accel_update_uncontested(vehicle, .5) == 0
    assert rl.accel_update_uncontested(vehicle, 0) == 0
    assert rl.accel_update_uncontested(vehicle, 1) == 0

    vehicle.velocity = rl.speed_limit + 1
    assert rl.accel_update_uncontested(vehicle, .5) == \
        SHARED.SETTINGS.min_braking
    assert rl.accel_update_uncontested(vehicle, 0) == \
        SHARED.SETTINGS.min_braking
    assert rl.accel_update_uncontested(vehicle, 1) == \
        SHARED.SETTINGS.min_braking


def test_accel_following(rl: RoadLane, vehicle: AutomatedVehicle):
    vehicle.velocity = rl.speed_limit
    assert rl.accel_update_following(vehicle, 1) == SHARED.SETTINGS.min_braking
    assert rl.accel_update_following(vehicle, 0) == 0
    brake_threshold = 1-vehicle.stopping_distance()/rl.trajectory.length
    assert rl.accel_update_following(vehicle, brake_threshold
                                     ) == SHARED.SETTINGS.min_braking
    # Have enough room to not brake but not enough to not accel
    assert rl.accel_update_following(vehicle, brake_threshold*.8) == 0
    # Same room, but you're told that you have less room to brake than is left
    # on the lane (mocking as if there's another car ahead)
    assert rl.accel_update_following(vehicle, brake_threshold-1e-6,
                                     (1-brake_threshold)*rl.trajectory.length/2
                                     ) == SHARED.SETTINGS.min_braking

    vehicle.velocity = rl.speed_limit/2
    assert rl.accel_update_following(vehicle, 1) == SHARED.SETTINGS.min_braking
    assert rl.accel_update_following(vehicle, 0
                                     ) == SHARED.SETTINGS.min_acceleration
    brake_threshold_half = 1-vehicle.stopping_distance()/rl.trajectory.length
    assert rl.accel_update_following(vehicle, brake_threshold_half
                                     ) == SHARED.SETTINGS.min_braking
    # Holding speed keeps vehicle in no-crash range, but accelerating doesn't
    assert rl.accel_update_following(vehicle, brake_threshold_half*.9995) == 0
    # Accelerating keeps vehicle in no-crash range
    assert rl.accel_update_following(vehicle, brake_threshold_half*.9
                                     ) == SHARED.SETTINGS.min_acceleration

    vehicle.velocity = rl.speed_limit+1
    assert rl.accel_update_following(vehicle, 1) == SHARED.SETTINGS.min_braking
    assert rl.accel_update_following(vehicle, 0) == SHARED.SETTINGS.min_braking


def test_speed_update(rl: RoadLane, vehicle: AutomatedVehicle):

    # Starting at 0 velocity
    su = rl.speed_update(vehicle, 0, 1)
    assert su.velocity == SHARED.SETTINGS.TIMESTEP_LENGTH
    assert su.acceleration == 1
    su = rl.speed_update(vehicle, 1, 1)
    assert su.velocity == SHARED.SETTINGS.TIMESTEP_LENGTH
    assert su.acceleration == 1

    # At speed limit, going over should be ignored
    vehicle.velocity = rl.speed_limit
    su = rl.speed_update(vehicle, 0, -1)
    assert su.velocity == rl.speed_limit - SHARED.SETTINGS.TIMESTEP_LENGTH
    assert su.acceleration == -1
    su = rl.speed_update(vehicle, 1, -1)
    assert su.velocity == rl.speed_limit - SHARED.SETTINGS.TIMESTEP_LENGTH
    assert su.acceleration == -1
    # Going over should be ignored
    su = rl.speed_update(vehicle, 0, 1)
    assert su.velocity == rl.speed_limit
    assert su.acceleration == 0
    su = rl.speed_update(vehicle, 1, 1)
    assert su.velocity == rl.speed_limit
    assert su.acceleration == 0

    # Just under speed limit, speed should be clipped to speed limit
    vehicle.velocity = rl.speed_limit - 1e-6
    su = rl.speed_update(vehicle, 0, 1)
    assert su.velocity == rl.speed_limit
    assert su.acceleration == 1
    su = rl.speed_update(vehicle, 1, 1)
    assert su.velocity == rl.speed_limit
    assert su.acceleration == 1


def test_progress_update(rl: RoadLane, vehicle: AutomatedVehicle):
    vehicle.velocity = rl.speed_limit
    distance_in_1t = (rl.speed_limit * SHARED.SETTINGS.TIMESTEP_LENGTH)
    p_in_1t = distance_in_1t / rl.trajectory.length

    # Base case, no exits, all in
    p_new, p_rear, transfers = rl.update_vehicle_progress(
        vehicle, VehicleProgress(.5, .5, .5))
    assert len(transfers) == 0
    assert p_rear == p_new.front == p_new.center == p_new.rear == .5 + p_in_1t

    # Front exits
    p_new, p_rear, transfers = rl.update_vehicle_progress(
        vehicle, VehicleProgress(1, .5, .5))
    assert len(transfers) == 1
    assert transfers[0].vehicle == vehicle
    assert transfers[0].section == VehicleSection.FRONT
    assert transfers[0].distance_left == approx(distance_in_1t)
    assert transfers[0].pos == rl.trajectory.end_coord
    assert p_new.front is None
    assert p_rear == p_new.center == p_new.rear == .5 + p_in_1t

    # Front exits but center and rear aren't in
    with raises(RuntimeError):
        rl.update_vehicle_progress(vehicle, VehicleProgress(1, None, None))

    # Front and center exit
    p_new, p_rear, transfers = rl.update_vehicle_progress(
        vehicle, VehicleProgress(1, 1, .5))
    assert len(transfers) == 2
    assert transfers[0].vehicle == transfers[1].vehicle == vehicle
    assert transfers[0].section == VehicleSection.FRONT
    assert transfers[1].section == VehicleSection.CENTER
    assert transfers[0].distance_left == transfers[1].distance_left == \
        approx(distance_in_1t)
    assert transfers[0].pos == transfers[1].pos == rl.trajectory.end_coord
    assert p_new.front is None
    assert p_new.center is None
    assert p_rear == p_new.rear == .5 + p_in_1t

    # Front and center exit but rear isn't in
    with raises(RuntimeError):
        rl.update_vehicle_progress(vehicle, VehicleProgress(1, 1, None))

    # All three exit
    p_new, p_rear, transfers = rl.update_vehicle_progress(
        vehicle, VehicleProgress(1, 1, 1))
    assert len(transfers) == 3
    assert transfers[0].vehicle == transfers[1].vehicle == \
        transfers[1].vehicle == vehicle
    assert transfers[0].section == VehicleSection.FRONT
    assert transfers[1].section == VehicleSection.CENTER
    assert transfers[2].section == VehicleSection.REAR
    assert transfers[0].distance_left == transfers[1].distance_left == \
        transfers[2].distance_left == approx(distance_in_1t)
    assert transfers[0].pos == transfers[1].pos == transfers[2].pos == \
        rl.trajectory.end_coord
    assert p_new.front is None
    assert p_new.center is None
    assert p_new.rear is None
    assert p_rear == 1

    # Front had already exited
    p_new, p_rear, transfers = rl.update_vehicle_progress(
        vehicle, VehicleProgress(None, .5, .5))
    assert len(transfers) == 0
    assert p_new.front is None
    assert p_rear == p_new.center == p_new.rear == .5 + p_in_1t

    # Front had already exited, center exits
    p_new, p_rear, transfers = rl.update_vehicle_progress(
        vehicle, VehicleProgress(None, 1, .5))
    assert len(transfers) == 1
    assert transfers[0].vehicle == vehicle
    assert transfers[0].section == VehicleSection.CENTER
    assert transfers[0].distance_left == approx(distance_in_1t)
    assert transfers[0].pos == rl.trajectory.end_coord
    assert p_new.front is None
    assert p_rear == p_new.rear == .5 + p_in_1t

    # Front and center had already exited
    p_new, p_rear, transfers = rl.update_vehicle_progress(
        vehicle, VehicleProgress(None, None, .5))
    assert len(transfers) == 0
    assert p_new.front is None
    assert p_new.center is None
    assert p_rear == p_new.rear == .5 + p_in_1t

    # Front and center had already exited, rear exits
    p_new, p_rear, transfers = rl.update_vehicle_progress(
        vehicle, VehicleProgress(None, None, 1))
    assert len(transfers) == 1
    assert transfers[0].vehicle == vehicle
    assert transfers[0].section == VehicleSection.REAR
    assert transfers[0].distance_left == approx(distance_in_1t)
    assert transfers[0].pos == rl.trajectory.end_coord
    assert p_new.front is None
    assert p_new.center is None
    assert p_new.rear is None
    assert p_rear == 1

    # All already exited
    with raises(RuntimeError):
        rl.update_vehicle_progress(vehicle, VehicleProgress(None, None, None))

    # Only front in
    p_new, p_rear, transfers = rl.update_vehicle_progress(
        vehicle, VehicleProgress(.5, None, None))
    assert len(transfers) == 0
    assert p_new.center is None
    assert p_new.rear is None
    assert p_new.front == .5 + p_in_1t
    assert p_rear == -1

    # Front and center in
    p_new, p_rear, transfers = rl.update_vehicle_progress(
        vehicle, VehicleProgress(.5, .5, None))
    assert len(transfers) == 0
    assert p_new.rear is None
    assert p_new.front == p_new.center == .5 + p_in_1t
    assert p_rear == -1


def test_pos_update(rl: RoadLane, vehicle: AutomatedVehicle):
    rl.update_vehicle_position(vehicle, .5)
    assert vehicle.pos == Coord(500, 0)
    assert vehicle.heading == 0
    rl.update_vehicle_position(vehicle, 1)
    assert vehicle.pos == Coord(1000, 0)
    assert vehicle.heading == 0

    rl.trajectory = BezierTrajectory(Coord(0, 0), Coord(-1000, 0),
                                     [Coord(-500, 0)])
    rl.update_vehicle_position(vehicle, .5)
    assert vehicle.pos == Coord(-500, 0)
    assert vehicle.heading == pi


def test_exit_check(rl: RoadLane):
    assert rl.has_vehicle_exited(VehicleProgress(None, None, None))
    assert not rl.has_vehicle_exited(VehicleProgress(1, 1, 1))
    # TODO: Replace with IntersectionLane since RoadLane overrides this.


def test_vehicle_add(rl: RoadLane, vehicle: AutomatedVehicle):
    assert len(rl.vehicles) == 0
    assert len(rl.vehicle_progress) == 0
    rl.add_vehicle(vehicle)
    assert rl.vehicles[0] is vehicle
    assert vehicle in rl.vehicle_progress


def test_vehicle_removal(rl: RoadLane, vehicle: AutomatedVehicle,
                         vehicle2: AutomatedVehicle):
    rl.vehicles.append(vehicle)
    rl.vehicle_progress[vehicle] = VehicleProgress()
    rl.vehicles.append(vehicle2)
    rl.vehicle_progress[vehicle2] = VehicleProgress()
    rl.remove_vehicle(vehicle)
    assert len(rl.vehicles) == 1
    assert len(rl.vehicle_progress) == 1
    assert rl.vehicles[0] is vehicle2
    assert vehicle not in rl.vehicle_progress
    assert vehicle2 in rl.vehicle_progress
