from __future__ import annotations
from typing import Dict, Any, Tuple, List
from random import seed

from pytest import raises, fixture
from naaims.endpoints.remover import VehicleRemover

import naaims.shared as SHARED
from naaims.vehicles import Vehicle, AutomatedVehicle
from naaims.endpoints import VehicleSpawner
from naaims.util import Coord
from naaims.lane import VehicleProgress
from naaims.road import Road
from naaims.trajectories import Trajectory, BezierTrajectory
from naaims.endpoints.factories import VehicleFactory, GaussianVehicleFactory
from naaims.pathfinder import Pathfinder


@fixture
def trajectory(length: float = 100):
    return BezierTrajectory(Coord(0, 0), Coord(length, 0),
                            [Coord(length/2, 0)])


@fixture
def road(trajectory: Trajectory):
    road_spec: Dict[str, Any] = dict(
        id=0,
        upstream_id=0,
        downstream_id=0,
        trajectory=trajectory,
        num_lanes=2,
        lane_width=4,
        upstream_is_spawner=True,
        downstream_is_remover=True,
        lane_offset_angle=0,
        len_approach_region=0,
        len_entrance_region=trajectory.length,
        speed_limit=15
    )

    return Road.from_spec(road_spec)


@fixture
def set_pathfinder_straight(trajectory: Trajectory, road: Road):

    top_lane_end_coord = Coord(trajectory.length, 2)
    bottom_lane_end_coord = Coord(trajectory.length, -2)

    od_pair: Dict[Tuple[Coord, int], List[Coord]] = {
        (top_lane_end_coord, 1): [top_lane_end_coord],
        (bottom_lane_end_coord, 1): [bottom_lane_end_coord]
    }

    SHARED.SETTINGS.pathfinder = Pathfinder([road], [], od_pair)


@fixture
def set_pathfinder_split(trajectory: Trajectory, road: Road):

    top_lane_end_coord = Coord(trajectory.length, 2)
    bottom_lane_end_coord = Coord(trajectory.length, -2)

    od_pair: Dict[Tuple[Coord, int], List[Coord]] = {
        (top_lane_end_coord, 1): [top_lane_end_coord],
        (top_lane_end_coord, 2): [],
        (bottom_lane_end_coord, 1): [],
        (bottom_lane_end_coord, 2): [bottom_lane_end_coord]
    }

    SHARED.SETTINGS.pathfinder = Pathfinder([road], [], od_pair)


@fixture
def remover():
    remover_spec: Dict[str, Any] = dict(
        id=0,
        road_id=0
    )

    return VehicleRemover.from_spec(remover_spec)


gvf_spec_0: Dict[str, Any] = dict(
    vehicle_type=AutomatedVehicle,
    num_destinations=2,
    destination_probabilities=[0, 1],
    source_node_id=0,
    max_accel_mn=3,  # maximum acceleration, in m/s^2
    max_accel_sd=0,
    max_braking_mn=-3.4,  # or -4.5, braking in m/s^2
    max_braking_sd=0,
    length_mn=4.5,  # length in meters
    length_sd=0,
    width_mn=3,  # width in meters
    width_sd=0,
    throttle_score_mn=0,
    throttle_score_sd=0,
    tracking_score_mn=0,
    tracking_score_sd=0,
    vot_mn=0,  # value of time
    vot_sd=0
)


def check_spawner_init_basics(spawner: VehicleSpawner, road: Road):
    assert spawner.downstream is road
    assert len(spawner.queue) == 0


def test_poisson_init(load_shared: None, road: Road):
    spawner = VehicleSpawner(road, 1, [GaussianVehicleFactory],
                             [gvf_spec_0], [1])
    check_spawner_init_basics(spawner, road)
    assert len(spawner.predetermined_spawns) == 0
    assert spawner.spawn_probability == 1/60**2
    assert sum(spawner.factory_selection_probabilities) == 1
    assert len(spawner.factories) == 1
    assert isinstance(spawner.factories[0], VehicleFactory)

    with raises(ValueError):
        VehicleSpawner(road, 1, [GaussianVehicleFactory], [gvf_spec_0], [])

    with raises(ValueError):
        VehicleSpawner(road, 1, [GaussianVehicleFactory], [], [1])

    with raises(ValueError):
        VehicleSpawner(road, 1, [], [gvf_spec_0], [2])


def test_predetermined_init(load_shared: None, road: Road, vehicle: Vehicle,
                            vehicle2: Vehicle, vehicle3: Vehicle):
    spawner = VehicleSpawner(road, 0, [], [], [], [
        (2.1, vehicle), (0.6, vehicle2), (9.9, vehicle3)])
    check_spawner_init_basics(spawner, road)
    assert spawner.spawn_probability == 0
    assert sum(spawner.factory_selection_probabilities) == 0
    assert len(spawner.factories) == 0
    assert len(spawner.predetermined_spawns) == 3
    assert spawner.predetermined_spawns[0] == (
        0.6/SHARED.SETTINGS.TIMESTEP_LENGTH, vehicle2)
    assert spawner.predetermined_spawns[1] == (
        2.1/SHARED.SETTINGS.TIMESTEP_LENGTH, vehicle)
    assert spawner.predetermined_spawns[2] == (
        9.9/SHARED.SETTINGS.TIMESTEP_LENGTH, vehicle3)


def test_poisson_spawn(load_shared_clean: None, road: Road,
                       set_pathfinder_straight: None):
    spawner = VehicleSpawner(road, 99999, [GaussianVehicleFactory],
                             [gvf_spec_0], [1])
    seed(0)
    spawned, entered = spawner.step_vehicles()
    assert len(spawned) == len(entered) == 1
    spawn = spawned[0]
    enter = entered[0]
    assert spawn is enter
    assert spawn.vin == 0
    assert spawn.vot == 0
    assert spawn.velocity == spawn.acceleration == 0
    assert len(road.entering_vehicle_buffer) == 3
    assert road.entering_vehicle_buffer[0].vehicle is \
        road.entering_vehicle_buffer[1].vehicle is \
        road.entering_vehicle_buffer[2].vehicle is spawn
    assert road.entering_vehicle_buffer[0].distance_left is \
        road.entering_vehicle_buffer[1].distance_left is \
        road.entering_vehicle_buffer[2].distance_left is None
    assert road.entering_vehicle_buffer[0].pos == \
        road.entering_vehicle_buffer[1].pos == \
        road.entering_vehicle_buffer[2].pos == Coord(0, 2)


def test_predetermined_spawn(load_shared_clean: None, road: Road,
                             set_pathfinder_straight: None, vehicle: Vehicle,
                             vehicle2: Vehicle, vehicle3: Vehicle):
    vehicle._Vehicle__destination = vehicle2._Vehicle__destination = 1
    spawner = VehicleSpawner(road, 0, [], [], [], [
        (0, vehicle), (0, vehicle2), (9.9, vehicle3)])
    seed(0)
    assert len(spawner.predetermined_spawns) == 3
    assert len(spawner.queue) == 0
    spawned, entered = spawner.step_vehicles()
    assert len(spawned) == len(entered) == 2
    assert vehicle in spawned
    assert vehicle2 in spawned
    assert vehicle in entered
    assert vehicle2 in entered
    assert len(road.entering_vehicle_buffer) == 6
    for i, veh in enumerate([vehicle, vehicle2]):
        assert road.entering_vehicle_buffer[3*i + 0].vehicle is \
            road.entering_vehicle_buffer[3*i + 1].vehicle is \
            road.entering_vehicle_buffer[3*i + 2].vehicle is veh
        assert road.entering_vehicle_buffer[3*i + 0].distance_left is \
            road.entering_vehicle_buffer[3*i + 1].distance_left is \
            road.entering_vehicle_buffer[3*i + 2].distance_left is None
        assert road.entering_vehicle_buffer[3*i + 0].pos == \
            road.entering_vehicle_buffer[3*i + 1].pos == \
            road.entering_vehicle_buffer[3*i + 2].pos == Coord(
                0, 2 * (-1 if i == 0 else 1))

    assert len(spawner.predetermined_spawns) == 1
    assert spawner.predetermined_spawns[0] == (
        9.9/SHARED.SETTINGS.TIMESTEP_LENGTH, vehicle3)


def test_multi_spawn(load_shared_clean: None, road: Road,
                     set_pathfinder_straight: None, vehicle: Vehicle,
                     vehicle2: Vehicle, vehicle3: Vehicle):
    vehicle._Vehicle__destination = vehicle2._Vehicle__destination = \
        vehicle3._Vehicle__destination = 1
    spawner = VehicleSpawner(road, 0, [], [], [], [
        (0, vehicle), (0, vehicle2), (0, vehicle3)])
    seed(0)
    assert len(spawner.predetermined_spawns) == 3
    assert len(spawner.queue) == 0
    spawned, entered = spawner.step_vehicles()
    assert len(spawned) == 3
    assert len(entered) == 2
    assert vehicle in spawned
    assert vehicle2 in spawned
    assert vehicle3 in spawned
    assert vehicle in entered
    assert vehicle2 in entered
    assert len(road.entering_vehicle_buffer) == 6
    for i, veh in enumerate([vehicle, vehicle2]):
        assert road.entering_vehicle_buffer[3*i + 0].vehicle is \
            road.entering_vehicle_buffer[3*i + 1].vehicle is \
            road.entering_vehicle_buffer[3*i + 2].vehicle is veh
        assert road.entering_vehicle_buffer[3*i + 0].distance_left is \
            road.entering_vehicle_buffer[3*i + 1].distance_left is \
            road.entering_vehicle_buffer[3*i + 2].distance_left is None
        assert road.entering_vehicle_buffer[3*i + 0].pos == \
            road.entering_vehicle_buffer[3*i + 1].pos == \
            road.entering_vehicle_buffer[3*i + 2].pos == Coord(
                0, 2 * (-1 if i == 0 else 1))

    assert len(spawner.predetermined_spawns) == 0
    assert len(spawner.queue) == 1
    assert spawner.queue[0][0] is vehicle3


def test_spawn_lane(load_shared_clean: None, road: Road,
                    set_pathfinder_split: None, vehicle: Vehicle,
                    vehicle2: Vehicle):
    vehicle._Vehicle__destination = 1
    vehicle2._Vehicle__destination = 2
    spawner = VehicleSpawner(road, 0, [], [], [], [(0, vehicle),
                                                   (0, vehicle2)])
    seed(0)
    assert len(spawner.predetermined_spawns) == 2
    assert len(spawner.queue) == 0
    spawned, entered = spawner.step_vehicles()
    assert len(spawned) == len(entered) == 2
    for transfer in road.entering_vehicle_buffer:
        if transfer.vehicle is vehicle:
            assert transfer.pos == Coord(0, 2)
        else:
            assert transfer.pos == Coord(0, -2)


def test_lane_spawn_block(load_shared_clean: None, road: Road,
                          set_pathfinder_split: None, vehicle: Vehicle,
                          vehicle2: Vehicle, vehicle3: Vehicle):

    # Spawn two vehicles that want to use the same lane, and a third that
    # doesn't care.
    vehicle._Vehicle__destination = vehicle2._Vehicle__destination = 1
    vehicle3._Vehicle__destination = 2
    spawner = VehicleSpawner(road, 0, [], [], [], [
        (0, vehicle), (0, vehicle2), (0, vehicle3)])
    seed(0)
    assert len(spawner.predetermined_spawns) == 3
    assert len(spawner.queue) == 0
    spawned, entered = spawner.step_vehicles()

    # Assert that both spawn but only one of them is able to enter. Assert that
    # the third vehicle sees no issues.
    assert len(spawned) == 3
    assert len(entered) == 2
    assert vehicle in spawned
    assert vehicle2 in spawned
    assert vehicle3 in spawned
    assert vehicle in entered
    assert vehicle3 in entered
    assert len(road.entering_vehicle_buffer) == 6
    for i, veh in enumerate([vehicle, vehicle3]):
        assert road.entering_vehicle_buffer[3*i + 0].vehicle is \
            road.entering_vehicle_buffer[3*i + 1].vehicle is \
            road.entering_vehicle_buffer[3*i + 2].vehicle is veh
        assert road.entering_vehicle_buffer[3*i + 0].distance_left is \
            road.entering_vehicle_buffer[3*i + 1].distance_left is \
            road.entering_vehicle_buffer[3*i + 2].distance_left is None
        assert road.entering_vehicle_buffer[3*i + 0].pos == \
            road.entering_vehicle_buffer[3*i + 1].pos == \
            road.entering_vehicle_buffer[3*i + 2].pos == Coord(
                0, 2 * (1 if i == 0 else -1))

    assert len(spawner.predetermined_spawns) == 0
    assert len(spawner.queue) == 1
    assert spawner.queue[0][0] is vehicle2


def test_lane_preblock(load_shared_clean: None, road: Road,
                       set_pathfinder_split: None, vehicle: Vehicle,
                       vehicle2: Vehicle, vehicle3: Vehicle):

    # Block the y = -2 lane before any vehicles can spawn.
    road.lanes[0].vehicles = [vehicle3]
    road.lanes[0].vehicle_progress[vehicle3] = VehicleProgress(0, 0, 0)

    # Spawn a vehicle that wants to use the blocked lane and another that
    # doesn't care.
    vehicle._Vehicle__destination = 1
    vehicle2._Vehicle__destination = 2
    spawner = VehicleSpawner(road, 0, [], [], [], [
        (0, vehicle), (0, vehicle2)])
    seed(0)
    assert len(spawner.predetermined_spawns) == 2
    assert len(spawner.queue) == 0
    spawned, entered = spawner.step_vehicles()

    # Assert that the first vehicle is unable to enter but the
    # second vehicle sees no issues.
    assert len(spawned) == 2
    assert entered == [vehicle]
    assert vehicle in spawned
    assert vehicle2 in spawned
    assert len(road.entering_vehicle_buffer) == 3
    assert road.entering_vehicle_buffer[0].vehicle is \
        road.entering_vehicle_buffer[1].vehicle is \
        road.entering_vehicle_buffer[2].vehicle is vehicle
    assert road.entering_vehicle_buffer[0].distance_left is \
        road.entering_vehicle_buffer[1].distance_left is \
        road.entering_vehicle_buffer[2].distance_left is None
    assert road.entering_vehicle_buffer[0].pos == \
        road.entering_vehicle_buffer[1].pos == \
        road.entering_vehicle_buffer[2].pos == Coord(0, 2)

    assert len(spawner.predetermined_spawns) == 0
    assert len(spawner.queue) == 1
    assert spawner.queue[0][0] is vehicle2
