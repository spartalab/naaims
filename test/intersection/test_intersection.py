from math import pi

from pytest import approx

from aimsim.intersection import Intersection
from aimsim.road import Road
from aimsim.util import Coord, VehicleSection, VehicleTransfer, SpeedUpdate
from aimsim.trajectories import BezierTrajectory
from aimsim.intersection.managers import StopSignManager
from aimsim.intersection.tilings import SquareTiling
from aimsim.intersection.tilings.tiles import DeterministicTile
from aimsim.vehicles import AutomatedVehicle

import aimsim.shared as SHARED


def test_init_2_lane(read_config: None):
    trajectory1 = BezierTrajectory(Coord(0, 0), Coord(100, 0),
                                   [Coord(50, 0)])
    trajectory2 = BezierTrajectory(Coord(200, 0), Coord(300, 0),
                                   [Coord(250, 0)])

    # Create IO roads
    road_in = Road(trajectory1, .2*trajectory1.length,
                   SHARED.SETTINGS.speed_limit,
                   upstream_is_spawner=True, downstream_is_remover=True,
                   num_lanes=2, lane_offset_angle=pi/2,
                   len_approach_region=.7*trajectory1.length)
    road_out = Road(trajectory2, .2*trajectory2.length,
                    SHARED.SETTINGS.speed_limit,
                    upstream_is_spawner=True, downstream_is_remover=True,
                    num_lanes=2, lane_offset_angle=pi/2,
                    len_approach_region=.7*trajectory1.length)

    intersection = Intersection([road_in], [road_out],
                                [(road_in, road_out, True)],
                                manager_type=StopSignManager,
                                manager_spec={
                                    'tiling_type': SquareTiling,
                                    'tiling_spec': {
                                        'tile_type': DeterministicTile,
                                        'misc_spec': {
                                            'tile_width': 5
                                        }
                                    }
    },
        speed_limit=SHARED.SETTINGS.speed_limit)

    for lane in road_in.lanes:
        assert intersection.incoming_road_lane_by_coord[
            lane.trajectory.end_coord] is lane

    for lane in road_out.lanes:
        assert intersection.outgoing_road_lane_by_coord[
            lane.trajectory.start_coord] is lane
        assert intersection.outgoing_road_by_lane_coord[
            lane.trajectory.start_coord] is road_out

    # Test that the IO lanes got connected correctly
    assert len(intersection.lanes) == 2
    lane0traj = intersection.lanes[0].trajectory
    lane1traj = intersection.lanes[1].trajectory
    assert lane0traj.start_coord == road_in.lanes[1].trajectory.end_coord
    assert lane0traj.end_coord == road_out.lanes[1].trajectory.start_coord
    assert lane0traj.control_coord == (
        (lane0traj.end_coord.x - lane0traj.start_coord.x)/2,
        lane0traj.start_coord.y)
    assert lane1traj.start_coord == road_in.lanes[0].trajectory.end_coord
    assert lane1traj.end_coord == road_out.lanes[0].trajectory.start_coord
    assert lane1traj.control_coord == (
        (lane1traj.end_coord.x - lane1traj.start_coord.x)/2,
        lane1traj.start_coord.y)

    assert type(intersection.manager) is StopSignManager
    assert type(intersection.manager.tiling) is SquareTiling
    assert type(intersection.manager.tiling.tile_type) is type(
        DeterministicTile)
    assert intersection.manager.tiling.tile_width == 5


def test_init_3_lane(read_config: None):
    trajectory1 = BezierTrajectory(Coord(0, 0), Coord(100, 0),
                                   [Coord(50, 0)])
    trajectory2 = BezierTrajectory(Coord(200, 0), Coord(300, 0),
                                   [Coord(250, 0)])

    # Create IO roads
    road_in = Road(trajectory1, .2*trajectory1.length,
                   SHARED.SETTINGS.speed_limit,
                   upstream_is_spawner=True, downstream_is_remover=True,
                   num_lanes=3, lane_offset_angle=pi/2,
                   len_approach_region=.7*trajectory1.length)
    road_out = Road(trajectory2, .2*trajectory2.length,
                    SHARED.SETTINGS.speed_limit,
                    upstream_is_spawner=True, downstream_is_remover=True,
                    num_lanes=3, lane_offset_angle=pi/2,
                    len_approach_region=.7*trajectory1.length)

    intersection = Intersection([road_in], [road_out],
                                [(road_in, road_out, True)],
                                manager_type=StopSignManager,
                                manager_spec={
                                    'tiling_type': SquareTiling,
                                    'tiling_spec': {
                                        'tile_type': DeterministicTile,
                                        'misc_spec': {
                                            'tile_width': 5
                                        }
                                    }
    },
        speed_limit=SHARED.SETTINGS.speed_limit)

    # Test that the IO lanes got connected correctly
    assert len(intersection.lanes) == 3
    assert intersection.lanes[0].trajectory.start_coord == \
        road_in.lanes[0].trajectory.end_coord
    assert intersection.lanes[0].trajectory.end_coord == \
        road_out.lanes[0].trajectory.start_coord
    assert intersection.lanes[1].trajectory.start_coord == \
        road_in.lanes[1].trajectory.end_coord
    assert intersection.lanes[1].trajectory.end_coord == \
        road_out.lanes[1].trajectory.start_coord
    assert intersection.lanes[2].trajectory.start_coord == \
        road_in.lanes[2].trajectory.end_coord
    assert intersection.lanes[2].trajectory.end_coord == \
        road_out.lanes[2].trajectory.start_coord


def test_init_right_turn(read_config: None):
    trajectory1 = BezierTrajectory(Coord(0, 0), Coord(100, 0),
                                   [Coord(50, 0)])
    trajectory2 = BezierTrajectory(Coord(150, -50), Coord(150, -150),
                                   [Coord(150, -100)])

    # Create IO roads
    road_in = Road(trajectory1, .2*trajectory1.length,
                   SHARED.SETTINGS.speed_limit,
                   upstream_is_spawner=True, downstream_is_remover=True,
                   num_lanes=2, lane_offset_angle=pi/2,
                   len_approach_region=.7*trajectory1.length)
    road_out = Road(trajectory2, .2*trajectory2.length,
                    SHARED.SETTINGS.speed_limit,
                    upstream_is_spawner=True, downstream_is_remover=True,
                    num_lanes=2, lane_offset_angle=pi/2,
                    len_approach_region=.7*trajectory1.length)

    intersection = Intersection([road_in], [road_out],
                                [(road_in, road_out, False)],
                                manager_type=StopSignManager,
                                manager_spec={
                                    'tiling_type': SquareTiling,
                                    'tiling_spec': {
                                        'tile_type': DeterministicTile,
                                        'misc_spec': {
                                            'tile_width': 5
                                        }
                                    }
    },
        speed_limit=SHARED.SETTINGS.speed_limit)

    # Test that the IO lanes got connected correctly
    assert len(intersection.lanes) == 1
    assert intersection.lanes[0].trajectory.start_coord == \
        road_in.lanes[0].trajectory.end_coord
    assert intersection.lanes[0].trajectory.end_coord == \
        road_out.lanes[1].trajectory.start_coord
    assert intersection.lanes[0].trajectory.control_coord == (
        road_out.lanes[1].trajectory.start_coord.x,
        road_in.lanes[0].trajectory.end_coord.y
    )


def test_init_left_turn(read_config: None):
    trajectory1 = BezierTrajectory(Coord(0, 0), Coord(100, 0),
                                   [Coord(50, 0)])
    trajectory2 = BezierTrajectory(Coord(150, 50), Coord(150, 150),
                                   [Coord(150, 100)])

    # Create IO roads
    road_in = Road(trajectory1, .2*trajectory1.length,
                   SHARED.SETTINGS.speed_limit,
                   upstream_is_spawner=True, downstream_is_remover=True,
                   num_lanes=2, lane_offset_angle=pi/2,
                   len_approach_region=.7*trajectory1.length)
    road_out = Road(trajectory2, .2*trajectory2.length,
                    SHARED.SETTINGS.speed_limit,
                    upstream_is_spawner=True, downstream_is_remover=True,
                    num_lanes=2, lane_offset_angle=pi/2,
                    len_approach_region=.7*trajectory1.length)

    intersection = Intersection([road_in], [road_out],
                                [(road_in, road_out, False)],
                                manager_type=StopSignManager,
                                manager_spec={
                                    'tiling_type': SquareTiling,
                                    'tiling_spec': {
                                        'tile_type': DeterministicTile,
                                        'misc_spec': {
                                            'tile_width': 5
                                        }
                                    }
    },
        speed_limit=SHARED.SETTINGS.speed_limit)

    # Test that the IO lanes got connected correctly
    assert len(intersection.lanes) == 1
    assert intersection.lanes[0].trajectory.start_coord == \
        road_in.lanes[1].trajectory.end_coord
    assert intersection.lanes[0].trajectory.end_coord == \
        road_out.lanes[0].trajectory.start_coord
    assert intersection.lanes[0].trajectory.control_coord == (
        road_out.lanes[0].trajectory.start_coord.x,
        road_in.lanes[1].trajectory.end_coord.y
    )


def test_speed_and_step(read_config: None, vehicle: AutomatedVehicle,
                        vehicle2: AutomatedVehicle):

    trajectory1 = BezierTrajectory(Coord(-100, 0), Coord(0, 0),
                                   [Coord(-50, 0)])
    trajectory2 = BezierTrajectory(Coord(100, 0), Coord(200, 0),
                                   [Coord(150, 0)])

    # Create IO roads
    road_in = Road(trajectory1, .2*trajectory1.length,
                   SHARED.SETTINGS.speed_limit,
                   upstream_is_spawner=True, downstream_is_remover=True,
                   num_lanes=2, lane_offset_angle=pi/2,
                   len_approach_region=.7*trajectory1.length)
    road_out = Road(trajectory2, .2*trajectory2.length,
                    SHARED.SETTINGS.speed_limit,
                    upstream_is_spawner=True, downstream_is_remover=True,
                    num_lanes=2, lane_offset_angle=pi/2,
                    len_approach_region=.7*trajectory1.length)

    intersection = Intersection([road_in], [road_out],
                                [(road_in, road_out, True)],
                                manager_type=StopSignManager,
                                manager_spec={
                                    'tiling_type': SquareTiling,
                                    'tiling_spec': {
                                        'tile_type': DeterministicTile,
                                        'misc_spec': {
                                            'tile_width': 5
                                        }
                                    }
    },
        speed_limit=SHARED.SETTINGS.speed_limit)

    vehicle.permission_to_enter_intersection = True
    vehicle2.permission_to_enter_intersection = True

    # Add a vehicle to each lane
    vehicles = [vehicle, vehicle2]
    for i in range(2):
        veh = vehicles[i]
        half_length = veh.length*(.5+SHARED.SETTINGS.length_buffer_factor)
        for j in range(3):
            intersection.lanes[i].enter_vehicle_section(VehicleTransfer(
                vehicle=veh,
                section=VehicleSection(j),
                distance_left=(2-j)*half_length,
                pos=intersection.lanes[i].trajectory.start_coord))
    for i in range(2):
        veh = vehicles[i]
        assert len(intersection.lanes[i].vehicles) == 1
        assert intersection.lanes[i].vehicles[0] is veh
    intersection.step_vehicles()
    for i in range(2):
        veh = vehicles[i]
        half_length = veh.length*(.5+SHARED.SETTINGS.length_buffer_factor)
        assert veh.pos == approx(
            Coord(half_length, intersection.lanes[i].trajectory.start_coord.y))

    # Get its speed in the next timestep
    new_speeds = intersection.get_new_speeds()
    assert len(new_speeds) == 2
    assert new_speeds[vehicle] == new_speeds[vehicle2] == SpeedUpdate(
        SHARED.SETTINGS.TIMESTEP_LENGTH*SHARED.SETTINGS.min_acceleration,
        SHARED.SETTINGS.min_acceleration
    )


# def test_transfers in test_integration because it depends on the Pathfinder
# component to work.
