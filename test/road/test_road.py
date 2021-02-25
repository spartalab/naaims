from math import pi, sin, cos
from typing import Any, Dict
from _pytest.python_api import raises

from pytest import approx
from pytest_mock import MockerFixture

import aimsim.shared as SHARED
from aimsim.road import Road
from aimsim.util import Coord, SpeedUpdate, VehicleTransfer, VehicleSection
from aimsim.vehicles import AutomatedVehicle

from test.lane.test_lane import straight_trajectory


def test_road_lane_offsets(mocker: MockerFixture, read_config: None):

    lane_width = 4

    road_1_lane = Road(straight_trajectory, .2*straight_trajectory.length,
                       SHARED.SETTINGS.speed_limit,
                       upstream_is_spawner=True, downstream_is_remover=True,
                       num_lanes=1, lane_width=lane_width)
    assert road_1_lane.lanes[0].trajectory.start_coord == \
        straight_trajectory.start_coord
    assert road_1_lane.lanes[0].trajectory.end_coord == \
        straight_trajectory.end_coord

    # 2 lane road. Should have one left lane -half width off in the x-axis and
    # another on the right a half width off.
    road_2_lane = Road(straight_trajectory, .2*straight_trajectory.length,
                       SHARED.SETTINGS.speed_limit,
                       upstream_is_spawner=True, downstream_is_remover=True,
                       num_lanes=2, lane_width=lane_width)
    assert road_2_lane.lanes[0].trajectory.start_coord == Coord(
        straight_trajectory.start_coord.x - lane_width/2,
        straight_trajectory.start_coord.y
    )
    assert road_2_lane.lanes[0].trajectory.end_coord == Coord(
        straight_trajectory.end_coord.x - lane_width/2,
        straight_trajectory.end_coord.y
    )
    assert road_2_lane.lanes[1].trajectory.start_coord == Coord(
        straight_trajectory.start_coord.x + lane_width/2,
        straight_trajectory.start_coord.y
    )
    assert road_2_lane.lanes[1].trajectory.end_coord == Coord(
        straight_trajectory.end_coord.x + lane_width/2,
        straight_trajectory.end_coord.y
    )

    # 3-lane road. Center lane should have same trajectory, with one lane to
    # the left and right 1 width offset in the x-axis
    road_3_lane = Road(straight_trajectory, .2*straight_trajectory.length,
                       SHARED.SETTINGS.speed_limit,
                       upstream_is_spawner=True, downstream_is_remover=True,
                       num_lanes=3, lane_width=lane_width)
    assert road_3_lane.lanes[0].trajectory.start_coord == Coord(
        straight_trajectory.start_coord.x - lane_width,
        straight_trajectory.start_coord.y
    )
    assert road_3_lane.lanes[0].trajectory.end_coord == Coord(
        straight_trajectory.end_coord.x - lane_width,
        straight_trajectory.end_coord.y
    )
    assert road_3_lane.lanes[1].trajectory.start_coord == \
        straight_trajectory.start_coord
    assert road_3_lane.lanes[1].trajectory.end_coord == \
        straight_trajectory.end_coord
    assert road_3_lane.lanes[2].trajectory.start_coord == Coord(
        straight_trajectory.start_coord.x + lane_width,
        straight_trajectory.start_coord.y
    )
    assert road_3_lane.lanes[2].trajectory.end_coord == Coord(
        straight_trajectory.end_coord.x + lane_width,
        straight_trajectory.end_coord.y
    )

    # 2-lane road with ends at a 45 degree angle
    angle = pi/4
    road_2_lane_angled = Road(straight_trajectory,
                              .2*straight_trajectory.length,
                              SHARED.SETTINGS.speed_limit,
                              upstream_is_spawner=True,
                              downstream_is_remover=True,
                              num_lanes=2, lane_offset_angle=angle)
    assert road_2_lane_angled.lanes[0].trajectory.start_coord == approx(Coord(
        straight_trajectory.start_coord.x - lane_width*sin(angle)/2,
        straight_trajectory.start_coord.y - lane_width*cos(angle)/2
    ))
    assert road_2_lane_angled.lanes[0].trajectory.end_coord == approx(Coord(
        straight_trajectory.end_coord.x - lane_width*sin(angle)/2,
        straight_trajectory.end_coord.y - lane_width*cos(angle)/2
    ))
    assert road_2_lane_angled.lanes[1].trajectory.start_coord == approx(Coord(
        straight_trajectory.start_coord.x + lane_width*sin(angle)/2,
        straight_trajectory.start_coord.y + lane_width*cos(angle)/2
    ))
    assert road_2_lane_angled.lanes[1].trajectory.end_coord == approx(Coord(
        straight_trajectory.end_coord.x + lane_width*sin(angle)/2,
        straight_trajectory.end_coord.y + lane_width*cos(angle)/2
    ))


def test_road_step(read_config: None):
    road = Road(straight_trajectory, .2*straight_trajectory.length,
                SHARED.SETTINGS.speed_limit,
                upstream_is_spawner=True, downstream_is_remover=True,
                num_lanes=2)
    # Hacky way to make upstream and downstream checks work without having to
    # create actual Spawner and Remover objects
    road._Road__upstream = None
    road._Road__downstream = None

    # Construct copyable transfer
    veh1 = AutomatedVehicle(0, 0)
    veh2 = AutomatedVehicle(1, 0)
    transfer1: Dict[str, Any] = dict(
        vehicle=veh1,
        distance_left=None,
        pos=road.lanes[0].trajectory.start_coord
    )
    transfer2: Dict[str, Any] = dict(
        vehicle=veh2,
        distance_left=None,
        pos=road.lanes[1].trajectory.start_coord
    )

    # Add a vehicle to each road lane
    for section in VehicleSection:
        road.transfer_vehicle(VehicleTransfer(section=section, **transfer1))
        road.transfer_vehicle(VehicleTransfer(section=section, **transfer2))
    road.process_transfers()
    assert len(road.lanes[0].vehicles) == 1
    assert len(road.lanes[1].vehicles) == 1
    assert road.lanes[0].vehicles[0] == veh1
    assert road.lanes[1].vehicles[0] == veh2
    road.step_vehicles()
    assert veh1.pos == approx(Coord(veh1.length*(
        .5+SHARED.SETTINGS.length_buffer_factor
    ) - .5*road.lane_width, 0))
    assert veh2.pos == approx(Coord(veh2.length*(
        .5+SHARED.SETTINGS.length_buffer_factor
    ) + .5*road.lane_width, 0))

    # Get their speeds in the next timestep
    new_speeds = road.get_new_speeds()
    assert len(new_speeds) == 2
    assert new_speeds[veh1] == new_speeds[veh2] == SpeedUpdate(
        SHARED.SETTINGS.TIMESTEP_LENGTH*veh1.max_acceleration,
        veh1.max_acceleration
    )


def test_transfer_error(read_config: None):
    road = Road(straight_trajectory, .2*straight_trajectory.length,
                SHARED.SETTINGS.speed_limit,
                upstream_is_spawner=True, downstream_is_remover=True,
                num_lanes=2)
    road.transfer_vehicle(VehicleTransfer(
        AutomatedVehicle(0, 0),
        VehicleSection.FRONT,
        None,
        Coord(99, 99)
    ))
    with raises(RuntimeError):
        road.process_transfers()