from typing import Dict, List, Optional, OrderedDict, Tuple
from math import ceil, floor

from pytest import raises, fixture, approx

from naaims.util import VehicleSection, Coord
from naaims.trajectories import BezierTrajectory
from naaims.lane import ScheduledExit, VehicleProgress
from naaims.intersection import IntersectionLane
from naaims.intersection.tilings import Tiling, SquareTiling
from naaims.intersection.reservation import Reservation
from naaims.road import RoadLane
from naaims.vehicles.vehicle import Vehicle
from naaims.vehicles.automated import AutomatedVehicle
from naaims.pathfinder import Pathfinder
from naaims.intersection.tilings.tiles import StochasticTile, DeterministicTile
import naaims.shared as SHARED


def square_tiling(x_min: float, x_max: float, y_min: float,
                  y_max: float, tile_width: float, speed_limit: int = 1,
                  crash_probability_tolerance: float = 0,
                  timeout: bool = False) -> SquareTiling:
    x_mid = (x_max-x_min)/2
    y_mid = (y_max-y_min)/2

    # Construct incoming road lane at top middle
    in_coord = Coord(x_mid, y_max)
    rl_in = RoadLane(BezierTrajectory(
        Coord(x_mid, y_max-50), in_coord, [Coord(x_mid, y_max+25)]
    ), 5, speed_limit, 5, 40)

    # Construct outgoing road lane at middle right
    out_coord = Coord(x_max, y_mid)
    rl_out = RoadLane(BezierTrajectory(
        out_coord, Coord(x_max+50, y_mid), [Coord(x_max+25, y_mid)]
    ), 5, speed_limit, 40, 5)

    # Construct intersection lane
    il = IntersectionLane(rl_in, rl_out, speed_limit)

    return SquareTiling({in_coord: rl_in}, {out_coord: rl_out}, (il,),
                        {(in_coord, out_coord): il},
                        crash_probability_tolerance, timeout=timeout,
                        misc_spec={'tile_width': tile_width})


@fixture
def sq(speed_limit: int = 30):
    return square_tiling(0, 100, 0, 200, 1, speed_limit)


@fixture
def sq_timeouts(speed_limit: int = 30):
    return square_tiling(0, 100, 0, 200, 1, speed_limit, timeout=True)


def test_init(load_shared: None):
    sq = square_tiling(0, 100, 0, 200, 1, 30, 1e-6)
    assert sq.threshold == 1e-6 / (sq.x_tile_count * sq.y_tile_count)
    assert sq.tile_type is DeterministicTile
    assert square_tiling(0, 100, 0, 200, 1,
                         30).tile_type is DeterministicTile


def test_timesteps_forward(load_shared: None):
    assert Tiling._exit_res_timesteps_forward(0) == 18
    assert Tiling._exit_res_timesteps_forward(15) == 18
    assert Tiling._exit_res_timesteps_forward(15.1) == 20


def test_res_acceptance(load_shared: None, sq: SquareTiling, vehicle: Vehicle):
    SHARED.t = 0
    for _ in range(9):
        sq._add_new_layer()
    i_lane = sq.lanes[0]
    i_coord = i_lane.trajectory.start_coord
    r_lane = sq.incoming_road_lane_by_coord[i_coord]
    scheduled_exit = ScheduledExit(vehicle, VehicleSection.FRONT, 0, 1)
    reservation = Reservation(vehicle, i_coord, {
        1: {sq.tiles[0][sq.buffer_tile_loc[i_coord]]: 1},
        9: {sq.tiles[8][sq.buffer_tile_loc[i_coord]]: 1}
    }, sq.lanes[0], scheduled_exit)

    # Check that the tiling and road lane are clean
    assert not vehicle.has_reservation
    assert not vehicle.permission_to_enter_intersection
    assert len(sq.queued_reservations) == 0
    assert r_lane.latest_scheduled_exit is None

    # Check that reservation start fails before confirming
    with raises(ValueError):
        sq.start_reservation(vehicle)

    # During the check request process, if a vehicle's reservation request is
    # confirmed, its front exit is replaced with its rear exit.
    rear_exit = ScheduledExit(vehicle, VehicleSection.REAR, 1, 2)
    reservation.entrance_rear = rear_exit

    # Confirm the reservation and check its outcomes
    sq.confirm_reservation(reservation, r_lane)
    assert vehicle.has_reservation
    assert vehicle.permission_to_enter_intersection
    assert len(sq.queued_reservations) == 1
    assert sq.queued_reservations[vehicle] is reservation
    assert r_lane.latest_scheduled_exit is rear_exit

    # Bring us to the reservation's starting time and start it
    SHARED.t = 1
    sq.handle_new_timestep()
    i_lane_res = sq.start_reservation(vehicle)
    assert i_lane_res is i_lane
    assert len(sq.queued_reservations) == 0
    assert len(sq.active_reservations) == 1
    assert sq.active_reservations[vehicle] is reservation

    # Bring us to the reservation's ending time and end it
    for _ in range(8):
        SHARED.t += 1
        sq.handle_new_timestep()
    sq.clear_reservation(vehicle)
    assert len(sq.active_reservations) == 0


def test_new_timestep(load_shared: None, sq: SquareTiling, vehicle: Vehicle,
                      vehicle2: Vehicle):
    SHARED.t = 0
    for _ in range(3):
        sq._add_new_layer()
    res1 = Reservation(vehicle, Coord(0, 0), {}, sq.lanes[0],
                       ScheduledExit(vehicle, VehicleSection.FRONT, 0, 0))
    sq.tiles[1][2].confirm_reservation(res1)
    res2 = Reservation(vehicle2, Coord(0, 0), {}, sq.lanes[0],
                       ScheduledExit(vehicle2, VehicleSection.FRONT, 0, 0))
    assert sq.tiles[0][2].will_reservation_work(res2) is True
    assert sq.tiles[1][2].will_reservation_work(res2) is False
    assert sq.tiles[2][2].will_reservation_work(res2) is True
    assert len(sq.tiles) == 3
    sq.handle_new_timestep()
    assert len(sq.tiles) == 2
    assert sq.tiles[0][2].will_reservation_work(res2) is False
    assert sq.tiles[1][2].will_reservation_work(res2) is True


def test_mock_speed_update(load_shared: None, sq: SquareTiling,
                           vehicle: Vehicle, vehicle2: Vehicle,
                           vehicle3: Vehicle):
    il = sq.lanes[0]

    # Place vehicles at the incoming and outgoing transition points
    il.vehicles = [vehicle, vehicle2, vehicle3]
    il.vehicle_progress = {
        vehicle: VehicleProgress(None, None, .99),
        vehicle2: VehicleProgress(.4, .5, .6),
        vehicle3: VehicleProgress(0.01, None, None)
    }

    # Get their speeds in the next timestep
    sq._mock_update_speeds(il)
    assert len(il.vehicles) == 3
    for veh in il.vehicles:
        assert veh.velocity == SHARED.SETTINGS.TIMESTEP_LENGTH *\
            SHARED.SETTINGS.min_acceleration
        assert veh.acceleration == SHARED.SETTINGS.min_acceleration

    # First two vehicles near speed limit
    vehicle.velocity = vehicle2.velocity = 30 - .05
    sq._mock_update_speeds(il)
    assert len(il.vehicles) == 3
    for veh in [vehicle, vehicle2]:
        assert vehicle2.velocity == 30
        assert vehicle2.acceleration == SHARED.SETTINGS.min_acceleration
    assert vehicle3.velocity == SHARED.SETTINGS.TIMESTEP_LENGTH *\
        SHARED.SETTINGS.min_acceleration*2
    assert vehicle3.acceleration == SHARED.SETTINGS.min_acceleration


def test_mock_outgoing_step(load_shared: None, sq: SquareTiling,
                            vehicle: Vehicle, vehicle2: Vehicle):

    # Test normal case
    orl = sq.outgoing_road_lane_by_coord[sq.lanes[0].trajectory.end_coord]
    orl.vehicles = [vehicle, vehicle2]
    orl.vehicle_progress[vehicle] = VehicleProgress(.5, .45, None)
    orl.vehicle_progress[vehicle2] = VehicleProgress(.1, .05, None)
    vehicle.velocity = vehicle2.velocity = 1
    vehicle.acceleration = vehicle2.acceleration = \
        SHARED.SETTINGS.min_acceleration
    sq._mock_outgoing_step_vehicles(orl)
    distance_covered = vehicle.velocity * SHARED.SETTINGS.TIMESTEP_LENGTH + \
        vehicle.acceleration * SHARED.SETTINGS.TIMESTEP_LENGTH**2
    p_covered = distance_covered / orl.trajectory.length
    assert orl.vehicle_progress[vehicle] == approx(
        VehicleProgress(.5 + p_covered, .45 + p_covered), 1e-3)
    assert orl.vehicle_progress[vehicle2] == approx(
        VehicleProgress(.1 + p_covered, .05 + p_covered), 1e-3)
    assert vehicle.pos == approx(
        orl.trajectory.get_position(.45 + p_covered), 1e-3)
    assert vehicle.heading == approx(
        orl.trajectory.get_heading(.45 + p_covered), 1e-3)
    assert vehicle2.pos == approx(
        orl.trajectory.get_position(.05 + p_covered), 1e-3)
    assert vehicle2.heading == approx(
        orl.trajectory.get_heading(.05 + p_covered), 1e-3)

    # Test vehicle exit intentional error
    orl.vehicle_progress[vehicle] = VehicleProgress(1, 1, .5)
    with raises(RuntimeError):
        sq._mock_outgoing_step_vehicles(orl)


def test_mock_intersection_step_transfer(load_shared: None, sq: SquareTiling,
                                         vehicle: Vehicle, vehicle2: Vehicle,
                                         vehicle3: Vehicle):
    il = sq.lanes[0]
    orl = sq.outgoing_road_lane_by_coord[sq.lanes[0].trajectory.end_coord]

    # Place vehicles at the incoming and outgoing transition points
    il.vehicles = [vehicle, vehicle2, vehicle3]
    il.vehicle_progress = {
        vehicle: VehicleProgress(1-1e-6, .95, .9),
        vehicle2: VehicleProgress(.6, .5, .4),
        vehicle3: VehicleProgress(0.01, None, None)
    }
    il.lateral_deviation[vehicle] = 0

    # Form reservations for each vehicle
    test_reservations = {
        vehicle: Reservation(vehicle, il.trajectory.start_coord, {}, il,
                             ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                             ),
        vehicle2: Reservation(vehicle2, il.trajectory.start_coord, {}, il,
                              ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                              ),
        vehicle3: Reservation(vehicle3, il.trajectory.start_coord, {}, il,
                              ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                              )
    }
    valid_reservations: List[Reservation] = []
    test_t = 4

    # Set vehicles to be moving
    for veh in [vehicle, vehicle2, vehicle3]:
        veh.velocity = 1
        veh.acceleration = 1

    # Step intersection
    assert not sq._mock_intersection_step_vehicles(il, orl, test_reservations,
                                                   valid_reservations, test_t,
                                                   False)

    # Check if the vehicle with center in intersection is in is new position
    new_p = .5 + vehicle2.velocity*SHARED.SETTINGS.TIMESTEP_LENGTH + \
        vehicle2.acceleration*SHARED.SETTINGS.TIMESTEP_LENGTH**2
    vehicle2.pos == il.trajectory.get_position(new_p)
    vehicle2.heading == il.trajectory.get_heading(new_p)

    # Check if the exiting vehicle's reservation updated properly
    assert vehicle in test_reservations
    assert valid_reservations == []
    assert vehicle in il.vehicles
    assert vehicle in il.vehicle_progress
    dist_covered = vehicle.velocity*SHARED.SETTINGS.TIMESTEP_LENGTH + \
        vehicle.acceleration*SHARED.SETTINGS.TIMESTEP_LENGTH**2
    p_il_covered = dist_covered / il.trajectory.length
    assert il.vehicle_progress[vehicle] == approx(VehicleProgress(
        None, .95 + p_il_covered, .9 + p_il_covered), 1e-3)
    assert orl.vehicles == [vehicle]
    assert vehicle in orl.vehicle_progress
    assert len(orl.vehicle_progress) == 1
    assert orl.vehicle_progress[vehicle] == approx(VehicleProgress(
        (dist_covered - 1e-6*il.trajectory.length)/orl.trajectory.length,
        None, None), 1e-2)


def test_mock_intersection_step_io_ok(load_shared: None, sq: SquareTiling,
                                      vehicle: Vehicle, vehicle2: Vehicle,
                                      vehicle3: Vehicle):
    il = sq.lanes[0]
    orl = sq.outgoing_road_lane_by_coord[sq.lanes[0].trajectory.end_coord]

    # Place vehicles at the incoming and outgoing transition points
    il.vehicles = [vehicle, vehicle2, vehicle3]
    il.vehicle_progress = {
        vehicle: VehicleProgress(None, None, 1-1e-6),
        vehicle2: VehicleProgress(.6, .5, .4),
        vehicle3: VehicleProgress(0.01, None, None)
    }
    il.lateral_deviation[vehicle] = 0
    orl.vehicles = [vehicle]
    orl.vehicle_progress[vehicle] = VehicleProgress(.1, .05, None)

    # Form reservations for each vehicle
    res1 = Reservation(vehicle, il.trajectory.start_coord, {}, il,
                       ScheduledExit(vehicle, VehicleSection.REAR, 0, 0))
    test_reservations = {
        vehicle: res1,
        vehicle2: Reservation(vehicle2, il.trajectory.start_coord, {}, il,
                              ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                              ),
        vehicle3: Reservation(vehicle3, il.trajectory.start_coord, {}, il,
                              ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                              )
    }
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    test_t = 4

    # Set vehicles to be moving
    for veh in [vehicle, vehicle2, vehicle3]:
        veh.velocity = 1
        veh.acceleration = 1

    # Find exit tiles used
    used_tiles = sq.io_tile_buffer(il, test_t, vehicle,
                                   test_reservations[vehicle], False)

    # Step intersection
    assert not sq._mock_intersection_step_vehicles(  # type: ignore
        il, orl, test_reservations, valid_reservations, test_t, False)

    # Check if the vehicle with center in intersection is in is new position
    new_p = .5 + vehicle2.velocity*SHARED.SETTINGS.TIMESTEP_LENGTH + \
        vehicle2.acceleration*SHARED.SETTINGS.TIMESTEP_LENGTH**2
    vehicle2.pos == il.trajectory.get_position(new_p)
    vehicle2.heading == il.trajectory.get_heading(new_p)

    # Check if the exiting vehicle's reservation updated properly
    assert vehicle not in test_reservations
    assert vehicle2 in test_reservations
    assert vehicle3 in test_reservations
    assert next(iter(valid_reservations.values())).vehicle is vehicle
    assert next(iter(valid_reservations.values())).tiles == used_tiles
    assert vehicle not in il.vehicles
    assert vehicle not in il.vehicle_progress
    assert orl.vehicles == []
    assert vehicle not in orl.vehicle_progress
    # dist_covered = vehicle.velocity*SHARED.SETTINGS.TIMESTEP_LENGTH + \
    #     vehicle.acceleration*SHARED.SETTINGS.TIMESTEP_LENGTH**2
    # p_orl_covered = dist_covered / orl.trajectory.length
    # assert orl.vehicle_progress == {vehicle: VehicleProgress(
    #     .1 + p_orl_covered, .05 + p_orl_covered,
    #     (dist_covered - 1e-6*il.trajectory.length)/orl.trajectory.length)}


def test_mock_intersection_step_block(load_shared: None, sq: SquareTiling,
                                      vehicle: Vehicle, vehicle2: Vehicle,
                                      vehicle3: Vehicle):
    il = sq.lanes[0]
    orl = sq.outgoing_road_lane_by_coord[sq.lanes[0].trajectory.end_coord]

    # Place vehicles at the incoming and outgoing transition points
    il.vehicles = [vehicle, vehicle2]
    il.vehicle_progress = {
        vehicle: VehicleProgress(None, None, 1-1e-6),
        vehicle2: VehicleProgress(.6, .5, .4)
    }
    il.lateral_deviation[vehicle] = 0
    orl.vehicles = [vehicle]
    orl.vehicle_progress[vehicle] = VehicleProgress(.1, .05, None)

    # Form reservations for each vehicle
    test_reservations = {
        vehicle: Reservation(vehicle, il.trajectory.start_coord, {}, il,
                             ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                             ),
        vehicle2: Reservation(vehicle2, il.trajectory.start_coord, {}, il,
                              ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                              )
    }
    a_valid_res = Reservation(vehicle3, il.trajectory.start_coord, {}, il,
                              ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                              )
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict(
        {vehicle3: a_valid_res})
    test_t = 4

    # Set vehicles to be moving
    for veh in [vehicle, vehicle2, vehicle3]:
        veh.velocity = 1
        veh.acceleration = 1
    end_of_window = test_t + \
        Tiling._exit_res_timesteps_forward(1)  # type: ignore

    # Find exit tiles used
    used_tiles = sq.io_tile_buffer(il, test_t, vehicle,
                                   test_reservations[vehicle], False)
    assert used_tiles is not None

    # Reserve one of the exit tiles for someone else so it'll reject
    assert len(used_tiles[end_of_window]) == 1
    for tile in used_tiles[end_of_window]:
        tile.confirm_reservation(a_valid_res)
        a_valid_res.tiles[end_of_window] = {tile: 1}

    # Step intersection
    assert sq._mock_intersection_step_vehicles(  # type: ignore
        il, orl, test_reservations, valid_reservations, test_t, False)

    # Check if the exiting vehicle's reservation updated properly
    assert valid_reservations == OrderedDict({vehicle3: a_valid_res})
    # TODO (stochastic, auction): Test marking and dependencies


def test_mock_intersection_step_exit_block(load_shared: None, sq: SquareTiling,
                                           vehicle: Vehicle, vehicle2: Vehicle,
                                           vehicle3: Vehicle):
    pass


def test_mock_incoming_step_normal(load_shared: None, sq: SquareTiling,
                                   vehicle: Vehicle):
    il = sq.lanes[0]
    irl = sq.incoming_road_lane_by_coord[sq.lanes[0].trajectory.start_coord]

    # Place vehicle at the transition point
    il.vehicles = [vehicle]
    il.vehicle_progress = {vehicle: VehicleProgress(.1, None, None)}
    irl.vehicles = [vehicle]
    irl.vehicle_progress = {vehicle: VehicleProgress(None, .9, .8)}

    # Form reservation
    test_reservations = {
        vehicle: Reservation(vehicle, il.trajectory.start_coord, {}, il,
                             ScheduledExit(vehicle, VehicleSection.REAR, 0, 0))
    }
    test_t = 5

    # Set vehicle in motion
    vehicle.velocity = 1
    vehicle.acceleration = 1

    new_exit = sq._mock_incoming_step_vehicles(  # type: ignore
        irl, il, {}, test_reservations, None, test_t)
    assert new_exit is None
    assert len(test_reservations) == 1
    assert vehicle in test_reservations
    assert test_reservations[vehicle].vehicle is vehicle
    p_next = .9 + (vehicle.velocity*SHARED.SETTINGS.TIMESTEP_LENGTH +
                   vehicle.acceleration*SHARED.SETTINGS.TIMESTEP_LENGTH**2) /\
        irl.trajectory.length
    assert irl.vehicle_progress[vehicle] == approx(VehicleProgress(
        None, p_next, p_next - .1), 1e-3)
    assert vehicle.pos == approx(irl.trajectory.get_position(p_next))
    assert vehicle.heading == approx(irl.trajectory.get_heading(p_next))


def test_mock_incoming_step_transfer(load_shared: None, sq: SquareTiling,
                                     vehicle: Vehicle):
    il = sq.lanes[0]
    irl = sq.incoming_road_lane_by_coord[sq.lanes[0].trajectory.start_coord]

    # Place vehicle at the transition point
    il.vehicles = [vehicle]
    il.vehicle_progress = {vehicle: VehicleProgress(.1, None, None)}
    irl.vehicles = [vehicle]
    irl.vehicle_progress = {vehicle: VehicleProgress(None, 1-1e-6, .8)}

    # Form reservation
    test_reservations = {
        vehicle: Reservation(vehicle, il.trajectory.start_coord, {}, il,
                             ScheduledExit(vehicle, VehicleSection.REAR, 0, 0))
    }
    test_t = 5

    # Set vehicle in motion
    vehicle.velocity = 1
    vehicle.acceleration = 1

    new_exit = sq._mock_incoming_step_vehicles(  # type: ignore
        irl, il, {}, test_reservations, None, test_t)
    assert new_exit is None
    assert len(test_reservations) == 1
    assert vehicle in test_reservations
    assert test_reservations[vehicle].vehicle is vehicle
    # TODO: Calculate p_next wrt irl AND il
    dist_covered = vehicle.velocity*SHARED.SETTINGS.TIMESTEP_LENGTH + \
        vehicle.acceleration*SHARED.SETTINGS.TIMESTEP_LENGTH**2
    p_next = (dist_covered - 1e-6 * irl.trajectory.length) \
        / il.trajectory.length
    # Note that the front progress is not changed in this example but would be
    # in the real check_request as mock_intersection_step_vehicles would have
    # been run before this, updating it.
    assert il.vehicle_progress[vehicle] == approx(VehicleProgress(
        .1, p_next, None), 1e-2)
    assert irl.vehicle_progress[vehicle] == approx(VehicleProgress(
        None, None, .8 + dist_covered / irl.trajectory.length), 1e-3)
    assert vehicle.pos == approx(il.trajectory.get_position(p_next), 1e-3)
    assert vehicle.heading == approx(il.trajectory.get_heading(p_next), 1e-3)


def test_mock_incoming_step_with_last(load_shared: None, sq: SquareTiling,
                                      vehicle: Vehicle, vehicle2: Vehicle):
    il = sq.lanes[0]
    irl = sq.incoming_road_lane_by_coord[sq.lanes[0].trajectory.start_coord]

    # Place vehicle at the transition point
    il.vehicles = [vehicle]
    il.vehicle_progress = {vehicle: VehicleProgress(.1, None, None)}
    irl.vehicles = [vehicle]
    irl.vehicle_progress = {vehicle: VehicleProgress(None, .9, .8)}

    # Form last exit
    last_exit = ScheduledExit(vehicle2, VehicleSection.REAR, 2, 10)

    # Form reservation
    test_reservations = {
        vehicle: Reservation(vehicle, il.trajectory.start_coord, {}, il,
                             ScheduledExit(vehicle, VehicleSection.REAR, 0, 0))
    }
    test_t = 5

    # Set vehicle in motion
    vehicle.velocity = 1
    vehicle.acceleration = 1

    new_exit = sq._mock_incoming_step_vehicles(  # type: ignore
        irl, il, {}, test_reservations, last_exit, test_t)
    assert new_exit is last_exit
    assert len(test_reservations) == 1
    assert vehicle in test_reservations
    assert test_reservations[vehicle].vehicle is vehicle
    p_next = .9 + (vehicle.velocity*SHARED.SETTINGS.TIMESTEP_LENGTH +
                   vehicle.acceleration*SHARED.SETTINGS.TIMESTEP_LENGTH**2) /\
        irl.trajectory.length
    assert irl.vehicle_progress[vehicle] == approx(VehicleProgress(
        None, p_next, p_next - .1), 1e-3)
    assert vehicle.pos == approx(irl.trajectory.get_position(p_next))
    assert vehicle.heading == approx(irl.trajectory.get_heading(p_next))


def test_mock_incoming_step_complete(load_shared: None, sq: SquareTiling,
                                     vehicle: Vehicle, vehicle2: Vehicle):
    il = sq.lanes[0]
    irl = sq.incoming_road_lane_by_coord[sq.lanes[0].trajectory.start_coord]
    vehicle_original = vehicle.clone_for_request()

    # Place vehicle at the transition point
    il.vehicles = [vehicle]
    il.vehicle_progress = {vehicle: VehicleProgress(.1, .05, None)}
    irl.vehicles = [vehicle]
    irl.vehicle_progress = {vehicle: VehicleProgress(None, None, 1-1e-6)}

    # Form exit
    last_exit = ScheduledExit(vehicle2, VehicleSection.REAR, 2, 10)

    # Form reservation
    test_reservations = {
        vehicle: Reservation(vehicle, il.trajectory.start_coord, {}, il,
                             ScheduledExit(vehicle, VehicleSection.REAR, 0, 0))
    }
    test_t = 5

    # Set vehicle in motion
    vehicle.velocity = 1
    vehicle.acceleration = 1
    supposed_new_exit = ScheduledExit(vehicle_original, VehicleSection.REAR,
                                      test_t, vehicle.velocity)

    new_exit = sq._mock_incoming_step_vehicles(  # type: ignore
        irl, il, {vehicle: vehicle_original}, test_reservations, last_exit,
        test_t)

    # Check that the exit and reservations updated
    assert new_exit == supposed_new_exit
    assert len(test_reservations) == 1
    assert vehicle in test_reservations
    assert test_reservations[vehicle].vehicle is vehicle
    assert test_reservations[vehicle].entrance_rear == supposed_new_exit

    # Check that the incoming road lane is now empty
    assert len(irl.vehicles) == 0
    assert vehicle not in irl.vehicle_progress

    # Check that the intersection lane updated properly
    assert il.vehicle_progress[vehicle].rear == approx(
        (vehicle.velocity*SHARED.SETTINGS.TIMESTEP_LENGTH +
         vehicle.acceleration*SHARED.SETTINGS.TIMESTEP_LENGTH**2 -
         1e-6*irl.trajectory.length) / il.trajectory.length, 1e-2)

    # Note that since the vehicle center was already in the intersection lane
    # before this timestep, its position is not updated by this call.


def test_all_pos(load_shared: None, sq: SquareTiling, vehicle: Vehicle,
                 vehicle2: Vehicle, vehicle3: Vehicle):
    il = sq.lanes[0]
    irl = sq.incoming_road_lane_by_coord[sq.lanes[0].trajectory.start_coord]

    # Place vehicles
    il.vehicles = [vehicle, vehicle2, vehicle3]
    il.vehicle_progress = {
        vehicle: VehicleProgress(1-1e-6, .95, .9),
        vehicle2: VehicleProgress(.6, .5, .4),
        vehicle3: VehicleProgress(0.01, None, None)
    }
    il.lateral_deviation[vehicle] = 0

    # Form reservations for each vehicle
    test_reservations = {
        vehicle: Reservation(vehicle, il.trajectory.start_coord, {}, il,
                             ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                             ),
        vehicle2: Reservation(vehicle2, il.trajectory.start_coord, {}, il,
                              ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                              ),
        vehicle3: Reservation(vehicle3, il.trajectory.start_coord, {}, il,
                              ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
                              )
    }
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    test_t = 4
    counter = 3
    end_at = 4

    counter_new = sq._all_pos_to_tile(  # type: ignore
        il, irl, {}, test_reservations, valid_reservations,  counter, end_at,
        test_t, False)
    assert counter == counter_new

    for veh, res in test_reservations.items():
        assert test_t in res.tiles
        assert res.tiles[test_t] == sq.pos_to_tiles(il, test_t, veh,
                                                    res)


def test_all_pos_block(load_shared: None, sq: SquareTiling, vehicle: Vehicle,
                       vehicle2: Vehicle):
    il = sq.lanes[0]
    irl = sq.incoming_road_lane_by_coord[sq.lanes[0].trajectory.start_coord]

    # Place vehicle
    il.vehicles = [vehicle2]
    il.vehicle_progress = {vehicle2: VehicleProgress(.6, .5, .4)}
    il.lateral_deviation[vehicle] = 0
    vehicle2.pos = il.trajectory.get_position(.5)
    vehicle2.heading = il.trajectory.get_heading(.5)

    # Form reservations for each vehicle
    res1 = Reservation(vehicle, il.trajectory.start_coord, {}, il,
                       ScheduledExit(vehicle, VehicleSection.REAR, 0, 0))
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict({
        vehicle: res1})
    res2 = Reservation(vehicle2, il.trajectory.start_coord, {}, il,
                       ScheduledExit(vehicle, VehicleSection.REAR, 0, 0),
                       dependent_on=res1)
    res1.dependency = res2
    test_reservations = {vehicle2: res2}
    test_t = 4
    counter = 1
    end_at = 1

    # Block one of the tiles used
    blockables = sq.pos_to_tiles(il, test_t, vehicle2,
                                 test_reservations[vehicle2])
    assert blockables is not None
    blocked_tile = list(blockables.keys())[-1]
    blocked_tile.confirm_reservation(valid_reservations[vehicle])
    valid_reservations[vehicle].tiles[test_t] = {}
    valid_reservations[vehicle].tiles[test_t][blocked_tile] = 1

    counter_new = sq._all_pos_to_tile(  # type: ignore
        il, irl, {}, test_reservations, valid_reservations,  counter, end_at,
        test_t, False)
    assert counter_new == -1

    assert valid_reservations[vehicle].dependency is None

# TODO (sequence): Test chained reservations


def test_clone_spawn(load_shared: None, sq: SquareTiling, vehicle: Vehicle):
    il_original = sq.lanes[0]
    il = il_original.clone()
    irl = sq.incoming_road_lane_by_coord[sq.lanes[0].trajectory.start_coord]

    test_reservations: Dict[Vehicle, Reservation] = {}
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    originals: List[Vehicle] = [vehicle]
    clone_to_original: Dict[Vehicle, Vehicle] = {}
    test_t = 5
    next_exit = ScheduledExit(vehicle, VehicleSection.FRONT, test_t, 2)

    complete, counter = sq._spawn_next_clone(  # type: ignore
        il, irl, originals, clone_to_original, test_reservations,
        valid_reservations, next_exit, 0, 1, test_t, il_original, False)
    assert not complete
    assert counter == 1
    assert len(clone_to_original) == 1
    clone = list(clone_to_original.keys())[0]
    assert clone_to_original[clone] is vehicle
    assert clone.vin == vehicle.vin
    assert clone in test_reservations

    # Find clone position and tiles used
    p = 1 - (vehicle.length * (.5 + SHARED.SETTINGS.length_buffer_factor))\
        / irl.trajectory.length
    vehicle.pos = irl.trajectory.get_position(p)
    vehicle.heading = irl.trajectory.get_heading(p)
    assert clone.pos == vehicle.pos
    assert clone.heading == vehicle.heading
    assert clone.velocity == next_exit.velocity
    tiles_used = sq.io_tile_buffer(
        il, test_t, vehicle, test_reservations[clone], True)
    assert tiles_used is not None
    tiles_on = sq.pos_to_tiles(
        il, test_t, vehicle, test_reservations[clone])
    assert tiles_on is not None
    tiles_used[test_t] = tiles_on
    assert test_reservations[clone] == Reservation(
        vehicle, il.trajectory.start_coord, tiles_used, il_original, next_exit)


def test_clone_spawn_io_fail(load_shared: None, sq: SquareTiling,
                             vehicle: Vehicle, vehicle2: Vehicle):
    il_original = sq.lanes[0]
    il = il_original.clone()
    irl = sq.incoming_road_lane_by_coord[sq.lanes[0].trajectory.start_coord]

    test_reservations: Dict[Vehicle, Reservation] = {}
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    originals: List[Vehicle] = [vehicle]
    clone_to_original: Dict[Vehicle, Vehicle] = {}
    test_t = 5
    end_at = 2
    next_exit = ScheduledExit(vehicle, VehicleSection.FRONT, test_t, 2)

    # Find vehicle position and tiles used
    p = 1 - (vehicle.length * (.5 + SHARED.SETTINGS.length_buffer_factor))\
        / irl.trajectory.length
    vehicle.pos = irl.trajectory.get_position(p)
    vehicle.heading = irl.trajectory.get_heading(p)
    res_compare = Reservation(
        vehicle, il.trajectory.start_coord, {}, il, next_exit)
    tiles_used = sq.io_tile_buffer(
        il, test_t, vehicle, res_compare, True)
    assert tiles_used is not None
    tiles_used_test_t = sq.pos_to_tiles(
        il, test_t, vehicle, res_compare)
    assert tiles_used_test_t is not None
    tiles_used[test_t] = tiles_used_test_t
    list(tiles_used[test_t-1].keys())[-1].confirm_reservation(Reservation(
        vehicle2, il.trajectory.start_coord, {}, il,
        ScheduledExit(vehicle2, VehicleSection.REAR, 0, 1)))

    complete, counter = sq._spawn_next_clone(  # type: ignore
        il, irl, originals, clone_to_original, test_reservations,
        valid_reservations, next_exit, 0, end_at, test_t, il_original, False)
    assert complete
    assert counter == end_at
    assert valid_reservations == OrderedDict()


def test_clone_spawn_tile_fail(load_shared: None, sq: SquareTiling,
                               vehicle: Vehicle, vehicle2: Vehicle):
    il_original = sq.lanes[0]
    il = il_original.clone()
    irl = sq.incoming_road_lane_by_coord[sq.lanes[0].trajectory.start_coord]

    test_reservations: Dict[Vehicle, Reservation] = {}
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    originals: List[Vehicle] = [vehicle]
    clone_to_original: Dict[Vehicle, Vehicle] = {}
    test_t = 5
    end_at = 2
    next_exit = ScheduledExit(vehicle, VehicleSection.FRONT, test_t, 2)

    # Find vehicle position and tiles used
    p = 1 - (vehicle.length * (.5 + SHARED.SETTINGS.length_buffer_factor))\
        / irl.trajectory.length
    vehicle.pos = irl.trajectory.get_position(p)
    vehicle.heading = irl.trajectory.get_heading(p)
    res_compare = Reservation(vehicle, il.trajectory.start_coord, {}, il,
                              next_exit)
    tiles_used = sq.io_tile_buffer(
        il, test_t, vehicle, res_compare, True)
    assert tiles_used is not None
    tiles_used_test_t = sq.pos_to_tiles(
        il, test_t, vehicle, res_compare)
    assert tiles_used_test_t is not None
    tiles_used[test_t] = tiles_used_test_t
    list(tiles_used[test_t].keys())[-1].confirm_reservation(Reservation(
        vehicle2, il.trajectory.start_coord, {}, il,
        ScheduledExit(vehicle2, VehicleSection.REAR, 0, 1)))

    complete, counter = sq._spawn_next_clone(  # type: ignore
        il, irl, originals, clone_to_original, test_reservations,
        valid_reservations, next_exit, 0, end_at, test_t, il_original, False)
    assert complete
    assert counter == end_at
    assert valid_reservations == OrderedDict()


def set_up_spawn(sq: SquareTiling, vehicle: Vehicle, vehicle2: Vehicle):
    il_original = sq.lanes[0]
    il = il_original.clone()
    irl_original = sq.incoming_road_lane_by_coord[il.trajectory.start_coord]
    irl_original.vehicles = [vehicle2, vehicle]
    irl_original.vehicle_progress = {vehicle2: VehicleProgress(None, 1, .99),
                                     vehicle: VehicleProgress(.9, .8, .7)}
    irl = irl_original.clone()
    orl = sq.outgoing_road_lane_by_coord[il.trajectory.end_coord]

    # Prep data
    test_reservations: Dict[Vehicle, Reservation] = {}
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    originals: List[Vehicle] = [vehicle]
    clone_to_original: Dict[Vehicle, Vehicle] = {}
    counter = 1
    end_at = 2
    new_exit = irl_original.soonest_exit(counter)
    assert new_exit is not None
    last_exit = None
    test_t = new_exit.t

    return counter, end_at, test_t, new_exit, irl, il, orl, \
        clone_to_original, test_reservations, valid_reservations, last_exit, \
        originals, irl_original, il_original


def test_mock_step_spawn(load_shared: None, sq: SquareTiling,
                         vehicle: Vehicle, vehicle2: Vehicle):
    counter, end_at, test_t, new_exit, irl, il, orl, clone_to_original, \
        test_reservations, valid_reservations, last_exit, originals, \
        irl_original, il_original = set_up_spawn(sq, vehicle, vehicle2)

    complete, counter_new, test_t_new, last_exit_new, new_exit_new = \
        sq._mock_step(  # type: ignore
            counter, end_at, test_t, new_exit, irl, il, orl, clone_to_original,
            test_reservations, valid_reservations, last_exit, originals,
            irl_original, il_original)
    assert not complete
    assert counter_new == end_at
    assert test_t_new == test_t + 1
    assert last_exit_new is None
    assert new_exit_new is None

    # Check that vehicle spawned properly
    assert len(irl.vehicles) == len(irl.vehicle_progress) == 1
    clone = irl.vehicles[0]
    assert clone.vin == vehicle.vin
    assert irl.vehicles == il.vehicles == [clone]
    assert len(irl.vehicle_progress) == len(il.vehicle_progress) == 1
    assert clone in irl.vehicle_progress
    p_clone = 1 - (.5+SHARED.SETTINGS.length_buffer_factor)*vehicle.length\
        / irl.trajectory.length
    assert irl.vehicle_progress[clone] == approx(VehicleProgress(
        None, p_clone, 1 - (1+2*SHARED.SETTINGS.length_buffer_factor)
        * vehicle.length/irl.trajectory.length), 1e-3)
    assert il.vehicle_progress[clone] == VehicleProgress(0, None, None)
    assert clone.pos == irl.trajectory.get_position(p_clone)
    assert clone.heading == irl.trajectory.get_heading(p_clone)
    assert clone.velocity == new_exit.velocity
    tiles_used = sq.io_tile_buffer(
        il, test_t, vehicle, test_reservations[clone], True)
    assert tiles_used is not None
    vehicle.pos = clone.pos
    vehicle.heading = clone.heading
    tiles_on = sq.pos_to_tiles(
        il, test_t, vehicle, test_reservations[clone])
    assert tiles_on is not None
    tiles_used[test_t] = tiles_on
    assert test_reservations[clone] == Reservation(
        vehicle, il.trajectory.start_coord, tiles_used, il_original, new_exit)


def test_mock_step_cant_spawn(load_shared: None, sq: SquareTiling,
                              vehicle: Vehicle, vehicle2: Vehicle):
    counter, end_at, test_t, new_exit, irl, il, orl, clone_to_original, \
        test_reservations, valid_reservations, last_exit, originals, \
        irl_original, il_original = set_up_spawn(sq, vehicle, vehicle2)

    # Block a tile
    while len(sq.tiles) < test_t:
        sq._add_new_layer()  # type: ignore
    blocked_tile = sq.tiles[test_t-1][
        sq.buffer_tile_loc[il.trajectory.start_coord]]
    blocked_tile.confirm_reservation(
        Reservation(vehicle2, il.trajectory.start_coord,
                    {test_t-1: {blocked_tile: 1}}, il,
                    ScheduledExit(vehicle2, VehicleSection.REAR, 0, 0)))

    complete, _, _, _, _ = sq._mock_step(  # type: ignore
        counter, end_at, test_t, new_exit, irl, il, orl, clone_to_original,
        test_reservations, valid_reservations, last_exit, originals,
        irl_original, il_original)
    assert complete


def test_mock_step_spawn_exit(load_shared: None, sq: SquareTiling,
                              vehicle: Vehicle):
    il_original = sq.lanes[0]
    il = il_original.clone()
    irl_original = sq.incoming_road_lane_by_coord[il.trajectory.start_coord]
    irl = irl_original.clone()
    orl = sq.outgoing_road_lane_by_coord[il.trajectory.end_coord]

    # Set vehicle in motion
    vehicle.velocity = 1
    vehicle.acceleration = a = SHARED.SETTINGS.min_acceleration
    vehicle.pos = il.trajectory.get_position(.05)
    vehicle.heading = il.trajectory.get_heading(.05)
    timestep = SHARED.SETTINGS.TIMESTEP_LENGTH

    # Create and place clone
    clone = vehicle.clone_for_request()
    irl.vehicles = [clone]
    il.vehicles = [clone]
    il.vehicle_progress = {clone: VehicleProgress(.1, .05, None)}
    il.lateral_deviation[clone] = 0
    irl.vehicle_progress = {clone: VehicleProgress(None, None, 1)}

    # Form reservations for each vehicle
    test_reservations = {clone: Reservation(
        vehicle, il.trajectory.start_coord, {}, il_original,
        ScheduledExit(vehicle, VehicleSection.FRONT, 0, 0))}
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    originals: List[Vehicle] = []
    clone_to_original = {clone: vehicle}
    test_t = 4
    counter = 1
    end_at = 1

    complete, counter_new, test_t_new, last_exit_new, new_exit_new = \
        sq._mock_step(  # type: ignore
            counter, end_at, test_t, None, irl, il, orl, clone_to_original,
            test_reservations, valid_reservations, None, originals,
            irl_original, il_original)
    assert not complete
    assert counter_new == counter
    assert test_t_new == test_t + 1
    p_new = ((1 + a*timestep)*timestep + a*timestep**2) / il.trajectory.length
    assert il.vehicle_progress[clone] == approx(
        VehicleProgress(p_new+.1, p_new+.05, p_new), 1e-1)
    assert clone.pos == approx(il.trajectory.get_position(p_new+.05), 1e-3)
    assert clone.heading == approx(il.trajectory.get_heading(p_new+.05), 1e-3)
    assert clone.velocity == 1 + a*timestep
    assert clone.acceleration == a
    assert len(test_reservations[clone].tiles) == 1
    assert test_reservations[clone].tiles[test_t] == sq.pos_to_tiles(
        il, test_t, clone, test_reservations[clone])
    assert last_exit_new == test_reservations[clone].entrance_rear == \
        ScheduledExit(vehicle, VehicleSection.REAR, test_t, clone.velocity)
    assert new_exit_new is None


def test_mock_step_continue(load_shared: None, sq: SquareTiling,
                            vehicle: Vehicle):
    il_original = sq.lanes[0]
    il = il_original.clone()
    irl_original = sq.incoming_road_lane_by_coord[il.trajectory.start_coord]
    irl = irl_original.clone()
    orl = sq.outgoing_road_lane_by_coord[il.trajectory.end_coord]

    # Set vehicle in motion
    vehicle.velocity = 1
    vehicle.acceleration = a = SHARED.SETTINGS.min_acceleration
    vehicle.pos = il.trajectory.get_position(.5)
    vehicle.heading = il.trajectory.get_heading(.5)
    timestep = SHARED.SETTINGS.TIMESTEP_LENGTH

    # Create and place clone
    clone = vehicle.clone_for_request()
    il.vehicles = [clone]
    il.vehicle_progress = {clone: VehicleProgress(.6, .5, .4)}
    il.lateral_deviation[clone] = 0

    # Form reservation for vehicle
    test_reservations = {
        clone: Reservation(vehicle, il.trajectory.start_coord, {}, il_original,
                           ScheduledExit(vehicle, VehicleSection.REAR, 0, 0))}
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    originals: List[Vehicle] = []
    clone_to_original = {clone: vehicle}
    test_t = 4
    counter = 1
    end_at = 1

    complete, counter_new, test_t_new, last_exit_new, new_exit_new = \
        sq._mock_step(  # type: ignore
            counter, end_at, test_t, None, irl, il, orl, clone_to_original,
            test_reservations, valid_reservations, None, originals,
            irl_original, il_original)
    assert not complete
    assert counter_new == counter
    assert test_t_new == test_t + 1
    assert last_exit_new is new_exit_new is None
    p_new = .5 + ((1 + a*timestep)*timestep + a*timestep**2)\
        / il.trajectory.length
    assert il.vehicle_progress[clone] == approx(
        VehicleProgress(p_new+.1, p_new, p_new-.1), 1e-3)
    assert clone.pos == approx(il.trajectory.get_position(p_new), 1e-3)
    assert clone.heading == approx(il.trajectory.get_heading(p_new), 1e-3)
    assert clone.velocity == 1 + a*timestep
    assert clone.acceleration == a
    assert len(test_reservations[clone].tiles) == 1
    assert test_reservations[clone].tiles[test_t] == sq.pos_to_tiles(
        il, test_t, clone, test_reservations[clone])


def test_mock_step_exiting(load_shared: None, sq: SquareTiling,
                           vehicle: Vehicle):
    il_original = sq.lanes[0]
    il = il_original.clone()
    irl_original = sq.incoming_road_lane_by_coord[il.trajectory.start_coord]
    irl = irl_original.clone()
    orl = sq.outgoing_road_lane_by_coord[il.trajectory.end_coord]

    # Set vehicle in motion
    vehicle.velocity = 1
    vehicle.acceleration = a = SHARED.SETTINGS.min_acceleration
    vehicle.pos = il.trajectory.get_position(.5)
    vehicle.heading = il.trajectory.get_heading(.5)
    timestep = SHARED.SETTINGS.TIMESTEP_LENGTH

    # Create and place clone
    clone = vehicle.clone_for_request()
    il.vehicles = [clone]
    il.vehicle_progress = {clone: VehicleProgress(1, .95, .9)}
    il.lateral_deviation[clone] = 0

    # Form reservation for vehicle
    test_reservations = {
        clone: Reservation(vehicle, il.trajectory.start_coord, {}, il_original,
                           ScheduledExit(vehicle, VehicleSection.REAR, 0, 0))}
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    originals: List[Vehicle] = []
    clone_to_original = {clone: vehicle}
    test_t = 4
    counter = 1
    end_at = 1

    complete, counter_new, test_t_new, last_exit_new, new_exit_new = \
        sq._mock_step(  # type: ignore
            counter, end_at, test_t, None, irl, il, orl, clone_to_original,
            test_reservations, valid_reservations,
            test_reservations[clone].entrance_front, originals, irl_original,
            il_original)
    assert not complete
    assert counter_new == counter
    assert test_t_new == test_t + 1
    assert last_exit_new is test_reservations[clone].entrance_front
    assert new_exit_new is None
    distance = (1 + a*timestep)*timestep + a*timestep**2
    p_new = .95 + distance / il.trajectory.length
    assert il.vehicle_progress[clone] == approx(
        VehicleProgress(None, p_new, p_new-.05), 1e-3)
    assert orl.vehicles == [clone]
    assert orl.vehicle_progress[clone] == approx(VehicleProgress(
        (distance / orl.trajectory.length), None, None), 1e-1)
    assert clone.pos == approx(il.trajectory.get_position(p_new), 1e-3)
    assert clone.heading == approx(il.trajectory.get_heading(p_new), 1e-3)
    assert clone.velocity == 1 + a*timestep
    assert clone.acceleration == a
    assert len(test_reservations[clone].tiles) == 1
    assert test_reservations[clone].tiles[test_t] == sq.pos_to_tiles(
        il, test_t, clone, test_reservations[clone])


def set_up_outgoing(load_shared: None, sq: SquareTiling,
                    vehicle: Vehicle, exiting: bool = True):
    il_og = sq.lanes[0]
    il = sq.lanes[0].clone()
    irl_og = sq.incoming_road_lane_by_coord[il.trajectory.start_coord]
    irl = irl_og.clone()
    orl = sq.outgoing_road_lane_by_coord[il.trajectory.end_coord]

    # Set vehicle in motion
    vehicle.velocity = 1
    vehicle.acceleration = a = SHARED.SETTINGS.min_acceleration
    vehicle.pos = il.trajectory.get_position(.5)
    vehicle.heading = il.trajectory.get_heading(.5)
    timestep = SHARED.SETTINGS.TIMESTEP_LENGTH

    # Create and place clone
    clone = vehicle.clone_for_request()
    il.vehicles = [clone]
    il.vehicle_progress = {clone: VehicleProgress(None, None,
                                                  1 if exiting else .95)}
    il.lateral_deviation[clone] = 0
    orl.vehicles = [clone]
    orl.vehicle_progress = {clone: VehicleProgress(.1, .05, None)}

    # Form reservation for vehicle
    last_exit = ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
    reservation = Reservation(vehicle, il.trajectory.start_coord, {}, il,
                              last_exit)
    test_reservations = {clone: reservation}
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    originals: List[Vehicle] = []
    clone_to_original = {clone: vehicle}
    test_t = 4
    counter = 1
    end_at = 1

    return counter, end_at, test_t, irl, il, orl, clone_to_original, \
        test_reservations, valid_reservations, last_exit, originals, irl_og, \
        il_og, a, timestep, reservation, clone


def test_mock_step_outgoing(load_shared: None, sq: SquareTiling,
                            vehicle: Vehicle):

    (counter, end_at, test_t, irl, il, orl, clone_to_original,
        test_reservations, valid_reservations, last_exit, originals, irl_og,
        il_og, a, timestep, reservation, clone) = set_up_outgoing(
            load_shared, sq, vehicle, False)

    complete, counter_new, test_t_new, last_exit_new, new_exit_new = \
        sq._mock_step(  # type: ignore
            counter, end_at, test_t, None, irl, il, orl, clone_to_original,
            test_reservations, valid_reservations, last_exit, originals,
            irl_og, il_og)
    assert not complete
    assert counter_new == counter
    assert test_t_new == test_t + 1
    assert last_exit_new is last_exit
    assert new_exit_new is None

    assert test_reservations == {clone: reservation}
    assert valid_reservations == OrderedDict()
    p_new = .05 + ((1 + a*timestep)*timestep + a*timestep**2)\
        / orl.trajectory.length
    assert il.vehicles == orl.vehicles == [clone]
    assert il.vehicle_progress[clone] == approx(VehicleProgress(
        None, None, .9 + p_new), 1e-2)
    assert orl.vehicle_progress[clone] == approx(VehicleProgress(
        p_new + .05, p_new, None), 1e-2)

    assert clone.pos == approx(orl.trajectory.get_position(p_new), 1e-3)
    assert clone.heading == approx(orl.trajectory.get_heading(p_new), 1e-3)
    assert clone.velocity == 1 + a*timestep
    assert clone.acceleration == a
    assert reservation.tiles[test_t] == sq.pos_to_tiles(il, test_t, clone,
                                                        reservation)


def test_mock_step_exited(load_shared: None, sq: SquareTiling,
                          vehicle: Vehicle):

    (counter, end_at, test_t, irl, il, orl, clone_to_original,
        test_reservations, valid_reservations, last_exit, originals, irl_og,
        il_og, a, timestep, reservation, clone) = set_up_outgoing(
            load_shared, sq, vehicle)

    complete, counter_new, test_t_new, last_exit_new, new_exit_new = \
        sq._mock_step(  # type: ignore
            counter, end_at, test_t, None, irl, il, orl, clone_to_original,
            test_reservations, valid_reservations, last_exit, originals,
            irl_og, il_og)
    assert not complete
    assert counter_new == counter
    assert test_t_new == test_t + 1
    assert last_exit_new is last_exit
    assert new_exit_new is None

    assert test_reservations == {}
    assert valid_reservations == OrderedDict({vehicle: reservation})
    p_new = .05 + ((1 + a*timestep)*timestep + a*timestep**2)\
        / orl.trajectory.length
    assert il.vehicles == orl.vehicles == []
    assert il.vehicle_progress == orl.vehicle_progress == {}

    # assert orl.vehicles == [clone]
    # assert orl.vehicle_progress[clone] == approx(VehicleProgress(
    #     p_new + .05, p_new, None), 1e-2)
    assert clone.pos == approx(orl.trajectory.get_position(p_new), 1e-3)
    assert clone.heading == approx(orl.trajectory.get_heading(p_new), 1e-3)
    assert clone.velocity == 1 + a*timestep
    assert clone.acceleration == a
    assert reservation.tiles == sq.io_tile_buffer(il, test_t, vehicle,
                                                  reservation, False)


def test_mock_step_invalid(load_shared: None, sq: SquareTiling,
                           vehicle: Vehicle, vehicle2: Vehicle):

    (counter, end_at, test_t, irl, il, orl, clone_to_original,
        test_reservations, valid_reservations, last_exit, originals, irl_og,
        il_og, _, _, _, _) = set_up_outgoing(load_shared, sq, vehicle)

    # Block a tile
    while len(sq.tiles) < test_t+2:
        sq._add_new_layer()  # type: ignore
    blocked_tile = sq.tiles[test_t+1][
        sq.buffer_tile_loc[il.trajectory.end_coord]]
    blocked_tile.confirm_reservation(
        Reservation(vehicle2, il.trajectory.start_coord,
                    {test_t+1: {blocked_tile: 1}}, il,
                    ScheduledExit(vehicle2, VehicleSection.REAR, 0, 0)))

    complete, _, _, _, _ = \
        sq._mock_step(  # type: ignore
            counter, end_at, test_t, None, irl, il, orl, clone_to_original,
            test_reservations, valid_reservations, last_exit, originals,
            irl_og, il_og)
    assert complete


@fixture(scope='module')
def clean_request(load_shared: None) -> Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]:
    # Redefined due to session particulars with pytest fixtures
    sq = square_tiling(0, 100, 0, 200, 1, 30)
    vehicle: Vehicle = AutomatedVehicle(0, 0)
    vehicle2: Vehicle = AutomatedVehicle(1, 0)

    il_og = sq.lanes[0]
    il = il_og.clone()
    irl_og = sq.incoming_road_lane_by_coord[il.trajectory.start_coord]
    irl = irl_og.clone()
    orl = sq.outgoing_road_lane_by_coord[il.trajectory.end_coord]

    # Load irl_og with vehicles to check the requests of
    irl_og.vehicles = [vehicle2, vehicle]
    dist_v2 = vehicle2.length * (1+2*SHARED.SETTINGS.length_buffer_factor)
    p_v2 = (dist_v2/2) / irl_og.trajectory.length
    irl_og.vehicle_progress = {vehicle2: VehicleProgress(1, 1 - p_v2,
                                                         1 - 2*p_v2),
                               vehicle: VehicleProgress(.9, .9 - p_v2,
                               .9 - 2*p_v2)}
    vehicle2.velocity = 0
    vehicle2.acceleration = 0
    vehicle2.pos = irl_og.trajectory.get_position(1-p_v2)
    vehicle2.heading = irl_og.trajectory.get_heading(1-p_v2)
    t_accel = ceil((2*dist_v2/SHARED.SETTINGS.min_acceleration)**.5)
    veh2_rear_entrance = ScheduledExit(
        vehicle2, VehicleSection.REAR, t_accel,
        t_accel*SHARED.SETTINGS.min_acceleration)
    veh2res = Reservation(vehicle2, il.trajectory.start_coord, {}, il_og,
                          ScheduledExit(vehicle2, VehicleSection.FRONT, 0, 0),
                          veh2_rear_entrance)
    sq.issue_permission(vehicle2, irl_og, veh2_rear_entrance)
    vehicle.velocity = 1
    vehicle.acceleration = SHARED.SETTINGS.min_acceleration
    vehicle.pos = il.trajectory.get_position(.9 - p_v2)
    vehicle.heading = il.trajectory.get_heading(.9 - p_v2)

    # Set up the Pathfinder
    SHARED.SETTINGS.pathfinder = Pathfinder([], [], {
        (il.trajectory.start_coord, 0): [il.trajectory.end_coord]})

    # Initialize data
    counter = 1
    end_at = 2
    new_exit: Optional[ScheduledExit] = irl_og.soonest_exit(counter)
    assert new_exit is not None
    test_t = new_exit.t
    assert new_exit is not None
    last_exit: Optional[ScheduledExit] = None
    test_reservations: Dict[Vehicle, Reservation] = {}
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    originals: List[Vehicle] = [vehicle, vehicle2]
    clone_to_original: Dict[Vehicle, Vehicle] = {}

    while (len(il.vehicles) > 0) or (counter < end_at):
        test_complete, counter, test_t, last_exit, new_exit = \
            sq._mock_step(  # type: ignore
                counter, end_at, test_t, new_exit, irl, il, orl,
                clone_to_original, test_reservations, valid_reservations,
                last_exit, originals, irl_og, il_og, False)
        if test_complete:
            break

    return sq, irl_og, valid_reservations, veh2res


def test_check_req_spawn_block(clean_request: Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]):
    sq, irl_og, valid_reservations, veh2res = clean_request

    # Block a tile at spawn
    t_max = min(next(iter(valid_reservations.values())).tiles.keys())
    for tile in next(iter(valid_reservations.values())).tiles[t_max]:
        veh2res.tiles[t_max] = {tile: 1}
        tile.confirm_reservation(veh2res)
        break

    cycle_res = sq.check_request(irl_og)
    assert cycle_res is None

    # Clean up
    tile._clear_all_reservations()  # type: ignore


def test_check_req_in_block(clean_request: Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]):
    sq, irl_og, valid_reservations, veh2res = clean_request

    # Block a tile in the middle
    times = list(next(iter(valid_reservations.values())).tiles.keys())
    t_use = floor(len(times)/2)
    for tile in next(iter(valid_reservations.values())).tiles[t_use]:
        veh2res.tiles[t_use] = {tile: 1}
        tile.confirm_reservation(veh2res)
        break

    cycle_res = sq.check_request(irl_og)
    assert cycle_res is None

    # Clean up
    tile._clear_all_reservations()  # type: ignore


def test_check_req_out_block(clean_request: Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]):
    sq, irl_og, valid_reservations, veh2res = clean_request

    # Block a tile at exit
    t_max = max(next(iter(valid_reservations.values())).tiles.keys())
    for tile in next(iter(valid_reservations.values())).tiles[t_max]:
        veh2res.tiles[t_max] = {tile: 1}
        tile.confirm_reservation(veh2res)
        break

    cycle_res = sq.check_request(irl_og)
    assert cycle_res is None

    # Clean up
    tile._clear_all_reservations()  # type: ignore


def test_check_req_ok(clean_request: Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]):
    sq, irl_og, valid_reservations, _ = clean_request
    cycle_res = sq.check_request(irl_og)
    assert cycle_res is not None
    assert cycle_res[0] == next(iter(valid_reservations.values()))


def request_timeout_option(timeout: bool, p_back: float = .9) -> Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]:
    sq = square_tiling(0, 100, 0, 200, 1, 30, timeout=timeout)
    vehicle: Vehicle = AutomatedVehicle(0, 0)
    vehicle2: Vehicle = AutomatedVehicle(1, 0)

    il_og = sq.lanes[0]
    il = il_og.clone()
    irl_og = sq.incoming_road_lane_by_coord[il.trajectory.start_coord]
    irl = irl_og.clone()
    orl = sq.outgoing_road_lane_by_coord[il.trajectory.end_coord]

    # Place vehicle at the edge of the intersection
    irl_og.vehicles = [vehicle]
    dist_v2 = vehicle.length * (1+2*SHARED.SETTINGS.length_buffer_factor)
    p_v2 = (dist_v2/2) / irl_og.trajectory.length
    irl_og.vehicle_progress = {vehicle: VehicleProgress(p_back, p_back - p_v2,
                                                        p_back - 2*p_v2)}
    vehicle.velocity = 0
    vehicle.acceleration = 0
    vehicle.pos = irl_og.trajectory.get_position(1-p_v2)
    vehicle.heading = irl_og.trajectory.get_heading(1-p_v2)

    # Set up the Pathfinder
    SHARED.SETTINGS.pathfinder = Pathfinder([], [], {
        (il.trajectory.start_coord, 0): [il.trajectory.end_coord]})

    # Initialize data
    counter = 0
    end_at = 1
    new_exit: Optional[ScheduledExit] = irl_og.soonest_exit(counter)
    assert new_exit is not None
    test_t = new_exit.t
    t_exit = new_exit.t  # For use in scheduling a competing reservation
    last_exit: Optional[ScheduledExit] = None
    test_reservations: Dict[Vehicle, Reservation] = {}
    valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
    originals: List[Vehicle] = []
    clone_to_original: Dict[Vehicle, Vehicle] = {}

    while (len(il.vehicles) > 0) or (counter < end_at):
        test_complete, counter, test_t, last_exit, new_exit = \
            sq._mock_step(  # type: ignore
                counter, end_at, test_t, new_exit, irl, il, orl,
                clone_to_original, test_reservations, valid_reservations,
                last_exit, originals, irl_og, il_og, False)
        if test_complete:
            break

    # Confirm a competing reservation at the moment of vehicle entry
    one_tile_used = list(next(iter(valid_reservations.values())).tiles[t_exit]
                         )[-1]
    veh2res = Reservation(vehicle2, il.trajectory.start_coord,
                          {t_exit: {one_tile_used: 1}},
                          il, ScheduledExit(
                              vehicle2, VehicleSection.REAR, 1,
                              SHARED.SETTINGS.min_acceleration))
    one_tile_used.confirm_reservation(veh2res)

    return sq, irl_og, valid_reservations, veh2res


@fixture(scope='function')
def request_with_timeout(load_shared_clean: None):
    return request_timeout_option(True)


@fixture(scope='function')
def request_with_timeout_short(load_shared_clean: None) -> Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]:
    return request_timeout_option(True, p_back=.99)


@fixture(scope='function')
def request_without_timeout(load_shared_clean: None) -> Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]:
    return request_timeout_option(False)


@fixture(scope='function')
def request_without_timeout_short(load_shared_clean: None) -> Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]:
    return request_timeout_option(False, p_back=.99)


def timeout_test_pattern(request_packet: Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation],
        p_back: float = .9, timeout: bool = True):
    sq, irl_og, valid_reservations, _ = request_packet
    vehicle = next(iter(valid_reservations.values())).vehicle

    # Calculate traversal time
    t_traverse = ceil((2*((1-p_back) * irl_og.trajectory.length) /
                       SHARED.SETTINGS.min_acceleration) ** .5
                      * SHARED.SETTINGS.steps_per_second)
    t_timeout = round(min(.5 * SHARED.SETTINGS.steps_per_second, t_traverse/2))

    # Ensure the vehicle's reservation request fails and its timeout is logged
    # if timeout is on.
    cycle_res = sq.check_request(irl_og)
    assert cycle_res is None
    if timeout:
        assert sq.timeout_until is not None
        assert sq.timeout_until[vehicle] == approx(t_timeout)
    else:
        assert sq.timeout_until is None

    # Advance time and vehicle
    SHARED.t += 1
    sq.handle_new_timestep()

    # Check that the vehicle's reservation request still fails if timeout is
    # on, and passes if not.
    cycle_res = sq.check_request(irl_og)
    if timeout:
        assert cycle_res is None
        assert sq.timeout_until is not None
        assert sq.timeout_until[vehicle] == approx(t_timeout)
    else:
        assert cycle_res is not None
        assert sq.timeout_until is None


def test_timeout_long(request_with_timeout: Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]):
    timeout_test_pattern(request_with_timeout)


def test_timeout_counterfactural(request_without_timeout: Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]):
    timeout_test_pattern(request_without_timeout, timeout=False)


def test_timeout_short(request_with_timeout_short: Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]):
    timeout_test_pattern(request_with_timeout_short, p_back=.99)


def test_timeout_short_counterfactual(request_without_timeout_short: Tuple[
        Tiling, RoadLane, OrderedDict[Vehicle, Reservation], Reservation]):
    timeout_test_pattern(request_without_timeout_short,
                         p_back=.99, timeout=False)
