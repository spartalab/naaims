from typing import DefaultDict, FrozenSet, List, Dict, Any, Tuple, Set
from _pytest.python_api import approx

from pytest import fixture, raises

import naaims.shared as SHARED
from naaims.util import Coord, VehicleSection
from naaims.intersection import Intersection, IntersectionLane
from naaims.intersection.reservation import Reservation
from naaims.intersection.managers import AuctionManager
from naaims.intersection.managers.auction import Mechanism
from naaims.intersection.tilings import SquareTiling
from naaims.intersection.tilings.tiles import DeterministicTile
from naaims.lane import ScheduledExit, VehicleProgress
from naaims.road import RoadLane
from naaims.vehicles import Vehicle
from naaims.trajectories import BezierTrajectory
from naaims.simulator import Simulator


def intersec(manager_misc_spec: Dict[str, Any] = {}):

    length = 50
    speed_limit = 15

    # Create IO roads
    traj_i_l = BezierTrajectory(Coord(-length, 10), Coord(0, 10),
                                [Coord(-length/2, 10)])
    traj_o_l = BezierTrajectory(Coord(0, 22), Coord(-length, 22),
                                [Coord(-length/2, 22)])
    traj_i_d = BezierTrajectory(Coord(22, -length), Coord(22, 0),
                                [Coord(22, -length/2)])
    traj_o_d = BezierTrajectory(Coord(10, 0), Coord(10, -length),
                                [Coord(10, -length/2)])
    traj_i_r = BezierTrajectory(Coord(32+length, 22), Coord(32, 22),
                                [Coord(32+length/2, 22)])
    traj_o_r = BezierTrajectory(Coord(32, 10), Coord(32+length, 10),
                                [Coord(32+length/2, 10)])
    traj_i_u = BezierTrajectory(Coord(10, 32+length), Coord(10, 32),
                                [Coord(10, 32+length/2)])
    traj_o_u = BezierTrajectory(Coord(22, 32), Coord(22, 32+length),
                                [Coord(22, 32+length/2)])
    traj_i = [traj_i_l, traj_i_d, traj_i_r, traj_i_u]
    traj_o = [traj_o_l, traj_o_d, traj_o_r, traj_o_u]

    road_specs: List[Dict[str, Any]] = []
    # Form incoming road specs
    for i, traj in enumerate(traj_i):
        road_specs.append(dict(
            id=i,
            upstream_id=i,
            downstream_id=0,  # Only one intersection
            trajectory=traj,
            num_lanes=3,
            lane_width=4,
            upstream_is_spawner=True,
            downstream_is_remover=False,
            lane_offset_angle=0,
            len_approach_region=traj.length*.8,
            len_entrance_region=traj.length*.19,
            speed_limit=speed_limit
        ))
    # Form outgoing road specs
    for i, traj in enumerate(traj_o):
        road_specs.append(dict(
            id=i+4,
            upstream_id=0,  # Only one intersection
            downstream_id=i,
            trajectory=traj,
            num_lanes=3,
            lane_width=4,
            upstream_is_spawner=False,
            downstream_is_remover=True,
            lane_offset_angle=0,
            len_approach_region=traj.length*.19,
            len_entrance_region=traj.length*.8,
            speed_limit=speed_limit
        ))

    spawner_specs: List[Dict[str, Any]] = []

    # Form spawner and factory specs
    for i in range(4):
        spawner_specs.append(dict(
            id=i,
            road_id=i,
            vpm=0,
            factory_selection_probabilities=[1],
            factory_types=[],
            factory_specs=[]
        ))

    # Form remover specs
    remover_specs: List[Dict[str, Any]] = []
    for i in range(4):
        remover_specs.append(dict(
            id=i,
            road_id=4+i
        ))

    # Form intersection spec
    intersection_spec: Dict[str, Any] = dict(
        id=0,
        incoming_road_ids=[0, 1, 2, 3],
        outgoing_road_ids=[4, 5, 6, 7],
        connectivity=[(0, 6, True),  # left to right
                      (1, 7, True),  # down to up
                      (2, 4, True),  # right to left
                      (3, 5, True),  # up to down
                      (0, 5, False),  # left to down
                      (0, 7, False),  # left to up
                      (1, 4, False),  # down to left
                      (1, 6, False),  # down to right
                      (2, 7, False),  # right to up
                      (2, 5, False),  # right to down
                      (3, 4, False),  # up to left
                      (3, 6, False)  # up to right
                      ],
        manager_type=AuctionManager,
        manager_spec=dict(
            tiling_type=SquareTiling,
            tiling_spec=dict(
                tile_type=DeterministicTile,
                misc_spec=dict(tile_width=4),
                timeout=True),
            misc_spec=manager_misc_spec),
        speed_limit=15
    )

    # Form pathfinder hardcode
    od_pair: Dict[Tuple[Coord, int], List[Coord]] = {}
    # Through   left (0) to right (2)
    od_pair[(Coord(0.0, 10.0), 2)] = [Coord(32.0, 10.0)]
    od_pair[(Coord(0.0, 14.0), 2)] = [Coord(32.0, 14.0)]
    od_pair[(Coord(0.0, 6.0), 2)] = [Coord(32.0, 6.0)]

    # Through   down (1) to up (3)
    od_pair[(Coord(18.0, -2.4492935982947064e-16), 3)
            ] = [Coord(18.0, 32.0)]
    od_pair[(Coord(22.0, 0.0), 3)] = [Coord(22.0, 32.0)]
    od_pair[(Coord(26.0, 2.4492935982947064e-16), 3)
            ] = [Coord(26.0, 32.0)]

    # Through   right (2) to left (0)
    od_pair[(Coord(32.0, 18.0), 0)] = [
        Coord(4.898587196589413e-16, 18.0)]
    od_pair[(Coord(32.0, 22.0), 0)] = [Coord(0.0, 22.0)]
    od_pair[(Coord(32.0, 26.0), 0)] = [
        Coord(-4.898587196589413e-16, 26.0)]

    # Through   up (3) to down (1)
    od_pair[(Coord(6.0, 32.0), 1)] = [
        Coord(6.0, -7.347880794884119e-16)]
    od_pair[(Coord(14.0, 32.0), 1)] = [
        Coord(14.0, 7.347880794884119e-16)]
    od_pair[(Coord(10.0, 32.0), 1)] = [Coord(10.0, 0.0)]

    # Right     left (0) to down (1)
    od_pair[(Coord(0.0, 6.0), 1)] = [
        Coord(6.0, -7.347880794884119e-16)]

    # Left      left (0) to up (3)
    od_pair[(Coord(0.0, 14.0), 3)] = [Coord(18.0, 32.0)]

    # Left      down (1) to left (0)
    od_pair[(Coord(18.0, -2.4492935982947064e-16), 0)
            ] = [Coord(18.0, 32.0)]

    # Right     down (1) to right (2)
    od_pair[(Coord(26.0, 2.4492935982947064e-16), 2)
            ] = [Coord(32.0, 6.0)]

    # Right     right (2) to up (3)
    od_pair[(Coord(32.0, 26.0), 3)] = [Coord(26.0, 32.0)]

    # Left      right (2) to down (1)
    od_pair[(Coord(32.0, 18.0), 1)] = [
        Coord(14.0, 7.347880794884119e-16)]

    # Right     up (3) to left (0)
    od_pair[(Coord(6.0, 32.0), 0)] = [
        Coord(-4.898587196589413e-16, 26.0)]

    # Left      up (3) to right (2)
    od_pair[(Coord(14.0, 32.0), 2)] = [Coord(32.0, 14.0)]

    return Simulator(road_specs, [intersection_spec], spawner_specs,
                     remover_specs, od_pair).intersections[0]


def test_init_multi_sequence(clean_shared: None):
    with raises(NotImplementedError):
        intersec(manager_misc_spec={'multiple': True, 'sequence': True})


def test_init_2nd(clean_shared: None):
    vot_mean = {Coord(0, 6): 10., Coord(0, 10): 5.}
    inter = intersec(manager_misc_spec={
        'multiple': True, 'sequence': False, 'mechanism': 'second',
        'vpm_mean': {Coord(0, 6): 60., Coord(0, 10): 30.6},
        'vot_mean': vot_mean})
    assert type(inter.manager) is AuctionManager
    assert inter.manager.multiple
    assert not inter.manager.sequence
    assert inter.manager.mechanism is Mechanism.SECOND_PRICE
    assert inter.manager.floor
    assert inter.manager.vps_mean[inter.incoming_road_lane_by_coord[
        Coord(0, 6)]] == 1.
    assert inter.manager.vps_mean[inter.incoming_road_lane_by_coord[
        Coord(0, 10)]] == .51
    assert inter.manager.vot_mean[inter.incoming_road_lane_by_coord[
        Coord(0, 6)]] == 10.
    assert inter.manager.vot_mean[inter.incoming_road_lane_by_coord[
        Coord(0, 10)]] == 5.
    assert len(inter.manager.payments) == 0


def test_init_externality(clean_shared: None):
    inter = intersec(manager_misc_spec={
        'multiple': False, 'sequence': True, 'mechanism': 'externality',
        'floor': False, 'vpm_mean': {Coord(0, 6): 24., Coord(0, 10): 600.}})
    assert type(inter.manager) is AuctionManager
    assert not inter.manager.multiple
    assert inter.manager.sequence
    assert inter.manager.mechanism is Mechanism.EXTERNALITY
    assert not inter.manager.floor
    assert inter.manager.vps_mean[inter.incoming_road_lane_by_coord[
        Coord(0, 6)]] == .4
    assert inter.manager.vps_mean[inter.incoming_road_lane_by_coord[
        Coord(0, 10)]] == 10.
    assert len(inter.manager.payments) == 0


@fixture
def intersection_clean(clean_shared: None):
    return intersec()


def test_check_empty(intersection_clean: Intersection, vehicle: Vehicle):
    intersection = intersection_clean
    assert type(intersection.manager) is AuctionManager
    assert intersection.manager.check_empty()
    intersection.lanes[0].add_vehicle(vehicle)
    assert not intersection.manager.check_empty()


@fixture
def intersection(intersection_clean: Intersection, vehicle: Vehicle,
                 vehicle2: Vehicle, vehicle3: Vehicle):
    intersection = intersection_clean
    vehicle._Vehicle__vot = 0.1
    vehicle._Vehicle__destination = 2
    vehicle2._Vehicle__vot = 100.
    vehicle._Vehicle__destination = 2
    vehicle3._Vehicle__vot = 5.
    vehicle3._Vehicle__destination = 3
    for i, lane in enumerate(
            intersection.incoming_road_lane_by_coord.values()):
        if i == 0:
            lane.vehicles = [vehicle, vehicle2]
            lane.vehicle_progress[vehicle] = VehicleProgress(.99, .95, .9)
            lane.vehicle_progress[vehicle2] = VehicleProgress(.85, .8, .75)
        elif i == 5:
            lane.vehicles = [vehicle3]
            lane.vehicle_progress[vehicle3] = VehicleProgress(.95, .9, .85)
    return intersection


def test_get_leading(intersection: Intersection, vehicle: Vehicle,
                     vehicle3: Vehicle):
    assert type(intersection.manager) is AuctionManager
    request_to_rl, rl_to_leading_request, sum_vot, start_idx = \
        intersection.manager.get_leading_requests()
    vehicles = set(request.vehicle for request in request_to_rl)
    assert vehicles == {vehicle, vehicle3}
    for request in request_to_rl:
        assert rl_to_leading_request[request_to_rl[request]] == request
    for rl in rl_to_leading_request:
        if rl_to_leading_request[rl].vehicle.vin == 0:
            assert sum_vot[rl] == 100.1
        else:
            assert sum_vot[rl] == 5.
        assert start_idx[rl] == 0


def make_reservation(vehicle: Vehicle, t_start: int, t_end: int, i_lane: IntersectionLane) -> Reservation:
    return Reservation(vehicle, Coord(0, 0), {}, i_lane, ScheduledExit(
        vehicle, VehicleSection.FRONT, t_start, 0), exit_rear=ScheduledExit(vehicle, VehicleSection.REAR, t_end, 0))


@fixture
def requests(intersection_clean: Intersection, vehicle: Vehicle,
             vehicle2: Vehicle, vehicle3: Vehicle) -> List[Reservation]:

    requests: List[Reservation] = []
    for i, veh in enumerate([vehicle, vehicle2, vehicle3]):
        veh._Vehicle__vot = i+1
        requests.append(make_reservation(veh, 0, i+1,
                                         intersection_clean.lanes[0]))
    return requests


@fixture
def rls(intersection_clean: Intersection) -> List[RoadLane]:
    return list(intersection_clean.incoming_road_lane_by_coord.values())


@fixture
def request_to_rl(rls: List[RoadLane], requests: List[Reservation]
                  ) -> Dict[Reservation, RoadLane]:
    return {requests[i]: rls[i] for i in
            range(min(len(rls), len(requests)))}


@fixture
def rl_to_leading_request(rls: List[RoadLane], requests: List[Reservation]
                          ) -> Dict[RoadLane, Reservation]:
    return {rls[i]: requests[i] for i in
            range(min(len(rls), len(requests)))}


@fixture
def incompatible_pairs(requests: List[Reservation]
                       ) -> Set[FrozenSet[Reservation]]:
    return set((frozenset((requests[0], requests[1])),
                frozenset((requests[0], requests[2]))))


def test_incompatible(requests: List[Reservation],
                      incompatible_pairs: Set[FrozenSet[Reservation]]):

    tile = DeterministicTile(0, 0)
    tile2 = DeterministicTile(1, 0)
    tile.mark(requests[0])
    requests[0].tiles = {0: {tile: 1}}
    tile.mark(requests[1])
    requests[1].tiles = {0: {tile: 1}}
    tile2.mark(requests[0])
    requests[0].tiles[0][tile2] = 1
    tile2.mark(requests[2])
    requests[2].tiles = {0: {tile2: 1}}

    assert AuctionManager.incompatible_pairs(requests, False) == set()
    assert AuctionManager.incompatible_pairs(requests, True) == \
        incompatible_pairs


def test_request_sets(rls: List[RoadLane],
                      request_to_rl: Dict[Reservation, RoadLane],
                      incompatible_pairs: Set[FrozenSet[Reservation]]):

    singletons = AuctionManager.request_sets_to_consider(
        request_to_rl, incompatible_pairs)
    seen_rls: Set[RoadLane] = set()
    for rl_set in singletons:
        assert len(rl_set) == 1
        rl = next(iter(rl_set))
        assert rl not in seen_rls
        seen_rls.add(rl)

    request_sets = AuctionManager.request_sets_to_consider(
        request_to_rl, incompatible_pairs, True)
    assert len(request_sets) == 5
    for rl_set in request_sets:
        if len(rl_set) > 1:
            if rl_set != frozenset((rls[1], rls[2])):
                raise AssertionError()


@fixture
def request_sequence(intersection_clean: Intersection, vehicle: Vehicle,
                     vehicle2: Vehicle, vehicle3: Vehicle
                     ) -> Tuple[Reservation, Reservation, Reservation, float]:

    vehicle._Vehicle__vot = 2
    vehicle2._Vehicle__vot = 5
    vehicle3._Vehicle__vot = 10

    i_lane = intersection_clean.lanes[0]
    request = make_reservation(vehicle, 0, 100, i_lane)
    request2 = make_reservation(vehicle2, 50, 150, i_lane)
    request3 = make_reservation(vehicle3, 60, 160, i_lane)

    request.dependency = request2
    request2.dependency = request3

    request2.dependent_on = request
    request3.dependent_on = request2

    request2.predecessors = frozenset([request])
    request3.predecessors = frozenset((request, request2))

    return request, request2, request3, sum(
        (vehicle.vot, vehicle2.vot, vehicle3.vot))


def test_extend(request_sequence: Tuple[Reservation, Reservation, Reservation,
                                        float]):
    request, request2, request3, bid_vot = request_sequence
    AuctionManager.extend_sequence(request, bid_vot, 0)
    assert request2.dependency is request3

    AuctionManager.extend_sequence(request, bid_vot,
                                   request3.vehicle.vot + 1e-6)
    assert request2.dependency is None

    AuctionManager.extend_sequence(request, bid_vot, bid_vot)
    assert request.dependency is None


def test_winner(rls: List[RoadLane],
                rl_to_leading_request: Dict[RoadLane, Reservation],
                request_sequence: Tuple[Reservation,
                                        Reservation, Reservation, float]
                ):

    # Standard singletons
    sum_vot = {rls[0]: 3., rls[1]: 2., rls[2]: 4.}
    rl_sets = frozenset(frozenset([rl]) for rl in rls[:3])
    (winning_rls, winning_vot, first_losing_rls, first_losing_vot,
     all_set_vot) = AuctionManager.find_winner(
        rl_sets, rl_to_leading_request, sum_vot, Mechanism.FIRST_PRICE, False)
    assert winning_rls == frozenset([rls[2]])
    assert winning_vot == 4
    assert first_losing_rls == frozenset([rls[0]])
    assert first_losing_vot == 3
    assert len(all_set_vot) == 0

    # Empty set
    (winning_rls, winning_vot, first_losing_rls, first_losing_vot,
     all_set_vot) = AuctionManager.find_winner(
        frozenset(), rl_to_leading_request, sum_vot, Mechanism.SECOND_PRICE,
         False)
    assert len(winning_rls) == winning_vot == len(first_losing_rls) == \
        first_losing_vot == len(all_set_vot) == 0

    # Multiple
    sum_vot = {rls[0]: 3., rls[1]: 2., rls[2]: 4.}
    rl_sets2 = set(frozenset([rl]) for rl in rls[:3])
    another_set = frozenset((rls[0], rls[1]))
    rl_sets2.add(another_set)
    (winning_rls, winning_vot, first_losing_rls, first_losing_vot,
     all_set_vot) = AuctionManager.find_winner(
        frozenset(rl_sets2), rl_to_leading_request, sum_vot,
        Mechanism.EXTERNALITY, False)
    assert winning_rls == another_set
    assert winning_vot == 5
    assert first_losing_rls == frozenset([rls[2]])
    assert first_losing_vot == 4
    assert all_set_vot == {rl_set: sum(sum_vot[rl] for rl in rl_set) for rl_set
                           in rl_sets2}

    # Sequence
    sum_vot = {rls[0]: 17., rls[3]: 11.}
    rl_to_leading_request[rls[0]] = request_sequence[0]
    rl_sets3 = frozenset({frozenset([rls[0]]), frozenset([rls[3]])})
    (winning_rls, winning_vot, first_losing_rls, first_losing_vot,
     all_set_vot) = AuctionManager.find_winner(
        rl_sets3, rl_to_leading_request, sum_vot, Mechanism.SECOND_PRICE, True)
    assert winning_rls == frozenset([rls[0]])
    assert winning_vot == 17
    assert first_losing_rls == frozenset([rls[3]])
    assert first_losing_vot == 11
    assert len(all_set_vot) == 0
    request_leader = rl_to_leading_request[rls[0]]
    assert request_leader.dependency is not None
    assert request_leader.dependency.dependency is None


def test_t_occupied(rls: List[RoadLane],
                    rl_to_leading_request: Dict[RoadLane, Reservation]):
    assert AuctionManager.t_occupied(
        frozenset([rls[0]]), rl_to_leading_request) == \
        approx(rl_to_leading_request[rls[0]].exit_rear.t *
               SHARED.SETTINGS.TIMESTEP_LENGTH)

    assert AuctionManager.t_occupied(
        frozenset([rls[0], rls[1], rls[2]]), rl_to_leading_request) == \
        approx(rl_to_leading_request[rls[2]].exit_rear.t *
               SHARED.SETTINGS.TIMESTEP_LENGTH)


def test_price_correction():
    assert AuctionManager.price_correction(0, 0, Mechanism.FIRST_PRICE) == 1
    assert AuctionManager.price_correction(
        3, 1.5, Mechanism.SECOND_PRICE) == .5
    assert AuctionManager.price_correction(
        3, 1.5, Mechanism.FIRST_PRICE) == 1


def test_simple_payment(rls: List[RoadLane], vehicle: Vehicle,
                        vehicle2: Vehicle, vehicle3: Vehicle,
                        rl_to_leading_request: Dict[RoadLane, Reservation]):

    with raises(ValueError):
        AuctionManager.payment_simple(
            frozenset(), rl_to_leading_request, {}, 0, 0,
            Mechanism.EXTERNALITY)

    # Empty
    assert len(AuctionManager.payment_simple(
        frozenset(), rl_to_leading_request, {}, 0, 0,
        Mechanism.SECOND_PRICE)) == 0

    start_idx: Dict[RoadLane, int] = DefaultDict(lambda: 0)
    rls[0].vehicles.append(vehicle)

    # NO SEQUENCE
    # First price
    assert AuctionManager.payment_simple(frozenset(
        [rls[0]]), rl_to_leading_request, start_idx, 1, .1,
        Mechanism.FIRST_PRICE) == approx(
            {vehicle: vehicle.vot * (vehicle.vin+1) *
             SHARED.SETTINGS.TIMESTEP_LENGTH})
    # Second price
    assert AuctionManager.payment_simple(frozenset(
        [rls[0]]), rl_to_leading_request, start_idx, 5, 4,
        Mechanism.SECOND_PRICE) == approx(
            {vehicle: vehicle.vot * (vehicle.vin+1) *
             SHARED.SETTINGS.TIMESTEP_LENGTH * 4/5})

    # MULTIPLE
    rls[1].vehicles.append(vehicle2)
    t_consumed = (rl_to_leading_request[rls[1]].exit_rear.t) * \
        SHARED.SETTINGS.TIMESTEP_LENGTH
    # First price
    payment = AuctionManager.payment_simple(frozenset(
        [rls[0], rls[1]]), rl_to_leading_request, start_idx, 3, 2,
        Mechanism.FIRST_PRICE)
    assert payment[vehicle] == approx(vehicle.vot * t_consumed)
    assert payment[vehicle2] == approx(vehicle2.vot * t_consumed)
    assert payment[vehicle3] == 0
    # Second price
    payment = AuctionManager.payment_simple(frozenset(
        [rls[0], rls[1]]), rl_to_leading_request, start_idx,
        vehicle.vot + vehicle2.vot, 2, Mechanism.SECOND_PRICE)
    assert payment[vehicle] == approx(
        vehicle.vot/(vehicle.vot + vehicle2.vot) * t_consumed * 2)
    assert payment[vehicle2] == approx(
        vehicle2.vot/(vehicle.vot + vehicle2.vot) * t_consumed * 2)
    assert payment[vehicle3] == 0


def test_simple_payment_sequence_first(
        rls: List[RoadLane], vehicle: Vehicle, vehicle2: Vehicle,
        vehicle3: Vehicle, rl_to_leading_request: Dict[RoadLane, Reservation],
        request_sequence: Tuple[Reservation, Reservation, Reservation, float]):
    start_idx: Dict[RoadLane, int] = DefaultDict(lambda: 0)
    rls[0].vehicles.extend([vehicle, vehicle2, vehicle3])
    rl_to_leading_request[rls[0]] = request_sequence[0]
    payments = AuctionManager.payment_simple(frozenset(
        [rls[0]]), rl_to_leading_request, start_idx,
        vehicle.vot + vehicle2.vot + vehicle3.vot, vehicle3.vot/2,
        Mechanism.FIRST_PRICE)
    assert len(payments) == 3
    t_first = request_sequence[0].exit_rear.t * SHARED.SETTINGS.TIMESTEP_LENGTH
    assert payments[vehicle] == approx(vehicle.vot * t_first)
    t_second = request_sequence[1].exit_rear.t * \
        SHARED.SETTINGS.TIMESTEP_LENGTH
    assert payments[vehicle2] == approx(vehicle2.vot * t_second)
    t_third = request_sequence[2].exit_rear.t * \
        SHARED.SETTINGS.TIMESTEP_LENGTH
    assert payments[vehicle3] == approx(vehicle3.vot * t_third)


def test_simple_payment_sequence_second(
        rls: List[RoadLane], vehicle: Vehicle, vehicle2: Vehicle,
        vehicle3: Vehicle, rl_to_leading_request: Dict[RoadLane, Reservation],
        request_sequence: Tuple[Reservation, Reservation, Reservation, float]):
    start_idx: Dict[RoadLane, int] = DefaultDict(lambda: 0)
    rls[0].vehicles.extend([vehicle, vehicle2, vehicle3])
    rl_to_leading_request[rls[0]] = request_sequence[0]
    winner = vehicle.vot + vehicle2.vot + vehicle3.vot
    loser = vehicle3.vot/2
    payments = AuctionManager.payment_simple(frozenset(
        [rls[0]]), rl_to_leading_request, start_idx,
        winner, loser, Mechanism.SECOND_PRICE)
    assert len(payments) == 3

    # All three vehicles contribute to winning the 1-length sequence
    t_first = request_sequence[0].exit_rear.t * SHARED.SETTINGS.TIMESTEP_LENGTH
    assert payments[vehicle] == approx(
        (vehicle.vot / winner * loser) * t_first)

    # Only the second and third vehicles contribute to winning the 2-length sequence
    t_second = request_sequence[1].exit_rear.t * \
        SHARED.SETTINGS.TIMESTEP_LENGTH - t_first
    assert payments[vehicle2] == approx(
        (vehicle2.vot / winner * loser) * t_first +
        (vehicle2.vot / (vehicle2.vot + vehicle3.vot) * loser) * t_second)

    # Only the third vehicle contributes to winning its own sequence
    t_third = request_sequence[2].exit_rear.t * \
        SHARED.SETTINGS.TIMESTEP_LENGTH - t_second - t_first
    assert payments[vehicle3] == approx(
        loser * t_third +
        (vehicle3.vot / (vehicle2.vot + vehicle3.vot) * loser) * t_second +
        (vehicle3.vot / winner * loser) * t_first)


def test_t_movement(request_sequence: Tuple[Reservation, Reservation,
                                            Reservation, float]):
    assert AuctionManager.t_movement(request_sequence[0]) == \
        AuctionManager.t_movement(request_sequence[1]) == \
        AuctionManager.t_movement(request_sequence[2]) == \
        100*SHARED.SETTINGS.TIMESTEP_LENGTH


def test_split_losers(rls: List[RoadLane]):

    # Empty case
    assert AuctionManager.split_losers(0., 0., 0., frozenset(
    ), frozenset(), frozenset(rls)) == (frozenset(), frozenset(rls))

    # Removing a vehicle doesn't trigger a switch from the actual winning set
    # to one that doesn't involve the winning road lane
    assert AuctionManager.split_losers(10., 9.9, 0., frozenset(
        [rls[0]]), frozenset([rls[1]]), frozenset(rls)) == (
            frozenset([rls[0]]), frozenset(rls[1:]))

    # Removing a vehicle causes the winner to switch from the present lane to
    # the next winning lane
    assert AuctionManager.split_losers(10., 9.9, 9., frozenset(
        [rls[0]]), frozenset([rls[1]]), frozenset(rls)) == (
            frozenset([rls[1]]), frozenset(rls[2:]))

    # Multiple, no switch.
    assert AuctionManager.split_losers(10., 9.9, 0., frozenset(
        rls[0:3]), frozenset(rls[3:5]), frozenset(rls)
    ) == (frozenset(rls[0:3]), frozenset(rls[3:]))

    # Multiple, switch.
    assert AuctionManager.split_losers(10., 9.9, 0.2, frozenset(
        rls[0:3]), frozenset(rls[3:5]), frozenset(rls)
    ) == (frozenset(rls[3:5]), frozenset(rls[5:]))


def test_externality(rls: List[RoadLane]):

    # Empty result. Should never trigger since the function where externality
    # is used has a short circuit to skip this calculation because it's just
    # going to be 0.
    assert AuctionManager.externality(
        0, 10, 5, frozenset(), frozenset(), frozenset(), {}, {}, {}) == 0

    # Removing vehicle i swaps the winning road lane bid from 6 to 5.
    assert AuctionManager.externality(
        2, 5, 2, frozenset(rls[:1]), frozenset(rls[1:2]), frozenset(rls[2:3]),
        {rls[0]: 6, rls[1]: 5, rls[2]: 0},
        {rls[0]: .1, rls[1]: .2, rls[2]: .3}, DefaultDict(lambda: .01)) == \
        (6 + .001*2/2 - 2)*2 - (5 + .002*5/2) * 5  # ~-17.023

    # Removing vehicle i swaps the winning road lane set bid from 6 to 5.
    assert AuctionManager.externality(
        2, 2, 5, frozenset(rls[:2]), frozenset(rls[2:4]), frozenset(rls[4:6]),
        {rls[0]: 3, rls[1]: 3, rls[2]: 2, rls[3]: 3, rls[4]: 4, rls[5]: .1},
        DefaultDict(lambda: .1),
        {rl: i+1 for i, rl in enumerate(rls[:6])}) == \
        (6 + (.1+.2)*5/2 - 2)*5 - (5 + (.3+.4)*2/2)*2 + \
        (4.1 + (.5+.6)*(5-2)/2)*(5-2)  # 29.6


def test_calculate_externality(rls: List[RoadLane]):

    # Empty case
    assert AuctionManager.calculate_externality(
        0, 0, 0, frozenset(), frozenset(), frozenset(), 1, 1, {}, {}, {}) == 0

    # No switch case
    assert AuctionManager.calculate_externality(1, 5, 3, frozenset(
        rls[0:1]), frozenset(rls[1:2]), frozenset(rls), 2, 1, {}, {}, {}) == 0

    # Multiple switch. This is the same case as the lst test_externality case.
    assert AuctionManager.calculate_externality(
        2, 6, 5, frozenset(rls[:2]), frozenset(rls[2:4]), frozenset(rls[:6]),
        2, 5, {rls[0]: 3, rls[1]: 3, rls[2]: 2, rls[3]: 3, rls[4]: 4,
               rls[5]: .1}, DefaultDict(lambda: .1),
        {rl: i+1 for i, rl in enumerate(rls[:6])}) == 29.6


def test_payment_externality_multiple(rls: List[RoadLane], vehicle: Vehicle,
                                      vehicle2: Vehicle, vehicle3: Vehicle,
                                      intersection_clean: Intersection):

    winning_rls = frozenset(rls[:2])
    first_losing_rls = frozenset(rls[2:4])
    all_rls = frozenset(rls[:6])
    sum_vot: Dict[RoadLane, float] = {rls[0]: 3, rls[1]: 3, rls[2]: 2,
                                      rls[3]: 3, rls[4]: 4, rls[5]: .1}
    vps_mean: Dict[RoadLane, float] = DefaultDict(lambda: .1)
    vot_mean: Dict[RoadLane, float] = {rl: i+1 for i, rl in enumerate(rls[:6])}
    rls[0].vehicles = [vehicle, vehicle3]
    vehicle._Vehicle__vot = vehicle3._Vehicle__vot = 1.5
    rls[1].vehicles = [vehicle2]
    i_lane = intersection_clean.lanes[0]
    payments = AuctionManager.payment_externality(
        winning_rls, 6, 5, all_rls, {
            rls[0]: make_reservation(
                vehicle, 0, 5*SHARED.SETTINGS.steps_per_second, i_lane),
            rls[1]: make_reservation(vehicle2, 0, 5, i_lane),
            rls[2]: make_reservation(
                vehicle3, 0, 2*SHARED.SETTINGS.steps_per_second, i_lane),
            rls[3]: make_reservation(vehicle2, 0, 0, i_lane),
            rls[4]: make_reservation(vehicle2, 0, 0, i_lane),
            rls[5]: make_reservation(vehicle2, 0, 0, i_lane)
        }, {winning_rls: 6, frozenset([rls[0]]): 5.5, first_losing_rls: 5
            }, DefaultDict(lambda: 0), sum_vot, vps_mean, vot_mean)
    assert len(payments) == 3
    assert payments[vehicle] == payments[vehicle3]
    assert payments[vehicle] == -AuctionManager.calculate_externality(
        vehicle.vot, 6, 5, winning_rls, first_losing_rls, all_rls,
        5, 2, sum_vot, vps_mean, vot_mean)
    assert payments[vehicle2] == 0.


def test_payment_externality_sequence(
    rls: List[RoadLane], vehicle: Vehicle, vehicle2: Vehicle,
    vehicle3: Vehicle,
    request_sequence: Tuple[Reservation, Reservation, Reservation, float],
        intersection_clean: Intersection):

    winning_rls = frozenset(rls[:1])
    first_losing_rls = frozenset(rls[1:2])
    all_rls = frozenset(rls[:6])
    sum_vot: Dict[RoadLane, float] = {rls[0]: 17, rls[1]: 11, rls[2]: 1,
                                      rls[3]: 0, rls[4]: 0, rls[5]: 0}
    vps_mean: Dict[RoadLane, float] = DefaultDict(lambda: .1)
    vot_mean: Dict[RoadLane, float] = {rl: i+1 for i, rl in enumerate(rls[:6])}
    rls[0].vehicles = [vehicle, vehicle2, vehicle3]
    rls[1].vehicles = [vehicle2]

    # Cut off the request sequence at 2 vehicles so we can check if the payment
    # calculation for non-moving vehicles in the winning lane is correct.
    request_sequence[1].dependency = None

    i_lane = intersection_clean.lanes[0]
    t_first_loser = 1
    payments = AuctionManager.payment_externality(
        winning_rls, 17, 11, all_rls, {
            rls[0]: request_sequence[0],
            rls[1]: make_reservation(
                vehicle2, 0, t_first_loser*SHARED.SETTINGS.steps_per_second,
                i_lane),
            rls[2]: make_reservation(vehicle2, 0, 0, i_lane)
        }, {winning_rls: 17, first_losing_rls: 11, all_rls: 1,
            }, DefaultDict(lambda: 0), sum_vot, vps_mean, vot_mean)
    assert len(payments) == 3
    t1 = AuctionManager.t_movement(request_sequence[0])
    t2_delta = request_sequence[1].exit_rear.t * \
        SHARED.SETTINGS.TIMESTEP_LENGTH - t1
    t2_loser = max(t_first_loser-t1, 0)
    assert payments[vehicle] == -AuctionManager.calculate_externality(
        vehicle.vot, 17, 11, winning_rls, first_losing_rls, all_rls,
        t1, t_first_loser, sum_vot, vps_mean, vot_mean)
    assert payments[vehicle2] == approx(-AuctionManager.calculate_externality(
        vehicle2.vot, 17, 11, winning_rls, first_losing_rls, all_rls,
        t1, t_first_loser, sum_vot, vps_mean, vot_mean) -
        AuctionManager.calculate_externality(
            vehicle2.vot, 15, 11, winning_rls, first_losing_rls, all_rls,
            t2_delta, t2_loser, sum_vot, vps_mean, vot_mean))
    assert payments[vehicle3] == approx(-AuctionManager.calculate_externality(
        vehicle3.vot, 17, 11, winning_rls, first_losing_rls, all_rls,
        t1, t_first_loser, sum_vot, vps_mean, vot_mean) -
        AuctionManager.calculate_externality(
        vehicle3.vot, 15, 11, winning_rls, first_losing_rls, all_rls,
        t2_delta, t2_loser, sum_vot, vps_mean, vot_mean))

    # Cut off the request sequence at 2 vehicles so we can check if the payment
    # calculation for non-moving vehicles in the winning lane is correct.
    t_first_loser = 2
    request_sequence[1].dependency = None
    payments = AuctionManager.payment_externality(
        winning_rls, 17, 11, all_rls, {
            rls[0]: request_sequence[0],
            rls[1]: make_reservation(
                vehicle2, 0, t_first_loser*SHARED.SETTINGS.steps_per_second,
                i_lane),
            rls[2]: make_reservation(vehicle2, 0, 0, i_lane)
        }, {winning_rls: 17, first_losing_rls: 11, all_rls: 1,
            }, DefaultDict(lambda: 0), sum_vot, vps_mean, vot_mean)
    assert len(payments) == 3
    t1 = AuctionManager.t_movement(request_sequence[0])
    t2_delta = request_sequence[1].exit_rear.t * \
        SHARED.SETTINGS.TIMESTEP_LENGTH - t1
    t2_loser = max(t_first_loser-t1, 0)
    assert payments[vehicle] == -AuctionManager.calculate_externality(
        vehicle.vot, 17, 11, winning_rls, first_losing_rls, all_rls,
        t1, t_first_loser, sum_vot, vps_mean, vot_mean)
    assert payments[vehicle2] == approx(-AuctionManager.calculate_externality(
        vehicle2.vot, 17, 11, winning_rls, first_losing_rls, all_rls,
        t1, t_first_loser, sum_vot, vps_mean, vot_mean) -
        AuctionManager.calculate_externality(
            vehicle2.vot, 15, 11, winning_rls, first_losing_rls, all_rls,
            t2_delta, t2_loser, sum_vot, vps_mean, vot_mean))
    assert payments[vehicle3] == approx(-AuctionManager.calculate_externality(
        vehicle3.vot, 17, 11, winning_rls, first_losing_rls, all_rls,
        t1, t_first_loser, sum_vot, vps_mean, vot_mean) -
        AuctionManager.calculate_externality(
        vehicle3.vot, 15, 11, winning_rls, first_losing_rls, all_rls,
        t2_delta, t2_loser, sum_vot, vps_mean, vot_mean))


def test_exiting(vehicle: Vehicle, intersection: Intersection):
    assert type(intersection.manager) is AuctionManager
    vehicle.trailing = True
    intersection.manager.payments[vehicle] = -1
    intersection.manager.finish_exiting(vehicle)
    assert vehicle.payment == 0
    assert vehicle not in intersection.manager.payments

    intersection.manager.floor = False
    intersection.manager.payments[vehicle] = -1
    intersection.manager.finish_exiting(vehicle)
    assert vehicle.payment == -1
    assert vehicle not in intersection.manager.payments


def test_process_first(intersection: Intersection, vehicle: Vehicle,
                       vehicle2: Vehicle, vehicle3: Vehicle):
    assert type(intersection.manager) is AuctionManager

    intersection.manager.process_requests()
    payments = intersection.manager.payments
    assert len(payments) == 2
    assert payments[vehicle] == approx(.503333333)
    assert payments[vehicle2] == approx(503.33333333)
    assert vehicle3 not in payments


def test_process_second(intersection: Intersection, vehicle: Vehicle,
                        vehicle2: Vehicle, vehicle3: Vehicle):
    assert type(intersection.manager) is AuctionManager
    intersection.manager.payments = DefaultDict(lambda: 0.)
    intersection.manager.mechanism = Mechanism.SECOND_PRICE
    intersection.manager.process_requests()
    payments = intersection.manager.payments
    assert len(payments) == 2
    assert payments[vehicle] < .503333333
    assert payments[vehicle2] < 503.33333333
    assert vehicle3 not in payments


def test_process_externality(intersection: Intersection, vehicle: Vehicle,
                             vehicle2: Vehicle, vehicle3: Vehicle):
    assert type(intersection.manager) is AuctionManager
    intersection.manager.vps_mean = DefaultDict(lambda: 0.)
    intersection.manager.vot_mean = DefaultDict(lambda: 0.)
    intersection.manager.payments = DefaultDict(lambda: 0.)
    intersection.manager.mechanism = Mechanism.EXTERNALITY
    intersection.manager.process_requests()
    payments = intersection.manager.payments
    assert len(payments) == 2
    assert payments[vehicle] == 0.
    assert payments[vehicle2] > 0.
    assert vehicle3 not in payments
