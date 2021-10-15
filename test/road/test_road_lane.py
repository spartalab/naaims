from math import ceil, log10

from _pytest.python_api import approx
from pytest import raises

import naaims.shared as SHARED
from naaims.road.lane import RoadLane, ScheduledExit
from naaims.vehicles import AutomatedVehicle
from naaims.lane import VehicleProgress
from naaims.util import VehicleSection

from test.test_lane import straight_trajectory


def test_road_lane_init(load_shared: None, rl: RoadLane):
    assert rl.entrance_end == 0.3
    assert rl.lcregion_end == 0.7
    with raises(ValueError):
        rl.connect_downstream_intersection(None)

    rl = RoadLane(straight_trajectory, 5, 30, .2*straight_trajectory.length,
                  .45*straight_trajectory.length)
    assert rl.entrance_end == .2
    assert rl.lcregion_end == .55
    assert rl.room_to_enter() == .2*straight_trajectory.length
    assert hash(rl) == hash(straight_trajectory)


def test_control(rl: RoadLane, vehicle: AutomatedVehicle):

    # Normal case
    rl.vehicles.append(vehicle)
    rl.vehicle_progress[vehicle] = VehicleProgress(0.2, 0.1, 0)
    b, p, s = rl.controls_this_speed(vehicle)
    assert b
    assert p == 0.2
    assert s == VehicleSection.FRONT

    # Rear is in intersection, not in control
    rl.vehicle_progress[vehicle] = VehicleProgress(0.1, 0, None)
    b, p, s = rl.controls_this_speed(vehicle)
    assert not b
    assert p == -float("inf")
    assert s == VehicleSection.FRONT

    # Rear is in spawner, in control
    rl.upstream_is_spawner = True
    b, p, s = rl.controls_this_speed(vehicle)
    assert b
    assert p == 0.1
    assert s == VehicleSection.FRONT

    # Front is in remover, in control
    rl.vehicle_progress[vehicle] = VehicleProgress(None, 0.1, 0)
    b, p, s = rl.controls_this_speed(vehicle)
    assert b
    assert p == 0
    assert s == VehicleSection.REAR

    # Front is in intersection, not in control
    rl.downstream_is_remover = False
    b, p, s = rl.controls_this_speed(vehicle)
    assert not b
    assert p == float("inf")
    assert s == VehicleSection.REAR


def test_accel_update(rl: RoadLane, vehicle: AutomatedVehicle,
                      vehicle2: AutomatedVehicle):
    rl.downstream_is_remover = False

    # No preceding but no permission to enter intersection, so brake.
    vehicle.velocity = rl.speed_limit/2
    assert rl.accel_update(vehicle, VehicleSection.FRONT,
                           1, None) == SHARED.SETTINGS.min_braking

    # No preceding and permission to enter intersection, so accelerate.
    vehicle.velocity = rl.speed_limit/2
    vehicle.permission_to_enter_intersection = True
    assert rl.accel_update(vehicle, VehicleSection.FRONT,
                           1, None) == SHARED.SETTINGS.min_acceleration

    # No preceding and permission to enter, at speed limit, so 0.
    vehicle.velocity = rl.speed_limit
    assert rl.accel_update(vehicle, VehicleSection.FRONT,
                           1, None) == 0

    # Preceding and no permission to enter.
    # Nice and far from both preceding and intersection.
    # Not at speed limit.
    # Accelerate.
    vehicle.permission_to_enter_intersection = False
    vehicle.velocity = rl.speed_limit/2
    rl.vehicle_progress[vehicle2] = VehicleProgress(.5, .5, .5)
    assert rl.accel_update(vehicle, VehicleSection.FRONT,
                           0, vehicle2) == SHARED.SETTINGS.min_acceleration

    # Preceding and no permission to enter.
    # Nice and far from both preceding and intersection.
    # At speed limit.
    # Maintain speed.
    vehicle.velocity = rl.speed_limit
    rl.vehicle_progress[vehicle2] = VehicleProgress(.5, .5, .5)
    assert rl.accel_update(vehicle, VehicleSection.FRONT,
                           0, vehicle2) == 0

    # Preceding and no permission to enter.
    # Close to intersection and preceding.
    # At speed limit.
    # Brake for intersection.
    rl.vehicle_progress[vehicle2] = VehicleProgress(1, 1, 1)
    assert rl.accel_update(vehicle, VehicleSection.FRONT,
                           .99, vehicle2) == SHARED.SETTINGS.min_braking

    # Preceding and no permission to enter.
    # Close to intersection and preceding.
    # Not at speed limit.
    # Brake for intersection.
    vehicle.velocity = rl.speed_limit/2
    assert rl.accel_update(vehicle, VehicleSection.FRONT,
                           .99, vehicle2) == SHARED.SETTINGS.min_braking

    # Preceding and permission to enter.
    # Close to intersection and preceding.
    # Not at speed limit.
    # Brake for preceding vehicle.
    vehicle.permission_to_enter_intersection = True
    vehicle.velocity = rl.speed_limit/2
    assert rl.accel_update(vehicle, VehicleSection.FRONT,
                           .99, vehicle2) == SHARED.SETTINGS.min_braking

    # Preceding and permission to enter.
    # Close to preceding but not intersection.
    # At speed limit.
    # Brake for preceding.
    vehicle.velocity = rl.speed_limit
    rl.vehicle_progress[vehicle2] = VehicleProgress(.5, .5, .5)
    assert rl.accel_update(vehicle, VehicleSection.FRONT,
                           .49, vehicle2) == SHARED.SETTINGS.min_braking


def test_speed_update(rl: RoadLane):
    rl.downstream_is_remover = False
    with raises(RuntimeError):
        rl.get_new_speeds()


def test_downstream_sd(rl: RoadLane, vehicle: AutomatedVehicle,
                       vehicle2: AutomatedVehicle):
    assert rl.downstream_stopping_distance() is None


def test_exited(rl: RoadLane):
    assert rl.has_vehicle_exited(VehicleProgress())
    assert rl.has_vehicle_exited(VehicleProgress(None, None, 1))
    assert not rl.has_vehicle_exited(VehicleProgress(None, 1, 1))
    assert not rl.has_vehicle_exited(VehicleProgress(1, 1, 1))


def test_remove(rl: RoadLane, vehicle: AutomatedVehicle,
                vehicle2: AutomatedVehicle):
    rl.add_vehicle(vehicle)
    rl.add_vehicle(vehicle2)
    assert len(rl.vehicles) == 2
    rl.latest_scheduled_exit = ScheduledExit(
        vehicle, VehicleSection.REAR, 0, 0)
    rl.remove_vehicle(vehicle)
    assert len(rl.vehicles) == 1
    assert rl.vehicles[0] == vehicle2
    assert vehicle2 in rl.vehicle_progress
    assert vehicle not in rl.vehicle_progress
    assert rl.latest_scheduled_exit is None

    rl.add_vehicle(vehicle)
    rl.latest_scheduled_exit = ScheduledExit(
        vehicle, VehicleSection.REAR, 0, 0)
    rl.remove_vehicle(vehicle2)
    assert rl.latest_scheduled_exit is not None


def test_room(rl: RoadLane, vehicle: AutomatedVehicle):
    assert rl.room_to_enter() == 300  # see conftest.py
    rl.add_vehicle(vehicle)
    rl.vehicle_progress[vehicle] = VehicleProgress(None, None, .2)
    assert rl.room_to_enter() == 200
    rl.vehicle_progress[vehicle] = VehicleProgress(None, None, .5)
    assert rl.room_to_enter() == 300


# def test_first_permission in test_integration because it depends on several
# different components working together.


def test_x_to_intersection(rl: RoadLane):
    assert rl._x_to_intersection(.5) == 500
    assert rl._x_to_intersection(.1) == 900
    assert rl._x_to_intersection(0) == 1000
    assert rl._x_to_intersection(1) == 0


def test_ff_exit():
    assert RoadLane._free_flow_exit(0, 1, 10, 10, 50, 50) == (10, 10)
    assert RoadLane._free_flow_exit(0, 1, 10, 10, 50, 10) == approx(
        (4.47213595499958, 4.47213595499958))
    assert RoadLane._free_flow_exit(0, 1, 10, 10, 50, 60) == (11, 10)
    t_odd = 5**.5-1
    assert RoadLane._free_flow_exit(1, 1, 3, 2, 4, 2) == (t_odd, 1+t_odd)
    assert RoadLane._free_flow_exit(1, 1, 3, 2, 4, 7) == (3, 3)


def test_x_in_intersection():
    assert RoadLane._x_in_intersection(0, 1, 1, 3, 1) == 1.5
    assert RoadLane._x_in_intersection(0, 2, 1, 3, 1) == 2


def test_enough_separation(vehicle: AutomatedVehicle):

    # This exit can never catch up to last exit regardless of its velocity.
    assert RoadLane._enough_separation(0, 1, 1, 100, 0, 10)

    # Trailing vehicle just catches up in intersection.
    assert RoadLane._enough_separation(1, 2, 1, 2, 2, 2)

    # Trailing vehicle overtakes in intersection due to high speed at entrance.
    assert not RoadLane._enough_separation(0, 2, 1, 2, 2, 2)

    # Trailing vehicle doesn't reach speed limit before end of window. I don't
    # think it's possible for the trailer to be unable to reach the speed limit
    # before the end of window AND overtake.
    assert RoadLane._enough_separation(3, 3, 1, 9, 9, 81/2)


def test_exactly_enough_separation(vehicle: AutomatedVehicle):

    # This exit can never catch up to last exit regardless of its velocity.
    assert RoadLane._exactly_enough_separation(0, 1, 1, 100, 0, 10) > 0

    # Trailing vehicle just catches up in intersection.
    assert RoadLane._exactly_enough_separation(1, 2, 1, 2, 2, 2) == 0

    # Trailing vehicle overtakes in intersection due to high speed at entrance.
    assert RoadLane._exactly_enough_separation(0, 2, 1, 2, 2, 2) < 0

    # Trailing vehicle doesn't reach speed limit before end of window. I don't
    # think it's possible for the trailer to be unable to reach the speed limit
    # before the end of window AND overtake.
    assert RoadLane._exactly_enough_separation(3, 3, 1, 9, 9, 81/2) > 0


def test_slowest_exit_complete_stop():

    # Should be guaranteed to need to brake before reaching v_max.
    v0, a, b, x = 5, 2, -3, 20
    t_slowest, t_brake, t_brake_largest, t_brake_smallest = \
        RoadLane._slowest_exit_complete_stop(v0, a, b, 10, 1000, x, 1000)
    t = RoadLane._slowest_exit_brake_before_v_max(v0, a, b, x)
    assert t_slowest == t
    assert t_brake == RoadLane._t_brake_given_t_slowest(t, v0, a, b)
    assert t_brake_largest is None
    assert t_brake_smallest == t_brake

    # Brakes after reaching v_max
    t_slowest, t_brake, t_brake_largest, t_brake_smallest = \
        RoadLane._slowest_exit_complete_stop(1, 1, -2, 2, 1.5, 100, 1)
    assert t_slowest == 97.5/2 + 1 + 1
    assert t_brake == 1
    assert t_brake_largest == t_brake
    assert t_brake_smallest is None


def test_slowest_exit_brake_before_intersection():
    brake_before_v_max_helper(5, 2, -3, 20)
    brake_before_v_max_helper(1, 2, -3, 100)
    brake_before_v_max_helper(50, 2, -3, 5)
    brake_before_v_max_helper(1, 5, -2, 100)
    brake_before_v_max_helper(50, 1, -6, 5)


def brake_before_v_max_helper(v0: float, a: float, b: float, x: float):
    t = RoadLane._slowest_exit_brake_before_v_max(v0, a, b, x)
    assert t > 0
    t_b = RoadLane._t_brake_given_t_slowest(t, v0, a, b)
    t_a = t - t_b
    x_a = RoadLane.x_over_constant_a(v0, a, t_a)
    x_b = RoadLane.x_over_constant_a(v0+a*t_a, b, t_b)
    assert x_a + x_b == approx(x)


def test_slowest_exit():

    # Complete stop case 1: Should brake before reaching v_max.
    v0, a, b, x = 5, 2, -3, 20
    t_slowest, v_slowest, t_brake, t_brake_largest, t_brake_smallest = \
        RoadLane._slowest_exit(v0, a, b, 10, 1000, x, 1000)
    t = RoadLane._slowest_exit_brake_before_v_max(v0, a, b, x)
    assert t_slowest == t
    assert v_slowest == 0
    assert t_brake == RoadLane._t_brake_given_t_slowest(t, v0, a, b)
    assert t_brake_largest is None
    assert t_brake_smallest == t_brake

    # Complete stop case 2: Brakes after reaching v_max
    t_slowest, t_brake, t_brake_largest, t_brake_smallest = \
        RoadLane._slowest_exit_complete_stop(1, 1, -2, 2, 1.5, 100, 1)
    assert t_slowest == 97.5/2 + 1 + 1
    assert t_brake == 1
    assert t_brake_largest == t_brake
    assert t_brake_smallest is None

    # Case 3: Full brake does not come to a complete stop.
    v0, a, b, x = 8, 2, -3, 1
    t_slowest, v_slowest, t_brake, t_brake_largest, t_brake_smallest = \
        RoadLane._slowest_exit(v0, a, b, 10, 9, x, 1)
    t = 0.12808
    assert t_slowest == t_brake == t_brake_smallest == approx(t, abs=1e-4)
    assert v_slowest == approx(v0 + b*t, abs=1e-4)
    assert t_brake_largest is None

    # Case 4: Full brake at v_max, but no time to stop.
    v0, a, b, x = 10, 2, -3, 0
    t_slowest, v_slowest, t_brake, t_brake_largest, t_brake_smallest = \
        RoadLane._slowest_exit(v0, a, b, v0, 0, x, 0)
    assert t_slowest == t_brake == t_brake_largest == approx(0, abs=1e-4)
    assert v_slowest == approx(v0, abs=1e-4)
    assert t_brake_smallest is None


def test_x_at_v_max():
    assert RoadLane._x_at_v_max(100, 10, 15) == 75
    assert RoadLane._x_at_v_max(99, 22, 33) == 44


def test_t_of_v_max():
    assert RoadLane._t_of_v_max_exit(10, 15, 50, 5) == 35


def test_reaches_v_max_before():
    # Test case:
    # 10 m/s v_max at -1 m/s^2
    # Takes 10s to reach 0 m/s covering 50m
    # Scenarios that allow more than 50m to brake will always reach v_max

    # Spends some time at v_max
    does, t_largest, t_smallest = RoadLane._reaches_v_max_before_intersection(
        10, None, None, -1, 10, 10, 100)
    assert does
    assert t_largest == 10
    assert t_smallest is None

    # Way too little to reach v_max
    does, t_largest, t_smallest = RoadLane._reaches_v_max_before_intersection(
        10, None, None, -1, 10, 10, 11)
    assert not does
    assert t_largest is None
    assert t_smallest == 10

    # Exactly at v_max
    does, t_largest, t_smallest = RoadLane._reaches_v_max_before_intersection(
        10, None, None, -1, 10, 10, 60)
    assert does
    assert t_largest == 10
    assert t_smallest is None

    # If the distance is 40m, this is the max t_brake that still reaches v_max
    t_crit = 2*(5-5**.5)

    # t_brake is less than the max possible to reach v_max
    does, t_largest, t_smallest = RoadLane._reaches_v_max_before_intersection(
        t_crit-1e-6, None, None, -1, 10, 10, 50)
    assert does
    assert t_largest == t_crit-1e-6
    assert t_smallest is None

    # t_brake is exactly the threshold seen at v_max
    does, t_largest, t_smallest = RoadLane._reaches_v_max_before_intersection(
        t_crit, t_crit-1e-6, None, -1, 10, 10, 50)
    assert does
    assert t_largest == t_crit
    assert t_smallest is None

    # t_brake is much less than the threshold seen at v_max, and we remember
    does, t_largest, t_smallest = RoadLane._reaches_v_max_before_intersection(
        t_crit/2, t_crit, None, -1, 10, 10, 50)
    assert does
    assert t_largest == t_crit
    assert t_smallest is None

    # t_brake is a little larger than threshold to hit v_max, so it brakes
    does, t_largest, t_smallest = RoadLane._reaches_v_max_before_intersection(
        t_crit+1e-6, t_crit, None, -1, 10, 10, 50)
    assert not does
    assert t_largest == t_crit
    assert t_smallest == t_crit+1e-6


def test_parameterized_exit():
    # Test case:
    # 10 m/s v_max at -1 m/s^2
    # Takes 10s to reach 0 m/s covering 50m
    # Scenarios that allow more than 50m to brake will always reach v_max
    # If the distance is 40m, this is the max t_brake that still reaches v_max

    # Reaches v_max: Speed 0 to v_max to 0
    t, v = RoadLane._parameterized_exit(10, True, 0, 1, -1, 10, 100, 50, 10)
    assert t == 20
    assert v == 0

    # Doesn't reach v_max: Speed 0 to 1 m/s under v_max to 0
    t, v = RoadLane._parameterized_exit(9, False, 0, 1, -1, 10, 81, 40.5, 10)
    assert t == 18
    assert v == 0

    # Reaches v_max from nonzero v0: Speed 1 to v_max to 0
    t, v = RoadLane._parameterized_exit(9, False, 1, 1, -1, 10, 80.5, 40, 9)
    assert t == 17
    assert v == 0

    # Reaches v_max and goes to nonzero end v: Speed 0 to v_max to 1
    t, v = RoadLane._parameterized_exit(9, True, 0, 1, -1, 10, 99.5, 50, 10)
    assert t == 19
    assert v == 1

    # Doesn't v_max and goes to nonzero v0: Speed 0 to 1 m/s under v_max to 1
    t, v = RoadLane._parameterized_exit(8, False, 0, 1, -1, 10, 80.5, 40, 10)
    assert t == 17
    assert v == 1

    # Reaches v_max, asymmetrical a, b: Speed to v_max to 0
    t, v = RoadLane._parameterized_exit(5, True, 0, 1, -2, 10, 75, 50, 10)
    assert t == 15
    assert v == 0

    # Doesn't reach v_max, asymmetrical a, b: Speed to v_max to 0
    t, v = RoadLane._parameterized_exit(4, False, 0, 1, -2, 10, 48, 50, 10)
    assert t == 12
    assert v == 0


def test_t_brake_search_step():
    # Test case 1 true value:
    # Start at 1m/s, accel for 9s, brake for 4s, accel for 5s
    v0 = 1
    a = 1
    b = -1
    v_max = 20
    x_to_v_max = 49.5
    t_to_v_max = 9
    x_to_intersection = 81.5
    x_crit = 42.5
    t_crit = 18

    # First step, clean, should start by going lower
    tl, tr, t, v, t_b_largest, t_b_smallest = RoadLane._t_brake_search_step(
        0, 16, None, None, v0, a, b, v_max, x_to_v_max, t_to_v_max,
        x_to_intersection, x_crit, t_crit)
    assert tl == 0
    assert tr == t_b_smallest == 8
    assert t_b_largest is None

    # 2nd step, now t_smallest is loaded, should start by going lower
    tl, tr, t, v, t_b_largest, t_b_smallest = RoadLane._t_brake_search_step(
        0, 8, None, 8, v0, a, b, v_max, x_to_v_max, t_to_v_max,
        x_to_intersection, x_crit, t_crit)
    assert t == 13
    assert tl == tr == t_b_smallest == 4
    assert v == 6
    assert t_b_largest is None

    # Test case 2 true value:
    # Start at 0m/s, accel for 4s, v_max for 1s, brake for 1s, accel for 1s
    v0 = 0
    a = 1
    b = -1
    v_max = 4
    x_to_v_max = 8
    t_to_v_max = 4
    x_to_intersection = 15.5
    x_crit = 3.5
    t_crit = 7

    # First step, clean, should start by going higher
    tl, tr, t, v, t_b_largest, t_b_smallest = RoadLane._t_brake_search_step(
        0, 4, None, None, v0, a, b, v_max, x_to_v_max, t_to_v_max,
        x_to_intersection, x_crit, t_crit)
    assert tl == 0
    assert tr == t_b_largest == 2
    assert t_b_smallest is None

    # Second step, clean, should start by going higher
    tl, tr, t, v, t_b_largest, t_b_smallest = RoadLane._t_brake_search_step(
        0, 2, None, None, v0, a, b, v_max, x_to_v_max, t_to_v_max,
        x_to_intersection, x_crit, t_crit)
    assert t == 6
    assert tl == tr == t_b_largest == 1
    assert t_b_smallest is None
    assert v == 3


def timestep_to_seconds(ts: int) -> float:
    return (ts - SHARED.t)*SHARED.SETTINGS.TIMESTEP_LENGTH


def test_soonest_exit(rl: RoadLane, vehicle: AutomatedVehicle,
                      vehicle2: AutomatedVehicle):
    rl.add_vehicle(vehicle)
    SHARED.t += 2
    order = 10**ceil(log10(SHARED.SETTINGS.TIMESTEP_LENGTH))

    # Vehicle hasn't fully entered the intersection
    assert rl.soonest_exit(0) is None

    # Free flow case:
    # 50m from intersection, a=3, v_max=30
    rl.vehicle_progress[vehicle] = VehicleProgress(.95)
    vehicle.velocity = 1
    se = rl.soonest_exit(0)
    assert se is not None
    t_ff_exit = 5.45
    v_ff_exit = vehicle.velocity + SHARED.SETTINGS.min_acceleration*t_ff_exit
    assert timestep_to_seconds(se.t) == approx(t_ff_exit, abs=order)
    assert se.velocity == approx(v_ff_exit, abs=order)

    # Basically free flow case:
    # Same as before but there's another exit in front of them but this vehicle
    # is exiting after and at a slower pace than the last exit.
    se = rl.soonest_exit(0, ScheduledExit(vehicle2, VehicleSection.REAR, 0,
                                          rl.speed_limit))
    assert se is not None
    assert timestep_to_seconds(se.t) == approx(t_ff_exit, abs=order)
    assert se.velocity == approx(v_ff_exit, abs=order)

    # Multi-vehicle free flow case:
    # Same as before but the preceding vehicle is now in the road lane.
    rl.add_vehicle(vehicle2)
    vehicle2.velocity = 1
    rl.vehicle_progress[vehicle] = VehicleProgress()
    rl.vehicle_progress[vehicle2] = VehicleProgress(.95)
    se = rl.soonest_exit(1, ScheduledExit(vehicle, VehicleSection.REAR, 0,
                                          rl.speed_limit))
    assert se is not None
    assert timestep_to_seconds(se.t) == approx(t_ff_exit, abs=order)
    assert se.velocity == approx(v_ff_exit, abs=order)

    # Closer basically free flow case:
    # This vehicle exits after but a little bit faster than the last exit, but
    # not so much that it'll overtake it.
    vehicle.velocity = rl.speed_limit
    rl.vehicle_progress[vehicle] = VehicleProgress(.95)
    se = rl.soonest_exit(0, ScheduledExit(vehicle2, VehicleSection.REAR, 0,
                                          rl.speed_limit-1e-6))
    assert se is not None
    assert timestep_to_seconds(se.t) == approx(50/rl.speed_limit, abs=order)
    assert se.velocity == approx(rl.speed_limit, abs=order)

    # Slowest exit collision case:
    # This exit is too close to the last one that not even a full brake can
    # prevent a collision.
    vehicle.velocity = 2*SHARED.SETTINGS.min_acceleration
    rl.vehicle_progress[vehicle] = VehicleProgress(1-1e-6)
    se = rl.soonest_exit(0, ScheduledExit(vehicle2, VehicleSection.REAR, 1, 0))
    assert se is None

    # Slowest exit case just right:
    # Brake for 2s to meet the last exit exactly.
    SHARED.t = 2*SHARED.SETTINGS.steps_per_second
    vehicle.velocity = -2*SHARED.SETTINGS.min_braking
    rl.vehicle_progress[vehicle] = VehicleProgress(1-5.2/rl.trajectory.length)
    se = rl.soonest_exit(0, ScheduledExit(vehicle2, VehicleSection.REAR,
                                          4*SHARED.SETTINGS.steps_per_second,
                                          0))
    assert se is not None
    assert timestep_to_seconds(se.t) == approx(2, abs=order)
    assert se.velocity == approx(0, abs=order)

    # Search required, reaches v_max before and in intersection exit:
    # True soonest exit: accelerate for 1s, v_max for 1s, brake for 1s, exit,
    # accel for -b/a s, speed limit for 1s. This takes 3s and 87.2m to reach
    # the intersection and extends 1-b/a s and 54.8733m into the intersection.
    # The last exit must leave 2.03647s before this trajectory finishes at
    # 30-2.03647*3 m/s in order to reach v_max at that time and location.
    SHARED.t = 0
    vehicle.velocity = rl.speed_limit - SHARED.SETTINGS.min_acceleration
    rl.vehicle_progress[vehicle] = VehicleProgress(1-87.2/rl.trajectory.length)
    t_p_crit = 2.03647
    t_p_exit = 3-SHARED.SETTINGS.min_braking / \
        SHARED.SETTINGS.min_acceleration+1-t_p_crit
    se = rl.soonest_exit(0, ScheduledExit(
        vehicle2, VehicleSection.REAR,
        ceil(t_p_exit*SHARED.SETTINGS.steps_per_second),
        rl.speed_limit - SHARED.SETTINGS.min_acceleration*t_p_crit))
    assert se is not None
    assert timestep_to_seconds(se.t) == approx(3, abs=order)
    assert se.velocity == approx(rl.speed_limit + SHARED.SETTINGS.min_braking,
                                 abs=order)

    # Search required, does not reach v_max exit:
    # True soonest exit: start at 1m/s, accelerate for 3s, brake for 2s, in the
    # intersection, accelerate for 4s. This takes 5s and 31.3m to reach the
    # intersection and extends 4s and 43.2m into the intersection.
    # The last exit must leave 1.56199s before this trajectory finishes at
    # 25.31403m/s in order to reach v_max at that time and location.
    vehicle.velocity = 1
    rl.vehicle_progress[vehicle] = VehicleProgress(1-31.3/rl.trajectory.length)
    t_p_crit = 1.56199
    t_p_exit = 9-t_p_crit
    se = rl.soonest_exit(0, ScheduledExit(
        vehicle2, VehicleSection.REAR,
        ceil(t_p_exit*SHARED.SETTINGS.steps_per_second),
        rl.speed_limit - SHARED.SETTINGS.min_acceleration*t_p_crit))
    assert se is not None
    assert timestep_to_seconds(se.t) == approx(5, abs=order)
    assert se.velocity == approx(1 + 3*SHARED.SETTINGS.min_acceleration +
                                 2*SHARED.SETTINGS.min_braking, abs=order)


def test_register_latest_exit(rl: RoadLane, vehicle: AutomatedVehicle):
    with raises(ValueError):
        rl.register_latest_scheduled_exit(
            ScheduledExit(vehicle, VehicleSection.CENTER, 0, 0))
    earlier = ScheduledExit(vehicle, VehicleSection.REAR, 0, 0)
    later = ScheduledExit(vehicle, VehicleSection.REAR, 10, 0)
    rl.register_latest_scheduled_exit(earlier)
    assert rl.latest_scheduled_exit is earlier
    rl.register_latest_scheduled_exit(later)
    assert rl.latest_scheduled_exit is later
    rl.register_latest_scheduled_exit(earlier)
    assert rl.latest_scheduled_exit is later


def test_clone(rl: RoadLane, vehicle: AutomatedVehicle,
               vehicle2: AutomatedVehicle):
    rl.add_vehicle(vehicle)
    rl.add_vehicle(vehicle2)
    cl = rl.clone()
    assert len(cl.vehicles) == 0
    assert len(cl.vehicle_progress) == 0
    assert cl.downstream_is_remover
    assert cl.downstream_intersection is None
    assert cl.latest_scheduled_exit is None
