from math import ceil

from pytest import approx, raises

from naaims.intersection import IntersectionLane
from naaims.road import RoadLane
from naaims.util import Coord, VehicleSection, VehicleTransfer
from naaims.trajectories import BezierTrajectory
from naaims.vehicles import Vehicle
from naaims.lane import LateralDeviation, ScheduledExit, VehicleProgress
from naaims.intersection.movement import OneDrawStochasticModel


def test_init(load_shared: None):
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
    il = IntersectionLane(rl_start, rl_end, speed_limit,
                          OneDrawStochasticModel)
    assert hash(il.trajectory) == hash(
        BezierTrajectory(Coord(0, 0), Coord(1, 1), [Coord(0, 1)]))
    assert type(il.movement_model) is OneDrawStochasticModel
    assert il.movement_model.trajectory is il.trajectory


def test_add_remove(il: IntersectionLane, vehicle: Vehicle, vehicle2: Vehicle):
    il.add_vehicle(vehicle)
    assert len(il.vehicles) == 1
    assert il.vehicles[0] is vehicle
    assert len(il.vehicle_progress) == 1
    assert il.vehicle_progress[vehicle] == VehicleProgress()
    assert len(il.lateral_deviation) == 1
    assert il.lateral_deviation[vehicle] == 0
    il.add_vehicle(vehicle2)
    assert len(il.vehicles) == 2
    assert il.vehicles[1] is vehicle2
    assert len(il.vehicle_progress) == 2
    assert il.vehicle_progress[vehicle] == VehicleProgress()
    assert len(il.lateral_deviation) == 2
    assert il.lateral_deviation[vehicle] == 0
    il.remove_vehicle(vehicle)
    assert len(il.vehicles) == 1
    assert il.vehicles[0] == vehicle2
    assert len(il.vehicle_progress) == 1
    assert len(il.lateral_deviation) == 1
    il.remove_vehicle(vehicle2)
    assert len(il.vehicles) == 0
    assert len(il.vehicle_progress) == 0
    assert len(il.lateral_deviation) == 0


def test_rear_exit_road(il: IntersectionLane, vehicle: Vehicle):
    v_max, a = 15, 3

    # Stays below speed limit
    t0, v0 = 10, 2.1
    stopped_exit = il.rear_exit(ScheduledExit(vehicle, VehicleSection.FRONT,
                                              t0, v0))
    assert stopped_exit.vehicle is vehicle
    assert stopped_exit.section is VehicleSection.REAR
    assert stopped_exit.velocity == approx(
        (v0**2 + 2*a*vehicle.length*1.2)**.5)
    assert stopped_exit.t == ceil((stopped_exit.velocity-v0)/a) + t0

    # Reaches speed limit during transition
    t0, v0 = 4, 14.9
    stopped_exit = il.rear_exit(ScheduledExit(vehicle, VehicleSection.FRONT,
                                              t0, v0))
    assert stopped_exit.vehicle is vehicle
    assert stopped_exit.section is VehicleSection.REAR
    assert stopped_exit.velocity == v_max
    t_accel = (15-v0)/a
    t_v_max = (vehicle.length*1.2 - (v0*t_accel + .5*a*t_accel**2))/v_max
    assert stopped_exit.t == ceil(t_accel + t_v_max) + t0

    # At speed limit before transition
    t0, v0 = 0, v_max
    stopped_exit = il.rear_exit(ScheduledExit(vehicle, VehicleSection.FRONT,
                                              t0, v0))
    assert stopped_exit.vehicle is vehicle
    assert stopped_exit.section is VehicleSection.REAR
    assert stopped_exit.velocity == v_max
    assert stopped_exit.t == ceil(vehicle.length*1.2/v0) + t0


def test_rear_exit_intersection(il: IntersectionLane,
                                vehicle: Vehicle):
    v_max, a = 15, 3

    # TODO: Stays below speed limit

    # Reaches speed limit during transition
    t0, v0 = 4, 14.9
    stopped_exit = il.rear_exit(ScheduledExit(vehicle, VehicleSection.FRONT,
                                              t0, v0), True)
    assert stopped_exit.vehicle is vehicle
    assert stopped_exit.section is VehicleSection.REAR
    assert stopped_exit.velocity == v_max
    t_accel = (15-v0)/a
    t_v_max = (vehicle.length*1.2*2 + il.trajectory.length -
               (v0*t_accel + .5*a*t_accel**2))/v_max
    assert stopped_exit.t == ceil(t_accel + t_v_max) + t0

    # At speed limit before transition
    t0, v0 = 0, v_max
    stopped_exit = il.rear_exit(ScheduledExit(vehicle, VehicleSection.FRONT,
                                              t0, v0), True)
    assert stopped_exit.vehicle is vehicle
    assert stopped_exit.section is VehicleSection.REAR
    assert stopped_exit.velocity == v_max
    assert stopped_exit.t == ceil((vehicle.length*1.2*2 + il.trajectory.length)
                                  / v0) + t0


def test_step(il: IntersectionLane, vehicle: Vehicle):
    with raises(ValueError):
        il.step_vehicles({vehicle: LateralDeviation(il, 0.1)})


def test_clone(il: IntersectionLane, vehicle: Vehicle, vehicle2: Vehicle,
               il_stochastic: IntersectionLane):
    il.add_vehicle(vehicle)
    il.add_vehicle(vehicle2)
    cl = il.clone()
    assert len(cl.vehicles) == 0
    assert len(cl.vehicle_progress) == 0
    assert len(cl.lateral_deviation) == 0

    il_stochastic.add_vehicle(vehicle)
    cl = il_stochastic.clone()
    assert type(cl.movement_model) is OneDrawStochasticModel
    assert cl.movement_model.disable_stochasticity


def test_section_enter_exit(il: IntersectionLane, vehicle: Vehicle,
                            vehicle2: Vehicle,
                            il_stochastic: IntersectionLane):
    assert vehicle not in il.lateral_deviation
    assert vehicle2 not in il.lateral_deviation
    il.add_vehicle(vehicle)
    il_stochastic.add_vehicle(vehicle)
    assert il.lateral_deviation[vehicle] == 0
    assert il_stochastic.lateral_deviation[vehicle] == 0

    assert type(il_stochastic.movement_model) is OneDrawStochasticModel
    t1f = VehicleTransfer(vehicle, VehicleSection.FRONT, 0, Coord(0, 0))
    il.enter_vehicle_section(t1f)
    il_stochastic.enter_vehicle_section(t1f)
    assert vehicle in il_stochastic.movement_model.p_cutoff
    assert vehicle not in il_stochastic.movement_model.max_lateral_deviation
    t1c = VehicleTransfer(
        vehicle, VehicleSection.CENTER, 0, Coord(0, 0))
    il.enter_vehicle_section(t1c)
    il_stochastic.enter_vehicle_section(t1c)
    assert vehicle in il_stochastic.movement_model.p_cutoff

    t2f = VehicleTransfer(vehicle2, VehicleSection.FRONT, 0, Coord(0, 0))
    t2c = VehicleTransfer(vehicle2, VehicleSection.CENTER, 0, Coord(0, 0))
    il.enter_vehicle_section(t2f)
    il.enter_vehicle_section(t2c)
    il_stochastic.enter_vehicle_section(t2f)
    il_stochastic.enter_vehicle_section(t2c)

    il.remove_vehicle(vehicle)
    assert vehicle not in il.lateral_deviation
    assert vehicle2 in il.lateral_deviation
    il_stochastic.remove_vehicle(vehicle)
    assert vehicle not in il_stochastic.lateral_deviation
    assert vehicle2 in il_stochastic.lateral_deviation
    assert vehicle not in il_stochastic.movement_model.p_cutoff
    assert vehicle2 in il_stochastic.movement_model.p_cutoff
    assert vehicle not in il_stochastic.movement_model.max_lateral_deviation


def test_lateral(il: IntersectionLane, vehicle: Vehicle,
                 il_stochastic: IntersectionLane):
    lat = il.movement_model.fetch_lateral_deviation(vehicle, .5)
    assert il.lateral_deviation_for(vehicle, .5) == lat
    assert il.lateral_deviation[vehicle] == lat == 0

    t1c = VehicleTransfer(
        vehicle, VehicleSection.CENTER, 0, Coord(0, 0))
    il_stochastic.add_vehicle(vehicle)
    il_stochastic.enter_vehicle_section(t1c)
    lat = il_stochastic.movement_model.fetch_lateral_deviation(vehicle, .5)
    assert il_stochastic.lateral_deviation_for(vehicle, .5) == lat
    assert il_stochastic.lateral_deviation[vehicle] == lat


def test_accel_update(il: IntersectionLane, vehicle: Vehicle,
                      il_stochastic: IntersectionLane, h_vehicle: Vehicle):

    assert il.accel_update(vehicle, VehicleSection.FRONT, .5, None
                           ) == il.accel_update_uncontested(vehicle, .5)
    il_stochastic.add_vehicle(vehicle)
    il_stochastic.enter_vehicle_section(VehicleTransfer(
        vehicle, VehicleSection.CENTER, 0, Coord(0, 0)))
    assert il_stochastic.accel_update(
        vehicle, VehicleSection.FRONT, .5, None
    ) == il.accel_update_uncontested(vehicle, .5)

    il_stochastic.add_vehicle(h_vehicle)
    il_stochastic.enter_vehicle_section(VehicleTransfer(
        h_vehicle, VehicleSection.FRONT, 0, Coord(0, 0)))
    assert il_stochastic.accel_update(
        h_vehicle, VehicleSection.FRONT, .5, None
    ) < il.accel_update_uncontested(h_vehicle, .5)

    vehicle.velocity = 10
    assert il.accel_update(vehicle, VehicleSection.FRONT, .5, h_vehicle) == 3
    vehicle.trailing = True
    il.vehicle_progress[h_vehicle] = VehicleProgress(.53, .52, .51)
    assert il.accel_update(vehicle, VehicleSection.FRONT, .5, h_vehicle) == \
        -2.6
