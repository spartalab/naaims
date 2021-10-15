from math import ceil

from pytest import approx

from naaims.intersection import IntersectionLane
from naaims.road import RoadLane
from naaims.util import Coord, VehicleSection
from naaims.trajectories import BezierTrajectory
from naaims.vehicles import AutomatedVehicle
from naaims.lane import ScheduledExit, VehicleProgress


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
    il = IntersectionLane(rl_start, rl_end, speed_limit)
    assert hash(il.trajectory) == hash(
        BezierTrajectory(Coord(0, 0), Coord(1, 1), [Coord(0, 1)]))


def test_add_remove(il: IntersectionLane, vehicle: AutomatedVehicle,
                    vehicle2: AutomatedVehicle):
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


def test_lateral_deviation(il: IntersectionLane, vehicle: AutomatedVehicle):
    il.add_vehicle(vehicle)
    il.lateral_deviation[vehicle] = 10
    assert il.lateral_deviation_for(vehicle, 5) == 0


def test_rear_exit_road(il: IntersectionLane, vehicle: AutomatedVehicle):
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
                                vehicle: AutomatedVehicle):
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


def test_clone(il: IntersectionLane, vehicle: AutomatedVehicle,
               vehicle2: AutomatedVehicle):
    il.add_vehicle(vehicle)
    il.add_vehicle(vehicle2)
    cl = il.clone()
    assert len(cl.vehicles) == 0
    assert len(cl.vehicle_progress) == 0
    assert len(cl.lateral_deviation) == 0
