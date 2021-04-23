from aimsim.intersection import IntersectionLane
from aimsim.road import RoadLane
from aimsim.util import Coord
from aimsim.trajectories import BezierTrajectory
from aimsim.vehicles import AutomatedVehicle
from aimsim.lane import VehicleProgress


def test_init(read_config: None):
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


def test_clone(il: IntersectionLane, vehicle: AutomatedVehicle,
               vehicle2: AutomatedVehicle):
    il.add_vehicle(vehicle)
    il.add_vehicle(vehicle2)
    cl = il.clone()
    assert len(cl.vehicles) == 0
    assert len(cl.vehicle_progress) == 0
    assert len(cl.lateral_deviation) == 0
