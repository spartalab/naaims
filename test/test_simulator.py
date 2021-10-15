from os import remove

from naaims.simulator import Simulator
from naaims.intersection.managers import FCFSManager
from scenarios.road_only import OneLaneRoadSim, TwoLaneRoadSim, \
    ThreeLaneRoadSim
from scenarios import SingleLaneXNoTurnsSim, SingleIntersectionSim


def test_blank_simulator(clean_shared: None):
    Simulator([], [], [], [], {})


def test_one_lane_road_simulator(clean_shared: None):
    sim = OneLaneRoadSim(visualize=False, length=30)
    for _ in range(60*60):
        sim.step()


def test_two_lane_road_simulator(clean_shared: None):
    sim = TwoLaneRoadSim(visualize=False, length=30)
    for _ in range(60*60):
        sim.step()


def test_three_lane_road_simulator(clean_shared: None):
    sim = ThreeLaneRoadSim(visualize=False, length=30)
    for _ in range(60*60):
        sim.step()


def test_single_lane_x_no_turns_simulator(clean_shared: None):
    sim = SingleLaneXNoTurnsSim(visualize=False, length=30)
    for _ in range(60*60):
        sim.step()


def test_single_lane_x_no_turns_simulator_fcfs(clean_shared: None):
    sim = SingleLaneXNoTurnsSim(length=30, manager_type=FCFSManager)
    for _ in range(60*60):
        sim.step()


def single_intersection_fcfs(clean_shared: None) -> Simulator:
    sim = SingleIntersectionSim(length=30, manager_type=FCFSManager)
    for _ in range(60*60):
        sim.step()
    return sim


def test_visualizer(clean_shared: None):
    sim = SingleLaneXNoTurnsSim(length=30, manager_type=FCFSManager)
    anim = sim.animate(max_timestep=1*60)


def test_save_log(clean_shared: None):
    sim = single_intersection_fcfs(clean_shared)
    sim.save_log('output/logs/test.csv')
    remove('output/logs/test.csv')
