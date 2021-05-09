from aimsim.simulator import Simulator
from scenarios.road_only import OneLaneRoadSim, TwoLaneRoadSim, \
    ThreeLaneRoadSim


def test_blank_simulator(clean_config: None):
    Simulator([], [], [], [], {})


def test_one_lane_road_simulator(clean_config: None):
    sim = OneLaneRoadSim(visualize=False, length=10)
    for _ in range(10*60):
        sim.step()


def test_two_lane_road_simulator(clean_config: None):
    sim = TwoLaneRoadSim(visualize=False, length=10)
    for _ in range(60*60):
        sim.step()


def test_three_lane_road_simulator(clean_config: None):
    sim = ThreeLaneRoadSim(visualize=False, length=10)
    for _ in range(60*60):
        sim.step()
