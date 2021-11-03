from os import remove

from pytest import raises

import naaims.shared as SHARED
from naaims.simulator import Simulator
from naaims.intersection.managers import FCFSManager
from naaims.intersection.movement import OneDrawStochasticModel
from scenarios.road_only import OneLaneRoadSim, TwoLaneRoadSim, \
    ThreeLaneRoadSim
from scenarios import SingleLaneXNoTurnsSim, Symmetrical4Way


def test_crash_calc():
    crash_mev = .05
    crash_v = crash_mev / 1_000_000
    vpm = 10
    ts_per_s = 60
    v_per_ts = vpm/60/ts_per_s
    crash_ts = v_per_ts * crash_v
    Simulator.crash_probability_tolerance(vpm, crash_mev, ts_per_s) == crash_ts


def test_init_blank(clean_shared: None):
    with raises(ValueError):
        Simulator([], [], [], [], acceptable_crash_mev=-.1)
    Simulator([], [], [], [], acceptable_crash_mev=1.1)


def test_one_lane_road_simulator(clean_shared: None):
    sim = OneLaneRoadSim(visualize=False, length=30)
    for _ in range(30*60):
        sim.step()


def test_two_lane_road_simulator(clean_shared: None):
    sim = TwoLaneRoadSim(visualize=False, length=30)
    for _ in range(30*60):
        sim.step()


def test_three_lane_road_simulator(clean_shared: None):
    sim = ThreeLaneRoadSim(visualize=False, length=30)
    for _ in range(30*60):
        sim.step()


def test_single_lane_x_no_turns_simulator(clean_shared: None):
    sim = SingleLaneXNoTurnsSim(visualize=False, length=30)
    for _ in range(30*60):
        sim.step()


def test_single_lane_x_no_turns_simulator_fcfs(clean_shared: None):
    sim = SingleLaneXNoTurnsSim(length=30, manager_type=FCFSManager)
    for _ in range(30*60):
        sim.step()


def test_single_intersection_fcfs(clean_shared: None) -> Simulator:
    sim = Symmetrical4Way(length=30, manager_type=FCFSManager)
    for _ in range(30*60):
        sim.step()
    return sim


def test_crash_init(clean_shared: None):
    sim = Symmetrical4Way(
        length=30, manager_type=FCFSManager, acceptable_crash_mev=12,
        movement_model='one draw')
    reject = Simulator.crash_probability_tolerance(
        4*10, 12, SHARED.SETTINGS.steps_per_second)
    assert isinstance(
        sim.intersections[0].lanes[0].movement_model, OneDrawStochasticModel)
    assert sim.intersections[0].lanes[0].movement_model.threshold \
        == reject/64


def test_visualizer(clean_shared: None):
    sim = SingleLaneXNoTurnsSim(length=30, manager_type=FCFSManager)
    anim = sim.animate(max_timestep=1*60)


def test_save_log(clean_shared: None):
    sim = Symmetrical4Way(clean_shared)
    sim.save_log('output/logs/test.csv')
    remove('output/logs/test.csv')
