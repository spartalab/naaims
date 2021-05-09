from typing import Any, Dict, List, Tuple

from IPython.display import HTML

from aimsim.simulator import Simulator
from aimsim.vehicles import AutomatedVehicle
from aimsim.endpoints.factories import GaussianVehicleFactory
from aimsim.util import Coord
from aimsim.trajectories import BezierTrajectory


def test_blank_simulator(clean_config: None):
    Simulator([], [], [], [], {})


def make_one_lane_simulator(visualize: bool = False):
    trajectory = BezierTrajectory(Coord(0, 0), Coord(10, 0),
                                  [Coord(5, 0)])
    road_spec: Dict[str, Any] = dict(
        id=0,
        upstream_id=0,
        downstream_id=0,
        trajectory=trajectory,
        num_lanes=1,
        lane_width=4,
        upstream_is_spawner=True,
        downstream_is_remover=True,
        lane_offset_angle=0,
        len_approach_region=0,
        len_entrance_region=trajectory.length,
        speed_limit=15
    )
    factory_spec: Dict[str, Any] = dict(
        vehicle_type=AutomatedVehicle,
        num_destinations=1,
        destination_probabilities=[1],
        source_node_id=None,
        max_accel_mn=3,
        max_accel_sd=0,
        max_braking_mn=-3.4,
        max_braking_sd=0,
        length_mn=4.5,
        length_sd=0,
        width_mn=3,
        width_sd=0,
        throttle_score_mn=0,
        throttle_score_sd=0,
        tracking_score_mn=0,
        tracking_score_sd=0,
        vot_mn=0,
        vot_sd=0
    )
    spawner_spec: Dict[str, Any] = dict(
        id=0,
        road_id=0,
        vpm=10,
        factory_selection_probabilities=[1],
        factory_types=[GaussianVehicleFactory],
        factory_specs=[factory_spec]
    )
    remover_spec: Dict[str, Any] = dict(
        id=0,
        road_id=0
    )
    od_pair: Dict[Tuple[Coord, int], List[Coord]] = {
        (trajectory.end_coord, 0): [trajectory.end_coord]
    }
    return Simulator([road_spec], [], [spawner_spec], [remover_spec], od_pair,
                     visualize=visualize)


def test_one_lane_road_simulator(clean_config: None):
    sim = make_one_lane_simulator(visualize=False)
    for _ in range(10*60):
        sim.step()
    print(len(sim.fetch_log()), len(sim.vehicles_in_scope))
    print(len(sim.fetch_log()) + len(sim.vehicles_in_scope))


def test_one_lane_visualization(clean_config: None):
    sim = make_one_lane_simulator(visualize=True)
    HTML(sim.animate(max_timestep=60).to_html5_video())
