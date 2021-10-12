from typing import Any, Dict, List, Tuple

from naaims.simulator import Simulator
from naaims.vehicles import AutomatedVehicle
from naaims.endpoints.factories import GaussianVehicleFactory
from naaims.util import Coord
from naaims.trajectories import BezierTrajectory


class ThreeLaneRoadSim(Simulator):

    def __init__(self, visualize: bool = True, length: float = 100):

        trajectory = BezierTrajectory(Coord(0, 0), Coord(length, 0),
                                      [Coord(length/2, 0)])
        road_spec: Dict[str, Any] = dict(
            id=0,
            upstream_id=0,
            downstream_id=0,
            trajectory=trajectory,
            num_lanes=3,
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
            vpm=30,
            factory_selection_probabilities=[1],
            factory_types=[GaussianVehicleFactory],
            factory_specs=[factory_spec]
        )
        remover_spec: Dict[str, Any] = dict(
            id=0,
            road_id=0
        )

        top_lane_end_coord = Coord(length, 4)
        bottom_lane_end_coord = Coord(length, -4)

        od_pair: Dict[Tuple[Coord, int], List[Coord]] = {
            (top_lane_end_coord, 0): [top_lane_end_coord, trajectory.end_coord,
                                      bottom_lane_end_coord],
            (trajectory.end_coord, 0): [top_lane_end_coord,
                                        trajectory.end_coord,
                                        bottom_lane_end_coord],
            (bottom_lane_end_coord, 0): [top_lane_end_coord,
                                         trajectory.end_coord,
                                         bottom_lane_end_coord]
        }

        super().__init__([road_spec], [], [spawner_spec], [remover_spec],
                         od_pair, visualize=visualize)
