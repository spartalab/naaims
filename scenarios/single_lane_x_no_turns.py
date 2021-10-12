from typing import Any, Dict, List, Tuple, Type

from naaims.simulator import Simulator
from naaims.vehicles import AutomatedVehicle
from naaims.endpoints.factories import GaussianVehicleFactory
from naaims.util import Coord
from naaims.trajectories import BezierTrajectory
from naaims.intersection.managers import IntersectionManager, StopSignManager
from naaims.intersection.tilings import SquareTiling
from naaims.intersection.tilings.tiles import DeterministicTile


class SingleLaneXNoTurnsSim(Simulator):

    def __init__(self, visualize: bool = True, length: float = 100,
                 manager_type: Type[IntersectionManager] = StopSignManager):
        """Create an instance of a double single lane crossing simulator."""

        # Create IO roads
        traj_i_lr = BezierTrajectory(Coord(-length, 12), Coord(0, 12),
                                     [Coord(-length/2, 12)])
        traj_i_up = BezierTrajectory(Coord(12, -length), Coord(12, 0),
                                     [Coord(12, -length/2)])
        traj_o_lr = BezierTrajectory(Coord(24, 12), Coord(24+length, 12),
                                     [Coord(24+length/2, 12)])
        traj_o_up = BezierTrajectory(Coord(12, 24), Coord(12, 24+length),
                                     [Coord(12, 24+length/2)])

        road_specs: List[Dict[str, Any]] = []
        # Form incoming road specs
        for i, traj in enumerate([traj_i_lr, traj_i_up]):
            road_specs.append(dict(
                id=i,
                upstream_id=i,
                downstream_id=0,
                trajectory=traj,
                num_lanes=1,
                lane_width=4,
                upstream_is_spawner=True,
                downstream_is_remover=False,
                lane_offset_angle=0,
                len_approach_region=traj.length*.8,
                len_entrance_region=traj.length*.19,
                speed_limit=15
            ))
        # Form outgoing road specs
        for i, traj in enumerate([traj_o_lr, traj_o_up]):
            road_specs.append(dict(
                id=i+2,
                upstream_id=0,
                downstream_id=i,
                trajectory=traj,
                num_lanes=1,
                lane_width=4,
                upstream_is_spawner=False,
                downstream_is_remover=True,
                lane_offset_angle=0,
                len_approach_region=traj.length*.19,
                len_entrance_region=traj.length*.8,
                speed_limit=15
            ))

        factory_spec: Dict[str, Any] = dict(
            vehicle_type=AutomatedVehicle,
            num_destinations=2,
            destination_probabilities=[1, 0],
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

        factory_spec_2nd = factory_spec.copy()
        factory_spec_2nd['destination_probabilities'] = [0, 1]
        factory_specs = [factory_spec, factory_spec_2nd]

        # Form spawner specs
        spawner_specs: List[Dict[str, Any]] = []
        for i in range(2):
            spawner_specs.append(dict(
                id=i,
                road_id=i,
                vpm=10,
                factory_selection_probabilities=[1],
                factory_types=[GaussianVehicleFactory],
                factory_specs=[factory_specs[i]]
            ))

        # Form remover specs
        remover_specs: List[Dict[str, Any]] = []
        for i in range(2):
            remover_specs.append(dict(
                id=i,
                road_id=2+i
            ))

        # Form intersection spec
        intersection_spec: Dict[str, Any] = dict(
            id=0,
            incoming_road_ids=[0, 1],
            outgoing_road_ids=[2, 3],
            connectivity=[(0, 2, True), (1, 3, True)],
            manager_type=manager_type,
            manager_spec=dict(tiling_type=SquareTiling, tiling_spec=dict(
                tile_type=DeterministicTile, misc_spec=dict(tile_width=100)
            )),
            speed_limit=15
        )

        # Form pathfinder hardcode
        lane_il = Coord(0, 12)
        lane_iu = Coord(12, 0)
        lane_or = Coord(24, 12)
        lane_ou = Coord(12, 24)
        od_pair: Dict[Tuple[Coord, int], List[Coord]] = {
            (lane_il, 0): [lane_or],
            (lane_iu, 1): [lane_ou],
        }

        super().__init__(road_specs, [intersection_spec], spawner_specs,
                         remover_specs, od_pair, visualize=visualize)
