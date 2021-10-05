from typing import Any, Dict, List, Tuple, Type
from math import pi

from aimsim.simulator import Simulator
from aimsim.vehicles import AutomatedVehicle
from aimsim.endpoints.factories import GaussianVehicleFactory
from aimsim.util import Coord
from aimsim.trajectories import BezierTrajectory
from aimsim.intersection.managers import IntersectionManager, StopSignManager
from aimsim.intersection.tilings import SquareTiling
from aimsim.intersection.tilings.tiles import DeterministicTile


class ZipperMergeSim(Simulator):

    def __init__(self, visualize: bool = True, length: float = 100,
                 manager_type: Type[IntersectionManager] = StopSignManager):
        """Create an instance of a double single lane crossing simulator."""

        # Create IO roads
        traj_i_lr = BezierTrajectory(Coord(-length, 12), Coord(0, 12),
                                     [Coord(-length/2, 12)])
        traj_i_up = BezierTrajectory(Coord(12, -length), Coord(12, 0),
                                     [Coord(12, -length/2)])
        side = (2*length**2)**.5/2
        traj_o = BezierTrajectory(Coord(24, 24), Coord(24+side, 24+side),
                                  [Coord(24+side/2, 24+side/2)])

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
        # Form outgoing road spec
        road_specs.append(dict(
            id=2,
            upstream_id=0,
            downstream_id=0,
            trajectory=traj_o,
            num_lanes=1,
            lane_width=4,
            upstream_is_spawner=False,
            downstream_is_remover=True,
            lane_offset_angle=pi/2,
            len_approach_region=traj_o.length*.19,
            len_entrance_region=traj_o.length*.8,
            speed_limit=15
        ))

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

        factory_spec_2nd = factory_spec.copy()
        factory_spec_2nd['destination_probabilities'] = [1]
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
        remover_specs: List[Dict[str, Any]] = [dict(
            id=0,
            road_id=2
        )]

        # Form intersection spec
        intersection_spec: Dict[str, Any] = dict(
            id=0,
            incoming_road_ids=[0, 1],
            outgoing_road_ids=[2],
            connectivity=[(0, 2, True), (1, 2, True)],
            manager_type=manager_type,
            manager_spec=dict(tiling_type=SquareTiling, tiling_spec=dict(
                tile_type=DeterministicTile, misc_spec=dict(tile_width=100)
            )),
            speed_limit=15
        )

        # Form pathfinder hardcode
        lane_il = Coord(0, 12)
        lane_iu = Coord(12, 0)
        lane_o = Coord(24, 24)
        od_pair: Dict[Tuple[Coord, int], List[Coord]] = {
            (lane_il, 0): [lane_o],
            (lane_iu, 0): [lane_o],
        }

        super().__init__(road_specs, [intersection_spec], spawner_specs,
                         remover_specs, od_pair, visualize=visualize)
