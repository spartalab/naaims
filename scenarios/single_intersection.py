from typing import Any, Dict, List, Tuple, Type

from aimsim.simulator import Simulator
from aimsim.vehicles import AutomatedVehicle
from aimsim.endpoints.factories import GaussianVehicleFactory
from aimsim.util import Coord
from aimsim.trajectories import BezierTrajectory
from aimsim.intersection.managers import IntersectionManager, StopSignManager
from aimsim.intersection.tilings import SquareTiling
from aimsim.intersection.tilings.tiles import Tile, DeterministicTile


class SingleIntersectionSim(Simulator):

    def __init__(self, visualize: bool = True, length: float = 100,
                 num_lanes: int = 3, speed_limit: float = 15,
                 turn_ratios: Tuple[float, float, float] = (.1, .8, .1),
                 manager_type: Type[IntersectionManager] = StopSignManager,
                 tile_type: Type[Tile] = DeterministicTile,
                 tile_width: float = 100):
        """Create an instance of a 4-way 3-lane intersection simulator.

        Destination
            0: left
            1: down
            2: right
            3: up
        """

        if sum(turn_ratios) != 1:
            raise ValueError('Turn ratios must sum to 1.')

        if num_lanes not in {1, 2, 3}:
            raise ValueError('Only 1 to 3 lanes supported.')

        # Create IO roads
        traj_i_l = BezierTrajectory(Coord(-length, 10), Coord(0, 10),
                                    [Coord(-length/2, 10)])
        traj_o_l = BezierTrajectory(Coord(0, 22), Coord(-length, 22),
                                    [Coord(-length/2, 22)])
        traj_i_d = BezierTrajectory(Coord(22, -length), Coord(22, 0),
                                    [Coord(22, -length/2)])
        traj_o_d = BezierTrajectory(Coord(10, 0), Coord(10, -length),
                                    [Coord(10, -length/2)])
        traj_i_r = BezierTrajectory(Coord(32+length, 22), Coord(32, 22),
                                    [Coord(32+length/2, 22)])
        traj_o_r = BezierTrajectory(Coord(32, 10), Coord(32+length, 10),
                                    [Coord(32+length/2, 10)])
        traj_i_u = BezierTrajectory(Coord(10, 32+length), Coord(10, 32),
                                    [Coord(10, 32+length/2)])
        traj_o_u = BezierTrajectory(Coord(22, 32), Coord(22, 32+length),
                                    [Coord(22, 32+length/2)])
        traj_i = [traj_i_l, traj_i_d, traj_i_r, traj_i_u]
        traj_o = [traj_o_l, traj_o_d, traj_o_r, traj_o_u]

        road_specs: List[Dict[str, Any]] = []
        # Form incoming road specs
        for i, traj in enumerate(traj_i):
            road_specs.append(dict(
                id=i,
                upstream_id=i,
                downstream_id=0,  # Only one intersection
                trajectory=traj,
                num_lanes=num_lanes,
                lane_width=4,
                upstream_is_spawner=True,
                downstream_is_remover=False,
                lane_offset_angle=0,
                len_approach_region=traj.length*.8,
                len_entrance_region=traj.length*.19,
                speed_limit=speed_limit
            ))
        # Form outgoing road specs
        for i, traj in enumerate(traj_o):
            road_specs.append(dict(
                id=i+4,
                upstream_id=0,  # Only one intersection
                downstream_id=i,
                trajectory=traj,
                num_lanes=num_lanes,
                lane_width=4,
                upstream_is_spawner=False,
                downstream_is_remover=True,
                lane_offset_angle=0,
                len_approach_region=traj.length*.19,
                len_entrance_region=traj.length*.8,
                speed_limit=speed_limit
            ))

        factory_spec_generic: Dict[str, Any] = dict(
            vehicle_type=AutomatedVehicle,
            num_destinations=4,
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

        spawner_specs: List[Dict[str, Any]] = []
        factory_specs: List[Dict[str, Any]] = []

        # Form spawner and factory specs
        for i in range(4):
            factory_spec = factory_spec_generic.copy()
            destination_ps = list(turn_ratios)
            destination_ps.insert(i, 0)
            factory_spec['destination_probabilities'] = destination_ps
            factory_spec['source_node_id'] = i
            factory_specs.append(factory_spec)
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
        for i in range(4):
            remover_specs.append(dict(
                id=i,
                road_id=4+i
            ))

        # Form intersection spec
        intersection_spec: Dict[str, Any] = dict(
            id=0,
            incoming_road_ids=[0, 1, 2, 3],
            outgoing_road_ids=[4, 5, 6, 7],
            connectivity=[(0, 6, True),  # left to right
                          (1, 7, True),  # down to up
                          (2, 4, True),  # right to left
                          (3, 5, True),  # up to down
                          (0, 5, False),  # left to down
                          (0, 7, False),  # left to up
                          (1, 4, False),  # down to left
                          (1, 6, False),  # down to right
                          (2, 7, False),  # right to up
                          (2, 5, False),  # right to down
                          (3, 4, False),  # up to left
                          (3, 6, False)  # up to right
                          ],
            manager_type=manager_type,
            manager_spec=dict(tiling_type=SquareTiling, tiling_spec=dict(
                tile_type=tile_type, misc_spec=dict(tile_width=tile_width))),
            speed_limit=speed_limit
        )

        # Form pathfinder hardcode
        od_pair: Dict[Tuple[Coord, int], List[Coord]] = {}
        if num_lanes == 3:
            # Through   left (0) to right (2)
            od_pair[(Coord(0.0, 10.0), 2)] = [Coord(32.0, 10.0)]
            od_pair[(Coord(0.0, 14.0), 2)] = [Coord(32.0, 14.0)]
            od_pair[(Coord(0.0, 6.0), 2)] = [Coord(32.0, 6.0)]

            # Through   down (1) to up (3)
            od_pair[(Coord(18.0, -2.4492935982947064e-16), 3)
                    ] = [Coord(18.0, 32.0)]
            od_pair[(Coord(22.0, 0.0), 3)] = [Coord(22.0, 32.0)]
            od_pair[(Coord(26.0, 2.4492935982947064e-16), 3)
                    ] = [Coord(26.0, 32.0)]

            # Through   right (2) to left (0)
            od_pair[(Coord(32.0, 18.0), 0)] = [
                Coord(4.898587196589413e-16, 18.0)]
            od_pair[(Coord(32.0, 22.0), 0)] = [Coord(0.0, 22.0)]
            od_pair[(Coord(32.0, 26.0), 0)] = [
                Coord(-4.898587196589413e-16, 26.0)]

            # Through   up (3) to down (1)
            od_pair[(Coord(6.0, 32.0), 1)] = [
                Coord(6.0, -7.347880794884119e-16)]
            od_pair[(Coord(14.0, 32.0), 1)] = [
                Coord(14.0, 7.347880794884119e-16)]
            od_pair[(Coord(10.0, 32.0), 1)] = [Coord(10.0, 0.0)]

            # Right     left (0) to down (1)
            od_pair[(Coord(0.0, 6.0), 1)] = [
                Coord(6.0, -7.347880794884119e-16)]

            # Left      left (0) to up (3)
            od_pair[(Coord(0.0, 14.0), 3)] = [Coord(18.0, 32.0)]

            # Left      down (1) to left (0)
            od_pair[(Coord(18.0, -2.4492935982947064e-16), 0)
                    ] = [Coord(18.0, 32.0)]

            # Right     down (1) to right (2)
            od_pair[(Coord(26.0, 2.4492935982947064e-16), 2)
                    ] = [Coord(32.0, 6.0)]

            # Right     right (2) to up (3)
            od_pair[(Coord(32.0, 26.0), 3)] = [Coord(26.0, 32.0)]

            # Left      right (2) to down (1)
            od_pair[(Coord(32.0, 18.0), 1)] = [
                Coord(14.0, 7.347880794884119e-16)]

            # Right     up (3) to left (0)
            od_pair[(Coord(6.0, 32.0), 0)] = [
                Coord(-4.898587196589413e-16, 26.0)]

            # Left      up (3) to right (2)
            od_pair[(Coord(14.0, 32.0), 2)] = [Coord(32.0, 14.0)]

        else:
            raise NotImplementedError("TODO: Hardcode other lane pathfinders.")

        super().__init__(road_specs, [intersection_spec], spawner_specs,
                         remover_specs, od_pair, visualize=visualize)
