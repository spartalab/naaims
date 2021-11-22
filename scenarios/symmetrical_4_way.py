from typing import Any, Dict, List, Optional, Tuple, Type, DefaultDict
from math import ceil

from naaims.simulator import Simulator
from naaims.vehicles import AutomatedVehicle, HumanGuidedVehicle
from naaims.endpoints.factories import UniformVehicleFactory
from naaims.util import Coord
from naaims.trajectories import BezierTrajectory
from naaims.intersection.managers import IntersectionManager, StopSignManager
from naaims.intersection.tilings import SquareTiling
from naaims.intersection.tilings.tiles import Tile, DeterministicTile


class Symmetrical4Way(Simulator):

    def __init__(self, visualize: bool = False, visualize_tiles: bool = False,
                 length: float = 100,
                 num_lanes: int = 3, speed_limit: float = 15,
                 turn_ratios: Tuple[float, float, float] = (.1, .8, .1),
                 manager_type: Type[IntersectionManager] = StopSignManager,
                 tile_type: Type[Tile] = DeterministicTile,
                 tile_width: float = 4, vpm: float = 10,
                 av_percentage: float = 1.,
                 movement_model: str = 'deterministic',
                 acceptable_crash_mev: float = 0.,
                 steps_per_second: int = 60,
                 hgv_throttle_mn: float = 0.0752,
                 hgv_throttle_sd: float = 0.1402,
                 hgv_tracking_mn: float = -0.0888,
                 hgv_tracking_sd: float = 0.0631,
                 vot_mn: float = .5,
                 vot_range: float = 1.,
                 multiple_sequence_none: Optional[bool] = None,
                 mechanism: str = 'first',
                 predetermined_spawn_specs: List[Dict[str, Any]] = []):
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

        if (av_percentage < 0) or (av_percentage > 1):
            raise ValueError('AV probability must be between 0 and 1')

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

        factory_spec_generic_av: Dict[str, Any] = dict(
            vehicle_type=AutomatedVehicle,
            num_destinations=4,
            source_node_id=None,
            max_accel_mn=3,
            max_accel_range=0,
            max_braking_mn=-3.4,
            max_braking_range=0,
            length_mn=4.5,
            length_range=0,
            width_mn=3,
            width_range=0,
            throttle_mn_mn=0,
            throttle_mn_range=0,
            throttle_sd_mn=0,
            throttle_sd_range=0,
            tracking_mn_mn=0,
            tracking_mn_range=0,
            tracking_sd_mn=0,
            tracking_sd_range=0,
            vot_mn=vot_mn,
            vot_range=vot_range
        )
        factory_spec_generic_hgv = factory_spec_generic_av.copy()
        factory_spec_generic_hgv['vehicle_type'] = HumanGuidedVehicle
        factory_spec_generic_hgv['throttle_mn_mn'] = hgv_throttle_mn
        factory_spec_generic_hgv['throttle_sd_mn'] = hgv_throttle_sd
        factory_spec_generic_hgv['tracking_mn_mn'] = hgv_tracking_mn
        factory_spec_generic_hgv['tracking_sd_mn'] = hgv_tracking_sd

        spawner_specs: List[Dict[str, Any]] = []

        # Create fixed vehicle spawns and split them according to the spawner
        # they're going to come from.
        predetermined_spawns_for: Dict[int, List[Dict[str, Any]]] = \
            DefaultDict(lambda: [])
        for vehicle_spec in predetermined_spawn_specs:
            predetermined_spawns_for[vehicle_spec['origin']].append(
                vehicle_spec)

        # Form spawner and factory specs
        for i in range(4):
            factory_spec_av = factory_spec_generic_av.copy()
            factory_spec_hgv = factory_spec_generic_hgv.copy()
            factory_specs = [factory_spec_av, factory_spec_hgv]
            turn_ratio_list = list(turn_ratios)
            destination_ps: List[float]
            if i == 0:
                destination_ps = [0.] + turn_ratio_list
            else:
                destination_ps = turn_ratio_list[len(turn_ratio_list)-i:] + \
                    [0] + turn_ratio_list[:len(turn_ratio_list)-i]
            for factory_spec in factory_specs:
                factory_spec['destination_probabilities'] = destination_ps
                factory_spec['source_node_id'] = i
            spawner_specs.append(dict(
                id=i,
                road_id=i,
                vpm=vpm,
                factory_selection_probabilities=[av_percentage,
                                                 1-av_percentage],
                factory_types=[UniformVehicleFactory, UniformVehicleFactory],
                factory_specs=factory_specs,
                predetermined_spawns=predetermined_spawns_for[i]
            ))
            del predetermined_spawns_for[i]

        if len(predetermined_spawns_for) > 0:
            raise ValueError("Predetermined spawns provided for nonexistent "
                             "spawner ID.")

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
            manager_spec=dict(
                tiling_type=SquareTiling,
                tiling_spec=dict(
                    tile_type=tile_type,
                    misc_spec=dict(tile_width=tile_width),
                    timeout=True
                ),
                misc_spec=dict(
                    multiple=(multiple_sequence_none is True),
                    sequence=(multiple_sequence_none is False),
                    mechanism=mechanism
                )),
            speed_limit=speed_limit,
            movement_model=movement_model
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
                    ] = [Coord(4.898587196589413e-16, 18.0)]

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

            # Calculate the vehicles per minute for each approaching lane
            vpm_through = turn_ratios[1] * vpm
            vpm_left = turn_ratios[0] * vpm
            vpm_left_total = vpm_left + vpm_through
            vpm_right = turn_ratios[2] * vpm
            vpm_right_total = vpm_right + vpm_through
            intersection_spec['manager_spec']['misc_spec']['vpm_mean'] = {
                # Left and through
                Coord(0.0, 14.0): vpm_left_total,
                Coord(18.0, -2.4492935982947064e-16): vpm_left_total,
                Coord(32.0, 18.0): vpm_left_total,
                Coord(14.0, 32.0): vpm_left_total,

                # Right and through
                Coord(0.0, 6.0): vpm_right_total,
                Coord(26.0, 2.4492935982947064e-16): vpm_right_total,
                Coord(32.0, 26.0): vpm_right_total,
                Coord(6.0, 32.0): vpm_right_total,

                # Through only
                Coord(0.0, 10.0): vpm_through,
                Coord(22.0, 0.0): vpm_through,
                Coord(32.0, 22.0): vpm_through,
                Coord(10.0, 32.0): vpm_through
            }
            intersection_spec['manager_spec']['misc_spec']['vot_mean'] = {
                coord: vot_mn for coord in
                intersection_spec['manager_spec']['misc_spec']['vpm_mean']}

            # Define the traffic plan for a phasing cycle that allows for
            # through and left turns for each approach in sequence.

            # vehicle length (veh/m) accounting for extension factor
            vehicle_length = 4.5*1.2
            # Saturation flow found by multiplying speed limit (m/s) by vehicle
            # length and again by the number of lanes.
            # Comes out to 32727 vph per approach for 15 m/s.
            saturation_flow = speed_limit/vehicle_length*3  # veh/s
            # 2.5x for left volume, 1.5x for right volume, from vpm to veh/s
            design_flow = (vpm_left*2.5 + vpm_through + vpm_right*1.5) / 60

            # 1s reaction time + full braking distance
            max_stopping_dist = speed_limit*1 + speed_limit**2/2.6
            t_intergreen = (max_stopping_dist + vehicle_length + 4*8) / \
                speed_limit

            # Intergreen time and flows are per approach, so multiply by 4.
            t_cycle = (1.5*(4*t_intergreen) + 5) / \
                (1 - 4*design_flow/saturation_flow)
            ts_phase = ceil(t_cycle/4 * steps_per_second)

            # Get intersection lanes coords by approach.
            intersection_spec['manager_spec']['misc_spec']['cycle'] = (

                # Left approach (0)
                (frozenset((
                    (Coord(0.0, 14.0), Coord(18.0, 32.0)),  # left
                    (Coord(0.0, 10.0), Coord(32.0, 10.0)),  # through x3
                    (Coord(0.0, 14.0), Coord(32.0, 14.0)),
                    (Coord(0.0, 6.0), Coord(32.0, 6.0)),
                    (Coord(0.0, 6.0), Coord(6.0, -7.347880794884119e-16)),
                )), ts_phase),

                # Down approach (1)
                (frozenset((
                    (Coord(18.0, -2.4492935982947064e-16),
                     Coord(4.898587196589413e-16, 18.0)),
                    (Coord(18.0, -2.4492935982947064e-16), Coord(18.0, 32.0)),
                    (Coord(22.0, 0.0), Coord(22.0, 32.0)),
                    (Coord(26.0, 2.4492935982947064e-16), Coord(26.0, 32.0)),
                    (Coord(26.0, 2.4492935982947064e-16), Coord(32.0, 6.0))
                )), ts_phase),

                # Right approach (2)
                (frozenset((
                    (Coord(32.0, 18.0), Coord(14.0, 7.347880794884119e-16)),
                    (Coord(32.0, 18.0), Coord(4.898587196589413e-16, 18.0)),
                    (Coord(32.0, 22.0), Coord(0.0, 22.0)),
                    (Coord(32.0, 26.0), Coord(-4.898587196589413e-16, 26.0)),
                    (Coord(32.0, 26.0), Coord(26.0, 32.0))
                )), ts_phase),

                # Up approach (3)
                (frozenset((
                    (Coord(6.0, 32.0), Coord(-4.898587196589413e-16, 26.0)),
                    (Coord(6.0, 32.0), Coord(6.0, -7.347880794884119e-16)),
                    (Coord(14.0, 32.0), Coord(14.0, 7.347880794884119e-16)),
                    (Coord(10.0, 32.0), Coord(10.0, 0.0)),
                    (Coord(14.0, 32.0), Coord(32.0, 14.0))
                )), ts_phase)

            )

        else:
            raise NotImplementedError("TODO: Hardcode other lane pathfinders.")

        super().__init__(road_specs, [intersection_spec], spawner_specs,
                         remover_specs, od_pair,
                         acceptable_crash_mev=acceptable_crash_mev,
                         visualize=visualize, visualize_tiles=visualize_tiles,
                         steps_per_second=steps_per_second)
