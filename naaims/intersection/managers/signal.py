from __future__ import annotations
from typing import (TYPE_CHECKING, Dict, Tuple, Type, Any, FrozenSet, Set,
                    DefaultDict)
from math import ceil

import naaims.shared as SHARED
from naaims.util import (Coord, VehicleSection, free_flow_exit, t_to_v,
                         x_over_constant_a)
from naaims.lane import ScheduledExit
from naaims.intersection.managers.manager import IntersectionManager

if TYPE_CHECKING:
    from naaims.road import RoadLane
    from naaims.intersection.tilings import Tiling
    from naaims.intersection import IntersectionLane


class SignalManager(IntersectionManager):
    """
    A traffic signal priority policy, i.e., red and green lights.
    """

    def __init__(self,
                 incoming_road_lane_by_coord: Dict[Coord, RoadLane],
                 outgoing_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Tuple[IntersectionLane, ...],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tiling_type: Type[Tiling],
                 tiling_spec: Dict[str, Any],
                 misc_spec: Dict[str, Any] = {}
                 ) -> None:
        """Create a new traffic signal intersection manager."""
        super().__init__(incoming_road_lane_by_coord,
                         outgoing_road_lane_by_coord,
                         lanes,
                         lanes_by_endpoints,
                         tiling_type,
                         tiling_spec,
                         misc_spec)

        # Fetch the traffic cycle from the misc_spec and save it.
        cycle: Tuple[Tuple[FrozenSet[Tuple[Coord, Coord]], int], ...] = \
            misc_spec.get('cycle', tuple())
        self.cycle: Tuple[Tuple[FrozenSet[IntersectionLane], int], ...] = \
            tuple((frozenset(lanes_by_endpoints[lane_coords] for lane_coords in
                             lanes), time) for lanes, time in cycle)
        self.cycle_ts = sum(phase[1] for phase in self.cycle)

    def process_requests(self) -> None:

        # Poll for requests from greenlit lanes only. We need only look at each
        # lane once.
        greenlit, ts_left = self.get_phase()
        lanes: Set[RoadLane] = set()
        allowed_targets: DefaultDict[RoadLane, Set[Coord]] = \
            DefaultDict(lambda: set())
        for i_lane in greenlit:
            # Convert greenlit intersection lanes to incoming road lane and
            # target exit Coord
            lane = self.incoming_road_lane_by_coord[
                i_lane.trajectory.start_coord]
            lanes.add(lane)
            allowed_targets[lane].add(i_lane.trajectory.end_coord)

        # Iterate through all incoming road lanes associated with at least one
        # greenlit movement/intersection lane.
        for lane in lanes:

            lane_searched: bool = False

            # Keep looking through the vehicles in this road lane until we
            # reach a vehicle we can't issue permission to (or there are no
            # vehicles # without permission left).
            while not lane_searched:
                # Get the index of the first vehicle without permission to
                # enter that wants to go down one of the greenlit intersection
                # lanes, if there is one.
                seeking_perms = lane.first_without_permission(
                    allowed_targets[lane])
                if seeking_perms is None:
                    # The first vehicle does not want to use any of the green
                    # lanes. Move onto the next lane.
                    break
                else:
                    start, end_at = seeking_perms
                    counter = start

                for vehicle in lane.vehicles[start:(end_at + 1)]:

                    # Check if the downstream lane has enough room
                    # TODO: (multiple) Consider vehicles that have yet to exit.
                    if vehicle.length > self.outgoing_road_lane_by_coord[
                        vehicle.next_movements(lane.trajectory.end_coord)[0]
                    ].room_to_enter():
                        lane_searched = True
                        break

                    # Estimate this vehicle's exit and check if it has enough
                    # time to clear the intersection without colliding with the
                    # vehicle ahead of it.
                    entrance_front = lane.soonest_exit(counter)
                    if entrance_front is None:
                        # It can't exit its RoadLane in a valid way. Move on.
                        lane_searched = True
                        break
                    v0 = entrance_front.velocity
                    a = SHARED.SETTINGS.min_acceleration
                    v_max = SHARED.SETTINGS.speed_limit
                    t_to_v_max = t_to_v(v0, a, v_max)
                    i_lane_start = lane.trajectory.end_coord
                    i_lane = self.lanes_by_endpoints[(
                        i_lane_start, vehicle.next_movements(i_lane_start)[0])]
                    t_exit, v_exit = free_flow_exit(
                        v0, a, v_max, t_to_v_max,
                        x_over_constant_a(v0, a, t_to_v_max),
                        i_lane.trajectory.length)
                    ts_exit = entrance_front.t + \
                        ceil(SHARED.SETTINGS.steps_per_second * t_exit)

                    # TODO: (multiple) use v_exit to check for sufficient space
                    #       in the outgoing road lane.

                    if ts_exit <= ts_left:
                        self.tiling.issue_permission(
                            vehicle, lane, SignalManager.entrance_rear(
                                entrance_front, a, v_max, t_to_v_max))
                    else:
                        lane_searched = True
                        break

                    counter += 1

    def get_phase(self) -> Tuple[Set[IntersectionLane], int]:
        """Return the lanes allowed by the current signal phase and time left.

        That is, the intersection lanes allowed and timesteps left in the
        current signal phase.
        """

        assert len(self.cycle) > 0
        ts_current = SHARED.t % self.cycle_ts
        ts_check = 0
        allowed_lanes: Set[IntersectionLane] = set()
        for phase in self.cycle:
            ts_check += phase[1]
            if ts_check > ts_current:
                # This is the current cycle.
                allowed_lanes = phase[0]
                break

        # Find the remaining time in the phase.
        ts_left: int = ts_check - ts_current

        return allowed_lanes, ts_left

    @staticmethod
    def entrance_rear(entrance_front: ScheduledExit, a: float,
                      v_max: float, t_to_v_max: float) -> ScheduledExit:
        """Find the entrance of this vehicle's rear section.

        That is, its entrance from the incoming road into the intersection.
        """
        v0 = entrance_front.velocity
        vehicle = entrance_front.vehicle

        t_rear_exit: float
        v_rear_exit: float

        if v0 >= v_max:
            # Vehicle's at the speed limit both when its front and rear enters
            # the intersection.
            v_rear_exit = v_max
            t_rear_exit = vehicle.length_buffered / v_max
        else:
            x_to_v_max = x_over_constant_a(v0, a, t_to_v_max)
            x_rear_left = vehicle.length_buffered - x_to_v_max
            if x_rear_left >= 0.:
                # Vehicle reaches the speed limit in the time it takes to fully
                # enter the intersection.
                v_rear_exit = v_max
                t_rear_exit = (t_to_v_max + x_rear_left/v_max)
            else:
                # Vehicle doesn't reach the speed limit.
                t_rear_exit = (
                    -v0 + (v0**2 + 2*a*vehicle.length_buffered
                           )**.5)/a
                v_rear_exit = v0 + a*t_rear_exit

        return ScheduledExit(vehicle, VehicleSection.REAR,
                             ceil(t_rear_exit *
                                  SHARED.SETTINGS.steps_per_second)
                             + entrance_front.t, v_rear_exit)
