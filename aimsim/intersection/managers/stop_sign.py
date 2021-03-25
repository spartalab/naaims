from __future__ import annotations
from math import ceil, sqrt
from typing import TYPE_CHECKING, Iterable, Dict, Tuple, Type, Any, List

import aimsim.shared as SHARED
from aimsim.util import Coord, VehicleSection
from aimsim.lane import ScheduledExit
from aimsim.vehicles import Vehicle
from aimsim.intersection.tilings import Tiling
from aimsim.intersection import IntersectionLane
from aimsim.intersection.managers.manager import IntersectionManager

if TYPE_CHECKING:
    from aimsim.road import RoadLane


class StopSignManager(IntersectionManager):
    """
    A traffic signal priority policy, i.e., red and green lights.
    """

    def __init__(self,
                 upstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 downstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Iterable[IntersectionLane],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tiling_type: Type[Tiling],
                 tiling_spec: Dict[str, Any]
                 ) -> None:
        """Create a new stop sign intersection manager."""
        super().__init__(upstream_road_lane_by_coord,
                         downstream_road_lane_by_coord,
                         lanes,
                         lanes_by_endpoints,
                         tiling_type,
                         tiling_spec)
        self.queue: List[RoadLane] = []
        self.queued_exits: Dict[RoadLane, ScheduledExit] = {}
        self.anyone_permitted_in_intersection = False

    def process_requests(self) -> None:

        # Check each incoming lane not in queue for a new vehicle that's
        # stopped at the intersection line. Add these lanes to the queue.
        seen_lanes = set(self.queue)
        for lane in self.upstream_road_lane_by_coord.values():
            if lane not in seen_lanes:
                v_index = lane.first_without_permission()
                if (v_index is not None) and (v_index[0] == 0):
                    # The first vehicle in the lane, if it has a valid exit,
                    # needs permission to enter the intersection.
                    potential_exit = lane.soonest_exit(0)
                    if (potential_exit is not None) and \
                        (potential_exit.vehicle.velocity == 0) and \
                            (potential_exit.t <= SHARED.t + 1):
                        # The first vehicle in the lane is stopped and will
                        # enter the intersection in the next timestep (i.e.,
                        # it's stopped at the line).
                        # TODO: (runtime) time check may be too stringent.
                        self.queue.append(lane)
                        self.queued_exits[lane] = potential_exit

        # If intersection is empty and queue is non-empty, pop the first
        # incoming lane from the queue and release its first vehicle into the
        # intersection
        if self.anyone_permitted_in_intersection and (len(self.queue) > 0):
            lane = self.queue.pop(0)
            v_index = lane.first_without_permission()
            assert v_index is not None
            front_exit = self.queued_exits.pop(lane)
            vehicle = front_exit.vehicle
            length_traversal_time = ceil(sqrt(
                2*vehicle.length*(1+2*SHARED.SETTINGS.length_buffer_factor) /
                vehicle.max_acceleration))
            # TODO: Alter traversal time to account for speed limit.
            rear_exit = ScheduledExit(vehicle, VehicleSection.REAR,
                                      front_exit.t + length_traversal_time,
                                      vehicle.max_acceleration *
                                      length_traversal_time)
            self.tiling.issue_permission(vehicle, lane, rear_exit)
            vehicle.permission_to_enter_intersection = True
            self.anyone_permitted_in_intersection = True

    def finish_exiting(self, vehicle: Vehicle) -> None:
        super().finish_exiting(vehicle)
        self.anyone_permitted_in_intersection = False
