from __future__ import annotations
from typing import TYPE_CHECKING, Dict, Tuple, Type, Any, List

import aimsim.shared as SHARED
from aimsim.util import Coord
from aimsim.vehicles import Vehicle
from aimsim.intersection.tilings import Tiling
from aimsim.intersection import IntersectionLane
from aimsim.intersection.managers.manager import IntersectionManager

if TYPE_CHECKING:
    from aimsim.road import RoadLane


class StopSignManager(IntersectionManager):
    """
    The simplest priority policy. Only allow one vehicle in the intersection at
    a time. Vehicles are chosen by how long they've been at a complete stop at
    the intersection line; all others are illegible until they reach the line.
    """

    def __init__(self,
                 incoming_road_lane_by_coord: Dict[Coord, RoadLane],
                 outgoing_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Tuple[IntersectionLane, ...],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tiling_type: Type[Tiling],
                 tiling_spec: Dict[str, Any]
                 ) -> None:
        """Create a new stop sign intersection manager."""
        super().__init__(incoming_road_lane_by_coord,
                         outgoing_road_lane_by_coord,
                         lanes,
                         lanes_by_endpoints,
                         tiling_type,
                         tiling_spec)
        self.queue: List[RoadLane] = []
        self.intersection_is_empty = True

        # Calculate the tolerance for proximity to an intersection, which is
        # the minimum distance a vehicle can travel after coming to a complete
        # stop, i.e., doing the acceleration action in one timestep and then
        # doing enough braking actions in following timesteps to bring its
        # velocity back to 0. Boost this by a factor of 2 to account for
        # conservativeness in when the vehicle starts braking.
        a = SHARED.SETTINGS.min_acceleration
        b = SHARED.SETTINGS.min_braking
        t_a = SHARED.SETTINGS.TIMESTEP_LENGTH
        t_b = (-a/b if -a/b > 1 else 1)*t_a
        self.tol_closeness = 2*(.5*a*t_a**2 + (a*t_a)*t_b + .5*b*t_b**2)

    def process_requests(self) -> None:

        # Check each incoming lane not in queue for a new vehicle that's
        # stopped at the intersection line. Add these lanes to the queue.
        seen_lanes = set(self.queue)
        for road_lane in self.incoming_road_lane_by_coord.values():
            if road_lane not in seen_lanes:
                v_index = road_lane.first_without_permission()
                if (v_index is not None) and (v_index[0] == 0):
                    # The lane has at least one vehicle in it without
                    # permission to enter the intersection, and that vehicle is
                    # the most forward one in the lane.
                    vehicle = road_lane.vehicles[0]
                    p = road_lane.vehicle_progress[vehicle].front
                    if (p is not None) and (vehicle.velocity == 0) and \
                            ((1-p)*road_lane.trajectory.length
                             < self.tol_closeness):
                        # This first vehicle is stopped at the intersection
                        # line (or close enough to within one timestep) and is
                        # thus eligible to enter the intersection.
                        self.queue.append(road_lane)

        # If the intersection is empty and there are lanes with stopped
        # vehicles in the queue, pop the first incoming lane from the queue and
        # release its first vehicle into the intersection.
        if self.intersection_is_empty and (len(self.queue) > 0):
            self.queue.pop(0).vehicles[0].permission_to_enter_intersection = \
                True
            self.intersection_is_empty = False

    def finish_exiting(self, vehicle: Vehicle) -> None:
        super().finish_exiting(vehicle)
        self.intersection_is_empty = True
