from __future__ import annotations
from typing import TYPE_CHECKING, Iterable, Dict, Tuple, Type, Any, TypeVar

from aimsim.archetypes import Configurable
from aimsim.util import Coord, VehicleSection
from aimsim.lane import ScheduledExit
from aimsim.vehicles import Vehicle
from aimsim.intersection.tilings import Tiling, SquareTiling, ArcTiling
from aimsim.intersection.reservation import Reservation
from aimsim.intersection import IntersectionLane
from aimsim.intersection.managers.manager import IntersectionManager

if TYPE_CHECKING:
    from aimsim.road import Road
    from aimsim.road import RoadLane


class AuctionManager(IntersectionManager):

    def __init__(self,
                 incoming_road_lane_by_coord: Dict[Coord, RoadLane],
                 outgoing_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Tuple[IntersectionLane, ...],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tiling_type: Type[Tiling],
                 tiling_spec: Dict[str, Any],
                 sequencing: bool = False
                 ) -> None:
        super().__init__(
            incoming_road_lane_by_coord=incoming_road_lane_by_coord,
            outgoing_road_lane_by_coord=outgoing_road_lane_by_coord,
            lanes=lanes,
            lanes_by_endpoints=lanes_by_endpoints,
            tiling_type=tiling_type,
            tiling_spec=tiling_spec)
        self.sequencing = sequencing

    def process_requests(self) -> None:
        """Issue reservations by auction if the intersection is clear."""

        # Check if the intersection is clear. If not, don't run auction.
        for lane in self.lanes:
            if len(lane.vehicles) != 0:
                return

        # Assemble a reservation request for each lane.
        #
        # If the self.sequencing flag is enabled, the request will have, in
        # order, the reservation of the lane leader and the reservations of
        # consecutive vehicles with the same desired movement (intersection
        # lane) as the leader, up to the number of consecutive reservations
        # that were successful.
        requests: Dict[RoadLane, Iterable[Reservation]] = {}
        for road_lane in self.incoming_road_lane_by_coord.values():
            requests[road_lane] = self.tiling.check_request(road_lane,
                                                            mark=True,
                                                            sequence=True)

        # Find and accept the maximum value combination of requests by testing
        # all of them on the tiling using the potential reservation system. An
        # individual request's value is determined by the VOT provided in the
        # reservation times the time needed to complete the movement, including
        # time spent waiting for the first vehicle to enter the intersection.
        #
        # If reservation requests include sequences of vehicles, the maximum
        # could include only the first x of n vehicles in the sequence if it
        # results in a larger total value.
        #
        # Break ties by deferring to the request that serves more vehicles. If
        # equal, break ties randomly.
        reservations: Iterable[Tuple[RoadLane, Reservation]
                               ] = self.tiling.find_best_batch(requests)
        for road_lane, reservation in reservations:
            self.tiling.confirm_reservation(reservation, road_lane)

        # Clear unused potential reservations from the tiling.
        self.tiling.clear_potential_reservations()
