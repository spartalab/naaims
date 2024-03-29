from __future__ import annotations
from typing import TYPE_CHECKING, Dict, List
from random import shuffle

from naaims.intersection.managers.manager import IntersectionManager

if TYPE_CHECKING:
    from naaims.road import RoadLane


class FCFSManager(IntersectionManager):
    """
    FCFS is a first come first served priority policy.
    """

    def process_requests(self) -> None:
        """Approve as many reservation requests as possible.

        FCFS can keep polling lanes until they’re out of reservations or the
        none of the lane leader reservations work
        """

        # Flag every incoming lane to determine if it should be polled for
        # requests (again) this step.
        # TODO: (FCFSSignals) Adjust this so that it can be changed to only
        #       look at whitelisted (i.e., non-green) lanes.
        poll_lane: Dict[RoadLane, bool] = {
            lane: True for lane in self.incoming_road_lane_by_coord.values()}

        # While there are lanes that could still return a request we haven't
        # looked at yet, i.e., all lanes on a fresh run and then only lanes
        # that we accepted requests for last loop.
        while any(poll_lane.values()):
            stop_polling_lanes: List[RoadLane] = []
            lanes = list(poll_lane.keys())
            shuffle(lanes)
            for lane in lanes:
                if poll_lane[lane]:  # Check for a new reservation request
                    # Have the tiling check for if there's a request in this
                    # lane and, if so, if it works.
                    request = self.tiling.check_request(lane)
                    if request is not None:
                        # It works, so confirm that reservation.
                        self.tiling.confirm_reservation(request[0], lane)
                    else:
                        # There is no request or if there is it doesn't work.
                        # Unflag this lane so it's not polled again.
                        stop_polling_lanes.append(lane)

            # Finalize the lanes for which not to flag.
            # (Don't modify a python object while iterating over it.)
            for lane in stop_polling_lanes:
                poll_lane[lane] = False

        # Tiles weren't marked for potential requests, so nothing to do here.
