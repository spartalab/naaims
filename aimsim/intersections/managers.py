"""
This module contains several intersection priority managers based on different
policies.

The manager itself handles whether reservations are requested and the order in
which reservations are allocated. Once a reservation has been accepted,
continued maintenance of requests before their completion is handed off to the
tiling (which is owned by the manager).
"""


from __future__ import annotations
from abc import abstractmethod
from typing import Iterable, Dict, List, Tuple, Set, Type, Any, TypeVar

from ..archetypes import Configurable
from ..util import Coord, VehicleSection
from ..roads import Road
from ..lanes import IntersectionLane, RoadLane, ScheduledExit
from ..vehicles import Vehicle
from .tilings import Tiling, SquareTiling, ArcTiling
from .reservations import Reservation

M = TypeVar('M', bound='IntersectionManager')


class IntersectionManager(Configurable):

    @abstractmethod
    def __init__(self,
                 upstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 downstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Iterable[IntersectionLane],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tiling_type: Type[Tiling],
                 tiling_spec: Dict[str, Any]
                 ) -> None:
        self.upstream_road_lane_by_coord = upstream_road_lane_by_coord
        self.downstream_road_lane_by_coord = downstream_road_lane_by_coord
        self.lanes = lanes
        self.lanes_by_endpoints = lanes_by_endpoints

        # Finish the spec for the manager by providing the roads and lanes
        tiling_spec['upstream_road_lane_by_coord'
                    ] = self.upstream_road_lane_by_coord
        tiling_spec['downstream_road_lane_by_coord'
                    ] = self.downstream_road_lane_by_coord
        tiling_spec['lanes'] = self.lanes
        tiling_spec['lanes_by_endpoints'] = self.lanes_by_endpoints

        # Create the tiling
        self.tiling = tiling_type.from_spec(tiling_spec)

        # Child managers should call this for initial setup, then continue in
        # their own init to set up whatever they need to.

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a manager spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: interpret the string into the spec dict. cycle especially will
        #       be quite complicated!
        raise NotImplementedError("TODO")

        # TODO: enforce provision of separate tiling_type and tiling_config
        #       fields in intersection spec string
        tiling_type: str
        # Based on the spec, identify the correct tiling type
        if tiling_type.lower() in {'square', 'squaretiling'}:
            spec['tiling_type'] = SquareTiling
        elif tiling_type.lower() in {'arc', 'arctiling'}:
            spec['tiling_type'] = ArcTiling
        else:
            raise ValueError("Unsupported Tiling type.")

        # TODO: consider if children need to do any additional processing
        #       of the inputs, if there are any custom inputs? maybe this could
        #       be solved by calling an abstract class function, or we simply
        #       enforce that no every manager type has exactly the same input
        #       arguments

        return spec

    @classmethod
    def from_spec(cls: Type[M], spec: Dict[str, Any]) -> M:
        """Should interpret a spec dict to call the manager's init."""
        return cls(
            upstream_road_lane_by_coord=spec['upstream_road_lane_by_coord'],
            downstream_road_lane_by_coord=spec[
                'downstream_road_lane_by_coord'],
            lanes=spec['lanes'],
            lanes_by_endpoints=spec['lanes_by_endpoints'],
            tiling_type=spec['tiling_type'],
            tiling_spec=spec['tiling_config']
        )

    # Begin simulation cycle methods

    def handle_logic(self) -> None:
        """Update tiling, reservations, and poll for new requests to accept."""

        # First update the tiling for the new timestep
        self.tiling.handle_new_timestep()

        # Then decide whether to poll for new reservation requests and, if so,
        # which ones to accept.
        self.process_requests()

    @abstractmethod
    def process_requests(self):
        """Update the request queue and check for requests to accept.

        Check every incoming road for vehicles or platoons that are eligible to
        try and make a reservation (i.e., lane leaders). Accept, reject, or
        hold onto their requests for a future step based on logic in this
        function.
        """
        raise NotImplementedError("Should be implemented in child classes.")

    def start_reservation(self, vehicle: Vehicle) -> IntersectionLane:
        """Start a reservation in the tiling and return its lane."""
        return self.tiling.start_reservation(vehicle)

    def finish_exiting(self, vehicle: Vehicle) -> None:
        """Clean up permissions and reservations for exiting vehicle."""
        vehicle.permission_to_enter_intersection = False
        if vehicle.has_reservation:
            self.tiling.clear_reservation(vehicle)
            vehicle.has_reservation = False


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
            lane: True for lane in self.upstream_road_lane_by_coord.values()}

        # While there are lanes that could still return a request we haven't
        # looked at yet, i.e., all lanes on a fresh run and then only lanes
        # that we accepted requests for last loop.
        while any(poll_lane.values()):
            stop_polling_lanes: List[RoadLane] = []
            for lane, check in poll_lane.items():
                if check:  # Check for a new reservation request
                    # Have the tiling check for if there's a request in this
                    # lane and, if so, if it works.
                    prs = self.tiling.check_request(lane)
                    if len(prs) > 0:
                        # It works, so confirm that reservation.
                        self.tiling.confirm_reservation(prs[0], lane)
                    else:
                        # There is no request or if there is it doesn't work.
                        # Unflag this lane so it's not polled again.
                        stop_polling_lanes.append(lane)

            # Finalize the lanes for which not to flag.
            # (Don't modify a python object while iterating over it.)
            for lane in stop_polling_lanes:
                poll_lane[lane] = False

        # Tiles weren't marked for potential requests, so nothing to do here.


class SignalsManager(IntersectionManager):
    """
    A traffic signal priority policy, i.e., red and green lights.
    """

    def process_requests(self) -> None:
        # Poll for requests from green light lanes only. We only need to look
        # at each lane once.

        # TODO: Think about the case with one incoming lane into a through and
        #       a right turn lane.

        # Iterate through all incoming road lanes associated with at least one
        # greenlit movement/intersection lane.
        for lane, targets in self.tiling.greenlit.items():
            # Keep looking through the vehicles in this road lane until we
            # reach a vehicle we can't issue permission to (or there are no
            # vehicles # without permission left).
            while True:
                # Get the index of the first vehicle without permission to
                # enter that wants to go down one of the greenlit intersection
                # lanes, if there is one.
                seeking_perms = lane.first_without_permission(targets)
                if seeking_perms is None:
                    # The first vehicle does not want to use any of the green
                    # lanes. Move onto the next lane.
                    break
                else:
                    index: int = seeking_perms[0]

                # Check if the downstream lane has enough room for this vehicle
                vehicle: Vehicle = lane.vehicles[index]
                if vehicle.length > self.downstream_road_lane_by_coord[
                    vehicle.next_movements(lane.end_coord)[0]
                ].room_to_enter(tight=False):
                    break

                # Estimate this vehicle's exit parameters and use those to see
                # if this exit gives the vehicle enough time to clear the
                # intersection without colliding with the vehicle ahead of it.
                # (I don't think we can do that fully accurately without
                # simulating their entire movements, but we can at least get an
                # approximation.)
                this_exit: ScheduledExit = lane.soonest_exit(index)
                assert self.tiling.cycle is not None
                move_time: int
                # TODO: Find how long it'll take for the vehicle to cross the
                #       the IntersectionLane at max accel to the speed limit
                #       and add a second of padding.
                estimated_time_to_finish: int
                # TODO: Estimate when it will itself finish exiting by assuming
                #       the vehicle stays at this_exit.v to travel its own
                #       length.
                if move_time <= self.tiling.time_left_in_cycle:
                    self.tiling.issue_permission(
                        vehicle, lane, ScheduledExit(
                            vehicle=this_exit.vehicle,
                            section=VehicleSection.REAR,
                            t=this_exit.t+estimated_time_to_finish,
                            v=this_exit.v))


class FCFSSignalsManager(SignalsManager, FCFSManager):

    def process_requests(self) -> None:
        # For FCFS-Light, resolve differently for human-driven vehicles and
        # automated vehicles. If the vehicle is expected to arrive in a green
        # period, pass it a res unconditionally. If it isn't but it's AV, do
        # like FCFS. If it's human, reject. The green light rez also has the
        # vehicle do lane following behavior.
        raise NotImplementedError("TODO")


class AuctionManager(IntersectionManager):

    def __init__(self,
                 upstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 downstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Iterable[IntersectionLane],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tiling_type: Type[Tiling],
                 tiling_spec: Dict[str, Any],
                 sequencing: bool = False
                 ) -> None:
        super().__init__(
            upstream_road_lane_by_coord=upstream_road_lane_by_coord,
            downstream_road_lane_by_coord=downstream_road_lane_by_coord,
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
        for road_lane in self.upstream_road_lane_by_coord.values():
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
