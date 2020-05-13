"""
This module contains several intersection priority managers based on different
policies.

Different policies need different amounts of information.
"""


from abc import ABC, abstractmethod
from typing import (Iterable, Dict, List, TypedDict, NamedTuple, Optional,
                    Tuple, Set)

from ..util import Coord
from .tilings import Tiling
from .reservations import ReservationRequest, Reservation
from ..roads import Road


class IntersectionManager(ABC):

    def __init__(self, tiling: Tiling,
                 incoming_roads: Dict[Road, Tuple[Coord]]) -> None:
        self.tiling = tiling
        self.incoming_roads = incoming_roads

    def handle_logic(self) -> None:
        """Do everything the manager needs to do in a single step."""

        # have manager check for new reservation requests and process them
        # once done, clear its cache
        self.handle_requests()

    @abstractmethod
    def handle_requests(self) -> None:
        """Should get new reservation requests and process them.

        Check every incoming road for vehicles or platoons that are eligible to
        try and make a reservation (i.e., lane leaders). Accept, reject, or 
        hold onto their requests for a future step based on logic in this
        function.
        """

        # Request handling blueprint:

        # Loop through each lane in each incoming road for new requests
        # for request n road.get_requests(), either just once or until some
        # stopping condition, e.g. every lane has had its reservation rejected

        # First, check if the request is individually feasible or if it
        # conflicts with any already confirmed requests. If it works, it
        # becomes a potential reservation and gets saved to a list
        # pr = tiling.check_request(request)
        # potential_reservations = []

        # Provide the entire bundle of reservations to the tiling to find the
        # highest value subset of reservations
        # to_confirm = self.tiling.find_best_batch(potential_reservations)

        # Confirm the reservations that
        # for reservation in to_confirm:
        #       profile = self.tiling.confirm_reservation(pr)

        # Clear marked potential reservations if necessary
        raise NotImplementedError("Should be implemented in child classes.")


class FCFSManager(IntersectionManager):
    """
    FCFS is a first come first served priority policy.
    """

    def __init__(self, tiling: Tiling,
                 incoming_roads: Dict[Road, Tuple[Coord]]) -> None:
        super().__init__(tiling, incoming_roads)

    def handle_requests(self) -> None:
        # FCFS can keep polling lanes until they’re out of reservations or the
        # none of the lane leader reservations work

        # create a dict to flag if a lane should be polled for requests again
        # this step
        poll_lane: Dict[Coord, bool] = {
            coord: True
            for coords
            in self.incoming_roads.values()
            for coord
            in coords
        }

        # while there are lanes that could still give a request we haven't
        # looked at yet, i.e., all lanes on a fresh run and then only lanes
        # that we accepted requests for last loop
        # while not all(self.pending_reservations.values())
        while any(poll_lane.values()):
            for road, lanes in self.incoming_roads.items():
                skip: Set[Coord] = set()
                for lane_coord in lanes:
                    if not poll_lane[lane_coord]:
                        # skip polling this lane this loop
                        skip.add(lane_coord)
                requests_to_process = road.get_reservation_candidates(
                    skip=skip)
                for request in requests_to_process:
                    # check if this request works
                    pr: Optional[Reservation
                                 ] = self.tiling.check_request(request)
                    if pr:
                        # if it does, confirm requested reservation
                        self.tiling.confirm_reservation(pr)
                    else:
                        # if not, mark this lane so it's not polled again
                        poll_lane[request.res_pos] = False

        # Tiles weren't marked for potential requests, so nothing to do here.


class SignalsManager(IntersectionManager):
    """
    Signals implements a traffic signal priority policy, i.e., red/green
    lights.
    """

    def handle_requests(self) -> None:
        # [Notes]
        # we need to maintain the assumption that every vehicle is accelerating
        # to the speed limit in the intersection

        # each time we admit a vehicle, remember when it was admitted and what
        # it max acceleration and braking were (can probably just keep a
        # pointer to the vehicle). prevent the next vehicle from entering
        # until . DO NOT use tiling to do this, since in FCFSSignals all
        # tiles associated with green will be reserved.

        # for vehicles coming in from a green lane, stagger requests such that
        # we can assume that everyone can accelerate to the speed limit in the
        # intersection without crashing within in the intersection. (I guess
        # we don't care what happens on the other end).

        # [Implementation]
        # 1. update the cycle step, if necessary
        raise NotImplementedError("TODO")

        # 2. poll for requests from green light lanes only

        # 3. given a request, assuming it's accelerating to or at the speed
        #    limit the entire time it's in the intersection, check if it can
        #   a. reach the end of the intersection before the light changes
        #   b. not collide with the vehicle dispatched before it from the
        #       same lane, regardless of if it's making the same movement
        #   c. pass the downstream road's check_entrance_collision check
        #       If it fulfills all of the above criteria, let it through.
        #       Note that we completely ignore the tiling here.

        # 4. if the request was accepted, save a pointer to the vehicle for the
        #    next check and repeat from 2 until we reject the first request.


class FCFSSignalsManager(SignalsManager, FCFSManager):

    def handle_requests(self) -> None:
        # For FCFS-Light, resolve differently for human-driven vehicles and
        # automated vehicles. If the vehicle is expected to arrive in a green
        # period, pass it a res unconditionally. If it isn't but it's AV, do
        # like FCFS. If it's human, reject. The green light rez also has the
        # vehicle do lane following behavior (TODO: how to do this).
        pass


class RequestWithFlag(NamedTuple):
    request: Optional[ReservationRequest] = None
    tried: bool = False


class AuctionManager(IntersectionManager):

    def __init__(self, tiling: Tiling,
                 incoming_roads: Dict[Road, Tuple[Coord]]) -> None:
        super().__init__(tiling, incoming_roads)
        self.pending_reservations: Dict[Road,
                                        Dict[Coord, RequestWithFlag]
                                        ] = {
            road: {
                c: RequestWithFlag() for c in coords
            } for road, coords in self.incoming_roads.items()
        }

    def handle_requests(self) -> None:
        # in SequencedAuctionManager, before sending another packet of request
        # iterables to tiling, check if there have been any changes in lane
        # since the last set. there's only ever a change if a new vehicle
        # arrives that has the same movement as the current packet of vehicles,
        # BUT WAIT the min time to intersection is based on not braking to stop
        # at the intersection line, so if it doesn't get a reservation by then
        # it's slowing down and the min time changes.

        # Auctions wait several time steps before accepting a single batch of
        # reservations

        # TODO: figure out how auction manager handles request caching

        pass
