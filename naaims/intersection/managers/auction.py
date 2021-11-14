from __future__ import annotations
from typing import (TYPE_CHECKING, Dict, Iterable, Set, Tuple, Type, Any,
                    DefaultDict, FrozenSet, Optional)
from enum import Enum
from itertools import chain, combinations

import naaims.shared as SHARED
from naaims.intersection.managers.manager import IntersectionManager


if TYPE_CHECKING:
    from naaims.util import Coord
    from naaims.intersection.tilings import Tiling
    from naaims.intersection.tilings.tiles import Tile
    from naaims.intersection.reservation import Reservation
    from naaims.intersection import IntersectionLane
    from naaims.road import RoadLane
    from naaims.vehicles.vehicle import Vehicle


class Mechanism(Enum):
    FIRST_PRICE = 0
    SECOND_PRICE = 1
    EXTERNALITY = 2


class AuctionManager(IntersectionManager):

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
        super().__init__(
            incoming_road_lane_by_coord=incoming_road_lane_by_coord,
            outgoing_road_lane_by_coord=outgoing_road_lane_by_coord,
            lanes=lanes,
            lanes_by_endpoints=lanes_by_endpoints,
            tiling_type=tiling_type,
            tiling_spec=tiling_spec,
            misc_spec=misc_spec)

        # Multiple dispatch allows more than one incoming road lane to let
        # vehicles through in a single auction.
        self.multiple: bool = misc_spec.get('multiple', False)

        # Sequencing allows more than one vehicle per road lane to enter in
        # a single auction, so long as they're immediately following the first
        # vehicle in the intersection securing a reservation and have the same
        # movement as the first vehicle.
        self.sequence: bool = misc_spec.get('sequence', False)

        if self.multiple and self.sequence:
            raise NotImplementedError("Multiple dispatch and sequencing not "
                                      "yet supported.")

        # The mechanism decides how participants bid and how much they pay.
        mechanism: str = misc_spec.get('mechanism', 'first').lower()
        self.mechanism: Mechanism
        if mechanism in {'2nd', 'second', 'second price', '2nd price'}:
            self.mechanism = Mechanism.SECOND_PRICE
        elif mechanism in {'externality', 'vcg', 'vcg-like'}:
            self.mechanism = Mechanism.EXTERNALITY
        else:
            self.mechanism = Mechanism.FIRST_PRICE

        # Whether to floor payments at 0.
        self.floor: bool = misc_spec.get('floor', True)

        # Unpack externality mechanism specific parameters.
        #   average vehicles per minute per lane, converted to per second units
        #   average VOT per vehicle per lane
        vpm_mean = misc_spec.get('vpm_mean', {})
        vot_mean = misc_spec.get('vot_mean', {})
        self.vps_mean: Dict[RoadLane, float] = {}
        self.vot_mean: Dict[RoadLane, float] = {}
        for rl_coord, rl in incoming_road_lane_by_coord.items():
            self.vps_mean[rl] = vpm_mean.get(rl_coord, 0.)/60
            self.vot_mean[rl] = vot_mean.get(rl_coord, 0.)

        # Initialize log of vehicles' payments at this intersection.
        self.payments: DefaultDict[Vehicle, float] = DefaultDict(lambda: 0.)

    def process_requests(self) -> None:
        """Issue reservations by auction if the intersection is clear."""

        # Wait until the intersection's clear before running the next auction.
        if not self.check_empty():
            return

        # Collect eligible winners, i.e., reservation requests from each lane.
        request_to_rl, rl_to_leading_request, sum_vot, start_idx = \
            self.get_leading_requests()

        # Identify which requests are incompatible with each other. This uses
        # the tiling if multiple requests can be dispatched at once, otherwise
        # it just returns sets of single requests.
        incompatible_pairs: Set[FrozenSet[Reservation]] = \
            AuctionManager.incompatible_pairs(request_to_rl.keys(),
                                              self.multiple)

        # Find all sets of road lanes whose lane leading requests are eligible
        # to win together. The set of road lanes can be converted back into
        # winning reservations later.
        rl_sets_to_consider = AuctionManager.request_sets_to_consider(
            request_to_rl, incompatible_pairs, self.multiple)

        # Find the winner, the eligible request set with the highest bid.
        (winning_rls, winning_vot, first_losing_rls, first_losing_vot,
         all_set_vot) = \
            AuctionManager.find_winner(rl_sets_to_consider,
                                       rl_to_leading_request, sum_vot,
                                       self.mechanism, self.sequence)

        # Find the payment for each winning vehicle.
        payments: Dict[Vehicle, float]
        if self.mechanism in {Mechanism.FIRST_PRICE, Mechanism.SECOND_PRICE}:
            payments = AuctionManager.payment_simple(
                winning_rls, rl_to_leading_request, start_idx, winning_vot,
                first_losing_vot, self.mechanism)
        elif self.mechanism is Mechanism.EXTERNALITY:
            payments = AuctionManager.payment_externality(
                winning_rls, winning_vot, first_losing_vot,
                frozenset(self.incoming_road_lane_by_coord.values()),
                rl_to_leading_request, all_set_vot, start_idx, sum_vot,
                self.vps_mean, self.vot_mean)
        else:
            raise ValueError("Invalid payment mechanism.")

        # Log the winners' payments.
        for vehicle, payment in payments.items():
            self.payments[vehicle] += payment

        # Confirm the winning reservations.
        for rl in winning_rls:
            reservation: Optional[Reservation] = rl_to_leading_request[rl]
            while reservation is not None:
                self.tiling.confirm_reservation(reservation, rl)
                reservation = reservation.dependency

        # Clear unused potential reservations from the tiling.
        # TODO: (low) probably unnecessary.
        self.tiling.clear_potential_reservations()

    def check_empty(self) -> bool:
        """Check if this intersection is empty and nothing is scheduled."""
        return not ((len(self.tiling.active_reservations) > 0) or
                    (len(self.tiling.queued_reservations) > 0))

    def get_leading_requests(self) -> Tuple[
            Dict[Reservation, RoadLane], Dict[RoadLane, Reservation],
            Dict[RoadLane, float], Dict[RoadLane, int]]:
        """Fetch the leading reservation request from each lane.

        Returns only the first reservation in a RoadLane sequence, even if the
        self.sequence flag is enabled. In the latter case, the reservation
        itself will link to trailing reservations of consecutive vehicles with
        the same desired movement (intersection lane) as this vehicle, up to
        the number of consecutive reservations that were successful. The
        leader's movement winning is a necessary prerequisite to the trailing
        reservations being approved, so this is sufficient to return.
        """
        request_to_rl: Dict[Reservation, RoadLane] = {}
        rl_to_leading_request: Dict[RoadLane, Reservation] = {}
        sum_vot: Dict[RoadLane, float] = {}
        start_idx: Dict[RoadLane, int] = {}
        for road_lane in self.incoming_road_lane_by_coord.values():
            # We only need to mark potential reservations if more than one
            # can win an individual auction.
            request = self.tiling.check_request(road_lane, mark=self.multiple,
                                                sequence=self.sequence)
            if request is not None:
                reservation = request[0]
                idx_start = request[1]

                request_to_rl[reservation] = road_lane
                rl_to_leading_request[road_lane] = reservation
                sum_vot[road_lane] = AuctionManager.get_bid(road_lane,
                                                            idx_start)
                start_idx[road_lane] = idx_start
        return request_to_rl, rl_to_leading_request, sum_vot, start_idx

    @staticmethod
    def get_bid(lane: RoadLane, idx_start: int) -> float:
        bid = 0.
        for vehicle in lane.vehicles[idx_start:]:
            bid += vehicle.vot
        return bid

    @staticmethod
    def incompatible_pairs(requests: Iterable[Reservation], multiple: bool
                           ) -> Set[FrozenSet[Reservation]]:
        """Identify all pairs of incompatible reservations.

        If not multiple, only singleton reservations are allowed, so simply
        return an empty set.
        """

        if not multiple:
            return set()

        seen: Set[Tile] = set()
        seen_twice: Set[Tile] = set()
        incompatible_pairs: Set[FrozenSet[Reservation]] = set()

        # For each reservation, iterate through every tile it uses.
        for request in requests:
            for tiles in request.tiles.values():
                for tile in tiles:
                    if tile in seen:
                        if tile not in seen_twice:
                            # We need only consider a tile if it shows up in
                            # more than one reservation. Furthermore, we only
                            # need to consider it once even if it shows up in
                            # three or more reservations.
                            seen_twice.add(tile)
                            incompatible_pairs.update(
                                frozenset(pair) for pair in
                                tile.incompatible_pairs())
                    else:
                        seen.add(tile)
        # TODO: (stochastic auctions) Account for probability of usage, e.g.,
        #       two reservations may be compatible but if you add a third
        #       they're incompatible.

        return incompatible_pairs

    @staticmethod
    def request_sets_to_consider(
        request_to_rl: Dict[Reservation, RoadLane],
        incompatible_pairs: Set[FrozenSet[Reservation]],
        multiple: bool = False
    ) -> FrozenSet[FrozenSet[RoadLane]]:
        """Return all sets of road lanes that can be dispatched together.

        If not multiple, the only sets that need to be considered are singleton
        sets of individual road lanes.
        """

        if not multiple:
            return frozenset(frozenset([rl]) for rl in request_to_rl.values())

        request_sets_to_consider: Set[FrozenSet[RoadLane]] = set()
        for subset in chain.from_iterable(
                combinations(request_to_rl.keys(), r) for r in
                range(len(request_to_rl.keys())+1)):
            request_set: FrozenSet[Reservation] = frozenset(subset)

            # Check every pair of requests that the tiling tells us are
            # incompatible, and consider only the reservation sets that
            # don't contain any of these incompatible pairs.
            for incompatible_pair in incompatible_pairs:
                if incompatible_pair.issubset(request_set):
                    break
            else:
                # Didn't find any incompatible pairs in this request set so
                # it's eligible to win.
                rl_set = frozenset(request_to_rl[r]  # type: ignore
                                   for r in request_set)
                request_sets_to_consider.add(rl_set)
        return frozenset(request_sets_to_consider)

    @staticmethod
    def find_winner(rl_sets_to_consider: FrozenSet[FrozenSet[RoadLane]],
                    rl_to_leading_request: Dict[RoadLane, Reservation],
                    sum_vot: Dict[RoadLane, float], mechanism: Mechanism,
                    sequence: bool
                    ) -> Tuple[FrozenSet[RoadLane], float, FrozenSet[RoadLane],
                               float, Dict[FrozenSet[RoadLane], float]]:
        """Return the winning road lanes, their bid, and first losers' bid.

        First losers' bid isn't calculated (and defaults to 0) if the payment
        mechanism used isn't SECOND_PRICE.

        Break ties by deferring to the road lane set that serves more lanes. If
        still equal, break ties randomly.
        TODO: Change to the road lane set that serves more total vehicles,
              and implement randomness.
        """
        # TODO: (multiple dispatch and sequence) Every single possible
        #       combination of lanes and sequence length would need to be
        #       considered for both of these flags to be enabled.

        winning_vot: float = 0.
        winning_rls: FrozenSet[RoadLane] = frozenset()

        # Save the first losing bid for the SECOND_PRICE mechanism.
        first_losing_vot: float = 0.
        first_losing_rls: FrozenSet[RoadLane] = frozenset()

        # Save the bid of every eligible set for the EXTERNALITY mechanism.
        all_set_vot: Dict[FrozenSet[RoadLane], float] = {}
        # TODO: (multiple dispatch and sequence) Should be changed to
        #       reservations instead of road lanes.

        for rl_set in rl_sets_to_consider:
            # Sum the VOTs across the set of lanes to form the bid.
            set_vot = sum(sum_vot[rl] for rl in rl_set)

            # Record the set with the highest sum bid as the winner and
            # demote the current highest sum bid to the first loser.
            if (set_vot > winning_vot) or ((set_vot == winning_vot) and
                                           (len(rl_set) > len(winning_rls))):
                first_losing_vot = winning_vot
                first_losing_rls = winning_rls
                winning_vot = set_vot
                winning_rls = rl_set
            # If the current observation isn't larger than the current winner,
            # maybe it'll qualify as the first loser?
            elif (set_vot > first_losing_vot) or \
                ((set_vot == first_losing_vot) and
                 (len(rl_set) > len(first_losing_rls))):
                first_losing_vot = set_vot
                first_losing_rls = rl_set

            # If using the EXTERNALITY mechanism, record all bids.
            if mechanism is Mechanism.EXTERNALITY:
                all_set_vot[frozenset(rl for rl in rl_set)] = set_vot

        if sequence and (len(winning_rls) > 0):
            if len(winning_rls) > 1:
                raise RuntimeError("Can't apply sequencing to a multiple "
                                   "dispatch winning reservation.")
            rl = next(iter(winning_rls))

            # Extend the sequence in this road lane until the sequence is
            # exhausted or the adjusted bid falls below the first losing bid.
            # The adjusted bid is found by removing the VOT of vehicles not
            # benefiting from further sequencing (i.e., the vehicles in front)
            # from the lane sum VOT.
            AuctionManager.extend_sequence(rl_to_leading_request[rl],
                                           sum_vot[rl], first_losing_vot)

        return (winning_rls, winning_vot, first_losing_rls, first_losing_vot,
                all_set_vot)

    @staticmethod
    def extend_sequence(request: Reservation, bid_vot: float,
                        first_losing_vot: float) -> None:
        """Extend the sequence in this road lane assuming no conflicts.

        Extend the sequence in this road lane until the sequence is exhausted
        or the adjusted bid falls below the first losing bid.

        The adjusted bid is found by removing the VOT of vehicles not
        benefiting from further sequencing (i.e., the vehicles in front) from
        the lane sum VOT.
        """

        bid_trailing = bid_vot - request.vehicle.vot
        while request.dependency is not None:

            # Extending the sequence fails to beat the first losing VOT. End
            # the sequence before here.
            if bid_trailing < first_losing_vot:
                request.dependency = None
                break

            request = request.dependency
            bid_trailing -= request.vehicle.vot

    @staticmethod
    def payment_simple(winning_rls: FrozenSet[RoadLane],
                       rl_to_leading_request: Dict[RoadLane, Reservation],
                       start_idx: Dict[RoadLane, int], winning_vot: float,
                       first_losing_vot: float, mechanism: Mechanism
                       ) -> Dict[Vehicle, float]:
        """Return first or second price payment for all winning vehicles.

        When running sequenced auctions, every vehicle in a winning lane pays
        their bid for the entire time consumed by vehicles in front of them or
        that are them. For example, the first vehicle in a sequence pays its
        VOT only for the time of its movement while the second vehicle in the
        sequence pays for both the time consumed by the first vehicle and the
        time consumed by its own movement. The third vehicle, although its
        movement isn't part of the sequence and therefore doesn't get to move
        in this auction, still pays for the time consumed by the movements of
        all vehicles preceding it, just as it would have even if only the first
        vehicle had won.

        The first losing bid, in VOT units, is divided among all winning
        vehicles proportionately and converted into real valued payments by
        applying the time consumed by the reservations each vehicle is
        responsible for at the appropriate rate.
        """

        if mechanism is Mechanism.EXTERNALITY:
            raise ValueError("Only first and second price mechanisms supported"
                             " by this method.")

        payments: DefaultDict[Vehicle, float] = DefaultDict(lambda: 0)
        request: Optional[Reservation]
        if len(winning_rls) > 1:
            # Multiple dispatch reservation, but no sequence. Get the time this
            # intersection will be occupied by any of the winning reservations.
            t_min_occupied = AuctionManager.t_occupied(winning_rls,
                                                       rl_to_leading_request)

            correction = AuctionManager.price_correction(
                winning_vot, first_losing_vot, mechanism)

            # Look at every vehicle in every winning lane.
            for rl in winning_rls:
                for vehicle in rl.vehicles[start_idx[rl]:]:
                    # Their payment is the time the winning set they bid on
                    # uses the intersection for times their personal VOT.
                    payments[vehicle] = t_min_occupied * vehicle.vot * \
                        correction

        elif len(winning_rls) > 0:
            # Sequence. Winner and first loser are singular lanes.
            if len(winning_rls) > 1:
                raise RuntimeError("Can't apply sequencing to a multiple "
                                   "dispatch winning reservation.")
            rl = next(iter(winning_rls))
            supporters = rl.vehicles[start_idx[rl]:]
            request = rl_to_leading_request[rl]
            winning_vot_sequence = winning_vot

            # Calculate marginal payment per sequence length, for every vehicle
            # in this road lane.
            ts_last = SHARED.t
            while request is not None:

                # Calculate the marginal time used by adding another vehicle
                # to this sequence.
                assert request.exit_rear is not None
                t_marginal = (request.exit_rear.t - ts_last) * \
                    SHARED.SETTINGS.TIMESTEP_LENGTH

                correction = AuctionManager.price_correction(
                    winning_vot_sequence, first_losing_vot, mechanism)

                # Loop through every vehicle supporting the sequence up to the
                # length so far, i.e., the current vehicle and all vehicles
                # behind it and have them pay for t_marginal.
                for supporter in supporters:
                    payments[supporter] += t_marginal * supporter.vot * \
                        correction

                # Prepare for the next loop iteration by removing the now
                # sequenced vehicle from the supporters list, reducing the
                # effective winning VOT (if necessary), preparing for the next
                # t_marginal calculation, and finally moving the sequence
                # reservation check down one request.
                assert supporters.pop(0) is request.vehicle
                if mechanism is Mechanism.SECOND_PRICE:
                    winning_vot_sequence -= request.vehicle.vot
                ts_last = request.exit_rear.t
                request = request.dependency

        return payments

    @staticmethod
    def t_occupied(rls: FrozenSet[RoadLane],
                   rl_to_leading_request: Dict[RoadLane, Reservation]
                   ) -> float:
        """Return the max time occupied by reservations from a set of lanes.

        Referenced to the current timestep, assuming that the auction is being
        run the moment the intersection is clear.
        """

        # Find the latest exit time.
        ts_latest = SHARED.t
        for rl in rls:
            request = rl_to_leading_request[rl]
            assert request.exit_rear is not None
            if request.exit_rear.t > ts_latest:
                ts_latest = request.exit_rear.t

        # Get delta w.r.t. current timestep, and convert to seconds.
        return (ts_latest - SHARED.t) * SHARED.SETTINGS.TIMESTEP_LENGTH

    @staticmethod
    def price_correction(winning_vot: float, first_losing_vot: float,
                         mechanism: Mechanism) -> float:
        """Returns a correction factor based on winning and first losing VOTs.

        Used to scale by the winning bid to apply to the first losing bid if
        using the second price mechanism. This is used to scale down a bidder's
        personal VOT by the ratio the first losing bid over the winning total
        bid, which their personal bid contributed to.
        """
        if mechanism is mechanism.FIRST_PRICE:
            return 1
        return first_losing_vot / winning_vot

    @staticmethod
    def payment_externality(winning_rls: FrozenSet[RoadLane],
                            winning_vot: float, first_losing_vot: float,
                            all_rls: FrozenSet[RoadLane],
                            rl_to_leading_request: Dict[RoadLane, Reservation],
                            all_set_vot: Dict[FrozenSet[RoadLane], float],
                            start_idx: Dict[RoadLane, int],
                            sum_vot: Dict[RoadLane, float],
                            vps_mean: Dict[RoadLane, float],
                            vot_mean: Dict[RoadLane, float]
                            ) -> Dict[Vehicle, float]:
        """Returns the externality payment for this winning set.

        Note that the payment is the negative of the externality, because
        winning vehicles are paying for the externality they impose.
        """

        payment: Dict[Vehicle, float] = DefaultDict(lambda: 0.)

        # Sort all eligible sets by bid (sum VOT).
        sets_by_vot = sorted(all_set_vot.items(), key=lambda kv: kv[1],
                             reverse=True)

        # Find the intersection time consumed by the winning set.
        t_winner = AuctionManager.t_occupied(winning_rls,
                                             rl_to_leading_request)

        # For each winning road lane, find the bid of the highest bidding
        # set that doesn't contain this road lane so we can run a
        # counterfactual to see who would've won if each individual vehicle
        # in this road lane hadn't been there.
        winning_rls_without_rl: FrozenSet[RoadLane] = frozenset()
        for rl in winning_rls:
            winning_rls_without_rl = frozenset()
            winning_vot_without_rl = 0.
            for losing_set, losing_bid in sets_by_vot:
                if rl not in losing_set:
                    if losing_bid > winning_vot_without_rl:
                        winning_vot_without_rl = losing_bid
                        winning_rls_without_rl = losing_set
                    break

            # Find the intersection time consumed by the set that would've
            # won had the winning road lane we're looking at not been
            # participating.
            t_without_rl = AuctionManager.t_occupied(
                winning_rls_without_rl, rl_to_leading_request)

            # Find the payment for each vehicle in the winning road lane.
            for vehicle_i in rl.vehicles[start_idx[rl]:]:
                payment[vehicle_i] = AuctionManager.calculate_externality(
                    vehicle_i.vot, winning_vot, winning_vot_without_rl,
                    winning_rls, winning_rls_without_rl, all_rls, t_winner,
                    t_without_rl, sum_vot, vps_mean, vot_mean)

        if len(winning_rls) == 1:
            # This might be a sequenced auction. We'll start the analysis at
            # the second vehicle in the sequence, if there is one.
            rl = next(iter(winning_rls))
            request: Optional[Reservation] = rl_to_leading_request[rl]
            assert request is not None
            winning_vot -= request.vehicle.vot
            idx_supporters = start_idx[rl] + 1

            # Prepare to find how much additional time this next vehicle in the
            # sequence will use, and to compare against the marginal additional
            # time the first losing set would've used past the lane leading
            # movement (if any).
            assert request.exit_rear is not None
            ts_previous_exit = request.exit_rear.t

            # Prepare to determine if the first loser would've used more or
            # less time in the intersection that this sequence length. Also
            # find its bid.
            ts_first_loser = SHARED.t
            first_losing_vot = 0.
            first_losing_rls = winning_rls_without_rl
            if len(first_losing_rls) > 0:
                rl_first_loser = next(iter(first_losing_rls))
                first_losing_vot = sum_vot[rl_first_loser]
                request_first_loser = rl_to_leading_request[rl_first_loser]
                assert request_first_loser.exit_rear is not None
                ts_first_loser = request_first_loser.exit_rear.t

            # Iterate through the lane leading reservation for accepted
            # sequence lengths.
            request = request.dependency
            while request is not None:
                # Each successive sequence member has essentially won a mini-
                # auction against the highest value set that doesn't include
                # itself for the marginal additional time it uses, so we'll
                # charge participating bidders accordingly by adjusting the
                # t_winner and t_first_loser values in addition to the
                # winning_vot.

                # Calculate the marginal additional time the first losing set
                # would've used, if any.
                t_first_loser = max(ts_first_loser - ts_previous_exit, 0) * \
                    SHARED.SETTINGS.TIMESTEP_LENGTH

                # Calculate the marginal additional time used by this next
                # vehicle in the sequence (and prepare to find the value for
                # the next sequence member).
                assert request.exit_rear is not None
                t_winner = (request.exit_rear.t - ts_previous_exit) * \
                    SHARED.SETTINGS.TIMESTEP_LENGTH
                ts_previous_exit = request.exit_rear.t

                # Calculate the externality payments for the mini-auction
                # associated with the next vehicle in the sequence.
                for vehicle_i in rl.vehicles[idx_supporters:]:
                    payment[vehicle_i] += \
                        AuctionManager.calculate_externality(
                        vehicle_i.vot, winning_vot, first_losing_vot,
                        winning_rls, first_losing_rls, all_rls, t_winner,
                        t_first_loser, sum_vot, vps_mean, vot_mean)

                # Update our memory of where the sequence ends and the rest of
                # the non-sequenced trailing vehicles in the lane begins, the
                # remaining effective bid of these trailing vehicles, and set
                # the request to look at the next one in the sequence.
                idx_supporters += 1
                winning_vot -= request.vehicle.vot
                request = request.dependency

        return payment

    @staticmethod
    def t_movement(reservation: Reservation) -> float:
        """Returns the time taken by a movement, in seconds."""
        assert reservation.exit_rear is not None
        return (reservation.exit_rear.t - reservation.entrance_front.t) * \
            SHARED.SETTINGS.TIMESTEP_LENGTH

    @staticmethod
    def calculate_externality(vehicle_i_vot: float, winning_vot: float,
                              winning_vot_without_rl: float,
                              winning_rls: FrozenSet[RoadLane],
                              winning_rls_without_rl: FrozenSet[RoadLane],
                              all_rls: FrozenSet[RoadLane],
                              t_winner: float, t_without_rl: float,
                              sum_vot: Dict[RoadLane, float],
                              vps_mean: Dict[RoadLane, float],
                              vot_mean: Dict[RoadLane, float]) -> float:

        erstwhile_winning_rls, everyone_else = \
            AuctionManager.split_losers(
                winning_vot, winning_vot_without_rl,
                vehicle_i_vot, winning_rls, winning_rls_without_rl,
                all_rls)

        if winning_rls == erstwhile_winning_rls:
            return 0.

        return AuctionManager.externality(
            vehicle_i_vot, t_winner, t_without_rl, winning_rls,
            erstwhile_winning_rls, everyone_else, sum_vot,
            vps_mean, vot_mean)

    @staticmethod
    def split_losers(winning_vot: float, winning_vot_without_rl: float,
                     vehicle_i_vot: float, winning_rls: FrozenSet[RoadLane],
                     winning_rls_without_rl: FrozenSet[RoadLane],
                     all_rls: FrozenSet[RoadLane]
                     ) -> Tuple[FrozenSet[RoadLane], FrozenSet[RoadLane]]:
        """Identify the winning road lane set without vehicle i.

        When removing (the bid of) vehicle i from a winning lane, the new
        set of winning lanes can be one of two options:
            1. The same set of winning lanes (the winning lanes won by more
               than vehicle i bid)
            2. The largest bidding set of lanes that doesn't include vehicle i,
               i.e., that doesn't include the road lane it came from (which is
               an assumption we can make because multiple dispatch and
               sequenced reservations are not allowed in this setup)
        """

        # Our default assumption is that removing vehicle i won't change the
        # set of winning road lanes.
        erstwhile_winning_rls = winning_rls

        # Check if removing this vehicle causes the alternative winning set of
        # road lanes to win.
        if winning_vot - vehicle_i_vot < winning_vot_without_rl:
            erstwhile_winning_rls = winning_rls_without_rl

        # Find all road lanes not in either the original winner or the new,
        # erstwhile winner.
        everyone_else = frozenset(all_rls.difference(
            winning_rls.union(erstwhile_winning_rls)))

        return erstwhile_winning_rls, everyone_else

    @staticmethod
    def externality(vehicle_i_vot: float,
                    t_winner: float, t_erstwhile: float,
                    winning_rls: FrozenSet[RoadLane],
                    erstwhile_winning_rls: FrozenSet[RoadLane],
                    everyone_else: FrozenSet[RoadLane],
                    sum_vot: Dict[RoadLane, float],
                    vps_mean: Dict[RoadLane, float],
                    vot_mean: Dict[RoadLane, float]) -> float:
        """Returns the externality of an individual vehicle.

        Only applies if winning_rl and erstwhile_winning_rl don't share a
        RoadLane. In that case, the externality would just be 0 since this
        vehicle's presence had no effect on the outcome.
        """

        t_diff = t_erstwhile - t_winner

        vot_winner = sum(
            sum_vot.get(rl, 0) + vps_mean[rl]*vot_mean[rl]*t_erstwhile/2 for rl
            in winning_rls) - vehicle_i_vot
        vot_first_loser = sum(
            sum_vot.get(rl, 0) + vps_mean[rl]*vot_mean[rl]*t_winner/2 for rl
            in erstwhile_winning_rls)
        vot_everyone_else = sum(
            sum_vot.get(rl, 0) + vps_mean[rl]*vot_mean[rl] * max(t_diff, 0)/2
            for rl in everyone_else)

        return (vot_first_loser * t_winner) - (vot_winner * t_erstwhile) + \
            (vot_everyone_else * t_diff)

    def finish_exiting(self, vehicle: Vehicle) -> None:
        # Get the sum of this vehicle's payments at this intersection and
        # and set it to 0 if necessary.
        payment = self.payments[vehicle]
        if self.floor and (payment < 0):
            payment = 0.
        vehicle.payment += payment
        del self.payments[vehicle]
        return super().finish_exiting(vehicle)
