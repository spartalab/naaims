"""
Tiles are the base component of tilings. Each tiling is associated with an
(x,y) Coord and a timestep and records the probability that vehicles intend to
be in this position at this time. The goal is to minimize the likelihood that
two more more vehicles use the same tile, causing a collision, while still
maximizing intersection throughput.
"""

from typing import Dict, Set, Tuple
from abc import ABC, abstractmethod
from itertools import combinations

from naaims.intersection.reservation import Reservation


class Tile(ABC):
    """
    This default tile is stochastic because it demands more input parameters
    than a deterministic tile.
    """

    def __init__(self, id: int, time: int, threshold: float = 0
                 ) -> None:
        """Create a new tile, including if it tracks potential requests.

        Parameters
            id: int
                The ID of this tile based on xy position (for hashing).
            time: int
                The timestep that this tile tracks (for hashing).
            threshold: float
                Benchmark probability.

                For deterministic tiles, probabilities less than this are
                treated as if the vehicle doesn't use this tile (i.e., p is
                cast to 0), and those greater are cast to 1.

                For stochastic tiles, if the probability that confirming the
                next request makes the likelihood that this tile is used
                greater than this threshold, reject the request. (Does not
                apply to the first reservation on this tile.)
        """
        self.__hash = hash((id, time))
        self.potentials: Dict[Reservation, float] = {}
        self.reserved_by: Dict[Reservation, float] = {}
        if not (0 <= threshold <= 1):
            raise ValueError("Rejection threshold must be in [0,1].")
        self.threshold = threshold

    # TODO: (sequence) Change all of these tile checks to account for the total
    #       probability that a tile is used by every vehicle in its sequence.
    #       We assume that a vehicle will never collide with another vehicle in
    #       its sequence due to lane-following behavior, but uncertainty in
    #       movements means one spacetime tile can have nonzero probabilities
    #       from more than one vehicle.

    @abstractmethod
    def will_reservation_work(self, r: Reservation, p: float = 1) -> bool:
        """Try a request and return false if it can't work.

        Return whether this tile can accept the proposed reservation that will
        use this tile with probability b.

        If the tile has no confirmed reservations, it's free.
        If adding this reservation would put this tile over its probability of
        use rejection threshold, it's not free.

        Parameters
            r: Reservation
            p: float
                The probability that the request uses this tile.
        """
        if not (0 <= p <= 1):
            raise ValueError("p must be between 0 and 1 (inclusive).")
        return ((len(self.reserved_by) == 0) or
                (r in self.reserved_by)) if (p > 0) else True

    @abstractmethod
    def mark(self, r: Reservation, p: float = 1) -> None:
        """Should log a potential reservation onto a tile."""
        raise NotImplementedError("Must be implemented in child classes.")

    def remove_mark(self, r: Reservation) -> None:
        """Clear the marking for this reservation if it exists."""
        if r in self.potentials:
            del self.potentials[r]

    def confirm_reservation(self, r: Reservation, p: float = 1,
                            force: bool = False) -> None:
        """Confirm that a reservation will use this tile.

        Parameters
            r: Reservation
            p: float
                The probability that the reservation being requested uses this
                tile. (Only used for stochastic reservations.)
            force: bool
                Ignore the compatibility check and confirm this reservation no
                matter what. Should only be used when updating stochastic
                reservations that have already been confirmed.
        """

        # TODO: (low) Consider not bothering with checking if the request will
        #       work or for the force flag.
        if force or self.will_reservation_work(r, p):
            self.confirm(r, p)
        else:
            raise ValueError("This request is incompatible with this tile.")

    @abstractmethod
    def confirm(self, r: Reservation, p: float = 1) -> None:
        """Register this reservation in reserved_by."""
        raise NotImplementedError("Should be implemented in child classes.")

    def remove_all_marks(self) -> None:
        """Clear all markings on this tile."""
        self.potentials = {}

    def __hash__(self) -> int:
        """Return this tile's unique hash based on its spacetime position."""
        return self.__hash

    def _clear_all_reservations(self):
        """Self-explanatory. Only for debugging and automated cleanup."""
        self.potentials = {}
        self.reserved_by = {}

    def incompatible_pairs(self) -> Set[Tuple[Reservation, Reservation]]:
        """Return pairs of reservations that are mutually exclusive."""

        # TODO: (stochastic auctions) Account for probability of usage, e.g.,
        #       two reservations may be compatible but if you add a third
        #       they're incompatible.
        return set(combinations(self.potentials.keys(), 2))
