"""
Tiles are the base component of tilings. Each tiling is associated with an
(x,y) Coord and a timestep and records the probability that vehicles intend to
be in this position at this time. The goal is to minimize the likelihood that
two more more vehicles use the same tile, causing a collision, while still
maximizing intersection throughput.
"""

from typing import Optional, Dict
from abc import ABC, abstractmethod

from aimsim.vehicles import Vehicle
from aimsim.intersection.reservation import Reservation


class Tile(ABC):
    """
    This default tile is stochastic because it demands more input parameters
    than a deterministic tile.
    """

    # TODO: (sequence) (stochastic) Consider banning the use of stochastic AND
    #       sequenced reservations. It's supported but the overhead is a lot.

    @abstractmethod
    def __init__(self, id: int, time: int, rejection_threshold: float = 0
                 ) -> None:
        """Create a new tile, including if it tracks potential requests.

        Parameters
            id: int
                The ID of this tile based on xy position (for hashing).
            time: int
                The timestep that this tile tracks (for hashing).
            rejection_threshold: float
                (Used only for stochastic reservations subclass). If the
                probability that confirming the next request makes the
                likelihood that this tile is used greater than this threshold,
                reject the request. (Check does not apply to the first
                reservation on this tile.)
        """

        if rejection_threshold < 0:
            raise ValueError("Rejection threshold must be non-negative.")

        self.__hash = hash((id, time))
        self.__potentials: Dict[Reservation, float] = {}
        self.__reserved_by: Dict[Optional[Vehicle], float] = {}
        self.__rejection_threshold = rejection_threshold

    # TODO: (sequence) Change all of these tile checks to account for the total
    #       probability that a tile is used by every vehicle in its sequence.
    #       We assume that a vehicle will never collide with another vehicle in
    #       its sequence due to lane-following behavior, but uncertainty in
    #       movements means one spacetime tile can have nonzero probabilities
    #       from more than one vehicle.

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
                The probability that the reservation being requested uses this
                tile. (Only used for stochastic reservations.)
        """

        if (len(self.__reserved_by) == 0) or (r.vehicle in self.__reserved_by):
            return True
        else:
            return sum(v for v in self.__reserved_by.values()) + p < \
                self.__rejection_threshold

    def mark(self, r: Reservation, p: float = 1) -> None:
        """Log a potential reservation onto a tile."""
        self.__potentials[r] = p

    def remove_mark(self, r: Reservation) -> None:
        """Clear the marking for this reservation if it exists."""
        if r in self.__potentials:
            del self.__potentials[r]

    def confirm_reservation(self, r: Reservation, p: float = 1,
                            force: bool = False) -> None:
        """Confirm that a reservation will use this tile.

        Parameters:
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
            if r is not None:
                self.__reserved_by[r.vehicle] = p
            else:
                self.__reserved_by[r] = p
        else:
            raise ValueError("This request is incompatible with this tile.")

    def remove_all_marks(self) -> None:
        """Clear all markings on this tile."""
        self.__potentials = {}

    def __hash__(self) -> int:
        """Return this tile's unique hash based on its spacetime position."""
        return self.__hash
