from typing import Optional, Set, Deque, Dict

from ..vehicles import Vehicle
from .reservations import ReservationRequest, Reservation


class Tile:
    """
    This default tile is stochastic because it demands more input parameters
    than a deterministic tile. 
    """

    # TODO lower priority: consider implementing a simpler, non-stochastic
    #                      tile class. Issue: they have different function
    #                      signatures.

    # TODO: consider banning the use of stochastic AND sequenced reservations.
    #       the API supports it but man that'd be a lot of overhead.

    # TODO: can make reservation without vehicle

    def __init__(self,
                 rejection_threshold: float = 0
                 ) -> None:
        """Create a new tile, including if it tracks potential requests.

        Parameters
            track: bool
                Whether or not to track potential requests.
            rejection_threshold: float
                (Used only for stochastic reservations subclass). If the
                probability that confirming the next request makes the
                likelihood that this tile is used greater than this threshold,
                reject the request. (Check does not apply to the first
                reservation on this tile.)
        """
        # self.__confirmed = False
        # self.__potentials: Set[PotentialReservation] = set()

        if rejection_threshold < 0:
            raise ValueError("Rejection threshold must be non-negative.")
        self.__potentials: Dict[Reservation, float] = {}
        self.__reserved_by: Dict[Optional[Vehicle], float] = {}
        # self.__track = track
        self.__rejection_threshold = rejection_threshold

    def will_request_work(self, r: ReservationRequest, p: float = 1) -> bool:
        """Try a request and return false if it can't work.

        Return whether this tile can accept the proposed reservation that will
        use this tile with probability b.

        If the tile has no confirmed reservations, it's free.
        If adding this reservation would put this tile over its probability of
        use rejection threshold, it's not free.

        Parameters
            r: ReservationRequest
            p: float
                The probability that the reservation being requested uses this
                tile. (Only used for stochastic reservations.)
        """
        # return self.__confirmed
        if (len(self.__reserved_by) == 0) or (r.vehicle in self.__reserved_by):
            return True
        else:
            return (
                sum(v for v in self.__reserved_by.values()) + p
            ) > self.__rejection_threshold

    def mark_tile(self, r: Reservation, p: float = 1) -> bool:
        """Try logging a potential reservation onto a tile."""
        if self.will_request_work(r.request, p):
            self.__potentials[r] = p
            return True
        else:
            return False
        # TODO: should it just error if it doesn't work?

    def confirm_request(self, r: Optional[ReservationRequest], p: float = 1,
                        force: bool = False) -> None:
        """Confirm that a reservation will use this tile.

        Parameters:
            r: ReservationRequest
            p: float
            force: bool
                Ignore the compatibility check and confirm this reservation no
                matter what. Should only be used when updating stochastic
                reservations that have already been confirmed.
        """
        # if self.__track and (r not in self.__potentials):

        # try:
        #     r in set(tpr.pr for tpr in self.__potentials)
        # except:
        #         r is not in set()):
        #     raise ValueError("This request was not approved for this Tile.")

        if (r is None) and (not force):
            raise ValueError("Empty reservations must be forced.")

        if force or self.will_request_work(r, p):
            if r is not None:
                self.__reserved_by[r.vehicle] = p
            else:
                self.__reserved_by[r] = p
        else:
            raise ValueError("This request is incompatible with this tile.")


class DeterministicTile(Tile):
    """
    Does everything Tile does, except as long as p>0, we reserve the entire
    tile. Like a regular Tile but more lightweight because all you need to do
    is check for sums.
    """

    # TODO: re-implement every
    raise NotImplementedError("TODO")
