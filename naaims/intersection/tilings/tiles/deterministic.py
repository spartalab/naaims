from warnings import warn

from naaims.intersection.tilings.tiles.tile import Tile
from naaims.intersection.reservation import Reservation


class DeterministicTile(Tile):
    """
    Does everything Tile does, except as long as p>0, we reserve the entire
    tile. Like a regular Tile but more lightweight because all you need to do
    is check for sums.
    """

    # TODO: (low) streamline more of the default tile logic to take advantage
    #       of these tiles being deterministic instead of stochastic, e.g.,
    #       change reserved_by to a single vehicle instead of a dict.
    def __init__(self, id: int, time: int, rejection_threshold: float = 0
                 ) -> None:
        if rejection_threshold != 0:
            warn('Nonzero rejection threshold set to 0.')
        super().__init__(id, time, 0)

    def will_reservation_work(self, r: Reservation, p: float = 1) -> bool:
        return (len(self.reserved_by) == 0) or \
            (r.vehicle.vin in self.reserved_by)

    def confirm(self, r: Reservation, p: float = 1) -> None:
        if p > 0:
            self.reserved_by[r.vehicle.vin if r is not None else None] = 1
