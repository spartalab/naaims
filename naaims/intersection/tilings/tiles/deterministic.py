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

    def will_reservation_work(self, r: Reservation, p: float = 1) -> bool:
        return (p < self.threshold) or \
            (not r.predecessors.isdisjoint(self.reserved_by)) or \
            super().will_reservation_work(r, p)

    def mark(self, r: Reservation, p: float = 1) -> None:
        """Log a potential reservation onto a tile.

        Only records if the reservation request has no predecessors already
        marked on the current tiling.
        """
        if (p >= self.threshold) and \
                r.predecessors.isdisjoint(self.potentials):
            self.potentials[r] = 1

    def confirm(self, r: Reservation, p: float = 1) -> None:
        if (p >= self.threshold) and \
                r.predecessors.isdisjoint(self.reserved_by):
            self.reserved_by[r] = 1
