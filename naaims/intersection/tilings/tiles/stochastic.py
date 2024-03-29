from math import prod

from naaims.intersection.tilings.tiles.tile import Tile
from naaims.intersection.reservation import Reservation


class StochasticTile(Tile):
    """
    Default Tile behavior is stochastic, so simply realize the abstract class
    as is.
    """

    def will_reservation_work(self, r: Reservation, p: float = 1) -> bool:
        if super().will_reservation_work(r, p=p):
            return True
        else:
            ps = list(self.reserved_by.values())
            ps.append(p)
            p_none = prod(1-p_i for p_i in ps)
            p_one: float = 0.
            for i in range(len(ps)):
                p_none_except_i: float = 1.
                for j in range(len(ps)):
                    if i != j:
                        p_none_except_i *= 1-ps[j]
                p_one += ps[i] * p_none_except_i
            return 1 - p_none - p_one <= self.threshold

    def mark(self, r: Reservation, p: float = 1) -> None:
        raise ValueError("StochasticTile doesn't support potential "
                         "reservations.")

    def confirm(self, r: Reservation, p: float = 1) -> None:
        self.reserved_by[r] = max(p, self.reserved_by.get(r, 0))
