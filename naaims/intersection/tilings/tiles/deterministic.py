from warnings import warn

from naaims.intersection.tilings.tiles.tile import Tile


class DeterministicTile(Tile):
    """
    Does everything Tile does, except as long as p>0, we reserve the entire
    tile. Like a regular Tile but more lightweight because all you need to do
    is check for sums.
    """

    # TODO: (low) streamline default tile logic to take advantage of these
    #       tiles being deterministic instead of stochastic.
    def __init__(self, id: int, time: int, rejection_threshold: float = 0
                 ) -> None:
        if rejection_threshold != 0:
            warn('Nonzero rejection threshold set to 0.')
        super().__init__(id, time, 0)
