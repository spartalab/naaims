from aimsim.intersection.tilings.tiles.tile import Tile


class StochasticTile(Tile):
    """
    Default Tile behavior is stochastic, so simply realize the abstract class
    as is.
    """

    def __init__(self, id: int, time: int, rejection_threshold: float = 0
                 ) -> None:
        super().__init__(id, time, rejection_threshold)
