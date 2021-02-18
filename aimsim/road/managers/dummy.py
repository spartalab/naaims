from __future__ import annotations
from typing import TYPE_CHECKING, Iterable, Dict, Set

from aimsim.road.managers.manager import LaneChangeManager

if TYPE_CHECKING:
    from aimsim.lane import LateralDeviation
    from aimsim.road.lane import RoadLane
    from aimsim.vehicles import Vehicle


class DummyManager(LaneChangeManager):

    def __init__(self,
                 lanes: Iterable[RoadLane]
                 ):
        """Create a LaneChangeManager that doesn't allow lane changes.

        Used roads that leave from a spawner or end at a remover.
        (All roads in one intersection simulations.)
        """
        pass

    def vehicles_to_slow(self, lane: RoadLane) -> Set[Vehicle]:
        """Return an empty set since a dummy manager doesn't slow vehicles."""
        return set()

    def lateral_movements(self, lane: RoadLane) -> Dict[Vehicle,
                                                        LateralDeviation]:
        """Return an empty dict because the dummy doesn't do lane changes."""
        return {}

    def update_schedule(self) -> None:
        """DummyManager has no logic, so do nothing."""
        pass
