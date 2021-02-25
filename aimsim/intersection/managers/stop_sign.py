from __future__ import annotations
from typing import TYPE_CHECKING, Iterable, Dict, Tuple, Type, Any, TypeVar

from aimsim.archetypes import Configurable
from aimsim.util import Coord, VehicleSection
from aimsim.lane import ScheduledExit
from aimsim.vehicles import Vehicle
from aimsim.intersection.tilings import Tiling, SquareTiling, ArcTiling
from aimsim.intersection.reservation import Reservation
from aimsim.intersection import IntersectionLane
from aimsim.intersection.managers.manager import IntersectionManager

if TYPE_CHECKING:
    from aimsim.road import Road
    from aimsim.road import RoadLane


class StopSignManager(IntersectionManager):
    """
    A traffic signal priority policy, i.e., red and green lights.
    """

    def process_requests(self) -> None:
        pass
