from __future__ import annotations
from typing import TYPE_CHECKING, Iterable, Dict, Tuple, Type, Any, TypeVar

from aimsim.archetypes import Configurable
from aimsim.util import Coord, VehicleSection
from aimsim.lane import ScheduledExit
from aimsim.vehicles import Vehicle
from aimsim.intersection.tilings import Tiling, SquareTiling, ArcTiling
from aimsim.intersection.reservation import Reservation
from aimsim.intersection import IntersectionLane
from aimsim.intersection.managers.signal import SignalsManager
from aimsim.intersection.managers.fcfs import FCFSManager

if TYPE_CHECKING:
    from aimsim.road import Road
    from aimsim.road import RoadLane


class FCFSSignalsManager(SignalsManager, FCFSManager):

    def process_requests(self) -> None:
        # For FCFS-Light, resolve differently for human-driven vehicles and
        # automated vehicles. If the vehicle is expected to arrive in a green
        # period, pass it a res unconditionally. If it isn't but it's AV, do
        # like FCFS. If it's human, reject. The green light rez also has the
        # vehicle do lane following behavior.
        raise NotImplementedError("TODO")
