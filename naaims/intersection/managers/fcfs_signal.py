from __future__ import annotations
from typing import TYPE_CHECKING, Iterable, Dict, Tuple, Type, Any, TypeVar

from naaims.archetypes import Configurable
from naaims.util import Coord, VehicleSection
from naaims.lane import ScheduledExit
from naaims.vehicles import Vehicle
from naaims.intersection.tilings import Tiling, SquareTiling
from naaims.intersection.reservation import Reservation
from naaims.intersection import IntersectionLane
from naaims.intersection.managers.signal import SignalsManager
from naaims.intersection.managers.fcfs import FCFSManager

if TYPE_CHECKING:
    from naaims.road import Road
    from naaims.road import RoadLane


class FCFSSignalsManager(SignalsManager, FCFSManager):

    def process_requests(self) -> None:
        # For FCFS-Light, resolve differently for human-driven vehicles and
        # automated vehicles. If the vehicle is expected to arrive in a green
        # period, pass it a res unconditionally. If it isn't but it's AV, do
        # like FCFS. If it's human, reject. The green light rez also has the
        # vehicle do lane following behavior.
        raise NotImplementedError("TODO")
