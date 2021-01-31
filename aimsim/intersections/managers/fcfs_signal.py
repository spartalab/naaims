from __future__ import annotations
from abc import abstractmethod
from typing import TYPE_CHECKING, Iterable, Dict, Tuple, Type, Any, TypeVar

from aimsim.archetypes import Configurable
from aimsim.util import Coord, VehicleSection
from aimsim.roads import Road
from aimsim.lane import ScheduledExit
from aimsim.vehicles import Vehicle
from aimsim.intersections.tilings import Tiling, SquareTiling, ArcTiling
from aimsim.intersections.reservation import Reservation
from aimsim.intersections import IntersectionLane
from aimsim.roads import RoadLane
from aimsim.intersections.managers.signal import SignalsManager
from aimsim.intersections.managers.fcfs import FCFSManager


class FCFSSignalsManager(SignalsManager, FCFSManager):

    def process_requests(self) -> None:
        # For FCFS-Light, resolve differently for human-driven vehicles and
        # automated vehicles. If the vehicle is expected to arrive in a green
        # period, pass it a res unconditionally. If it isn't but it's AV, do
        # like FCFS. If it's human, reject. The green light rez also has the
        # vehicle do lane following behavior.
        raise NotImplementedError("TODO")
