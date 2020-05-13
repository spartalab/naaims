"""
The reservations module holds some datatypes necessary for reservation logic to
work.
"""

from typing import TYPE_CHECKING, NamedTuple, Iterable, Dict
from __future__ import annotations

from ..util import Coord
from ..vehicles import Vehicle
from ..lanes import IntersectionLane

if TYPE_CHECKING:
    from .tilings import Tile


class ReservationRequest(NamedTuple):
    """Pattern for requesting a reservation.

    Parameters:
        vehicle: 
            The vehicle(s) making the reservation. If multiple, use Platoon.
        res_pos: Coord
            The end of the road lane the vehicle is exiting from, i.e. where is
            it entering the intersection from. (NOT the coord where it wants to
            exit the intersection; you can get that target Coord from the first
            Vehicle since in the reservation.)
        min_time_to_intersection: float
            Minimum time for the vehicle to reach the intersection traveling as
            fast as possible (<= the speed limit) starting from its current
            velocity. (This assumes that it didn't have to stop and wait at
            the intersection because it couldn't get a reservation.)
        v_enter: float
            Maximum speed at which the vehicle can enter the intersection at.
            Corresponds to the minimum time to enter the intersection.
        vot: float
            Total value of time for the request. Defaults to 0 in case a
            simulation that doesn't use values doesn't spawn vehicles with
            VOTs initialized.
    """
    vehicle: Vehicle
    res_pos: Coord
    min_time_to_intersection: float
    v_enter: float
    vot: float = 0


class Reservation(NamedTuple):
    request: ReservationRequest
    # tiles used: the proportion at which they're used
    tiles: Dict[Tile, float]
    lane: IntersectionLane
    delay: float = 0
