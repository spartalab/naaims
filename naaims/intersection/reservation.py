from __future__ import annotations
from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, Tuple, Optional

if TYPE_CHECKING:
    from naaims.intersection.tilings.tiles import Tile
    from naaims.intersection.lane import IntersectionLane
    from naaims.util import Coord
    from naaims.vehicles import Vehicle
    from naaims.lane import ScheduledExit


@dataclass
class Reservation:
    """
    Vehicles waiting in a road lane to enter an intersection must successfully
    reserve timespace tiles in the intersection in order to pass through. The
    intersection's manager and tiling process reservation requests based on
    feasibility and priority to decide which reservations to accept.

    After a reservation is created, the only properties that are modified are
    the tiles (by adding new tiles used each timestep of the reservation test),
    the dependency property (if we see another vehicle in its sequence get
    added), and its_exit (when the reservation test progresses to the point
    where we know when the vehicle's rear section enters the intersection).

    Parameters
        vehicle: Vehicle
        res_pos: Coord
        tiles: Dict[int, Dict[Tile, float]]
            A dict with timesteps keyed to to the tiles used at that timestep
            and the proportion at which they're used.
        lane: IntersectionLane
        entrance_front: ScheduledExit
            The scheduled time and speed of this vehicle's front section's
            entrance from road into the intersection.
        entrance_rear: Optional[ScheduledExit] = None
            The scheduled time and speed of this vehicle's rear section's
            entrance from road into the intersection. Initialized as None and
            filled in when the reservation reaches the rear section entrance.
        entrance_rear: Optional[ScheduledExit] = None
            The scheduled time and speed of this vehicle's rear section's
            exit from intersection onto the downstream road. Initialized as
            None and filled in when the reservation completes.
        dependent_on: Tuple[Vehicle, ...] = ()
            The vehicles whose reservations this vehicle's reservation is
            dependent on, i.e., the vehicles preceding it in a sequenced
            reservation.
        dependency: Optional[Reservation] = None
            The reservation of the first vehicle immediately following this
            vehicle in a sequenced reservation, if any.
    """
    vehicle: Vehicle
    res_pos: Coord
    tiles: Dict[int, Dict[Tile, float]]
    lane: IntersectionLane
    entrance_front: ScheduledExit
    entrance_rear: Optional[ScheduledExit] = None
    exit_rear: Optional[ScheduledExit] = None
    dependent_on: Tuple[Vehicle, ...] = ()
    dependency: Optional[Reservation] = None

    def __hash__(self) -> int:
        return hash(self.vehicle)
