"""
Contains miscellaneous utility classes that makes other stuff work.
"""

from __future__ import annotations
from enum import Enum
from typing import TYPE_CHECKING, NamedTuple, Optional

if TYPE_CHECKING:
    from .vehicles import Vehicle


class Coord(NamedTuple):
    """A simple way to track (x,y) coordinates consistently."""
    x: float
    y: float


class VehicleSection(Enum):
    """A consistent way to distinguish front, center, and rear sections."""
    FRONT = 0
    CENTER = 1
    REAR = 2


class SpeedUpdate(NamedTuple):
    """A consistent way to track a vehicle's speed and acceleration updates."""
    velocity: float
    acceleration: float


class VehicleTransfer(NamedTuple):
    """Pattern for transferring a vehicle from one object to another.

    This is primarily used to keep step_vehicles position updates consistent.
    Data necessary for speed updates should be handled by reservation requests.

    Parameters:
        vehicle: Vehicle
            The vehicle to be added.
        vehicle: VehicleSection
            The FRONT, CENTER, or REAR of the vehicle being transferred.
        d_left: Optional[float]
            Given its velocity in a timestep, the vehicle must move forward
            some fixed distance in this timestep. The vehicle moved some part
            of that distance along the lane it came from in the current
            timestep before it reached the transition to this new lane. d_left
            is the distance it has left to move in this timestep.
            d_left is only None when the vehicle enters from a spawner, in
            which case the Road needs to initialize its position wholesale.
        pos: Coord
            The position of the end of the lane the vehicle is exiting from.
    """
    vehicle: Vehicle
    section: VehicleSection
    distance_left: Optional[float]
    pos: Coord


class CollisionError(Exception):
    """Raised when vehicles collide."""
    pass


class MissingConnectionError(Exception):
    """Raised when a road doesn't find an upstream or downstream object."""
    pass
