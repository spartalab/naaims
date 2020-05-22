"""
Contains utility functions that makes other stuff work.
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import (TYPE_CHECKING, TypeVar, Type, NamedTuple, Optional, Any,
                    Iterable)

if TYPE_CHECKING:
    from .vehicles import Vehicle


class Coord(NamedTuple):
    x: float
    y: float


class VehicleSection(Enum):
    FRONT = 1
    CENTER = 2
    REAR = 3


class SpeedUpdate(NamedTuple):
    v: float
    a: float


class VehicleTransfer:
    """Pattern for transferring a vehicle from one object to another.

    Parameters:
        vehicle: Vehicle
            The vehicle to be added.
        t_left: float
            The vehicle moved some distance along the lane it came from in
            the current timestep before it reached the transition to this
            lane. This is the time left in the timestep after that move.
        pos: Coord
            The position of the end of the lane the vehicle is exiting from.
    """
    vehicle: Vehicle
    time_left: float
    pos: Coord
    section: VehicleSection


class CollisionError(Exception):
    """Raised when vehicles collide."""
    pass


class LinkError(Exception):
    """Raised when a road doesn't find an upstream or downstream object."""
    pass


class TooManyProgressionsError(Exception):
    def __init__(self,
                 msg='More than one vehicle transitioned between lanes in the '
                     'same timestep. This is usually result of the '
                     'ticks_per_second config being too low. Try increasing '
                     'it and running again.',
                 *args, **kwargs):
        super().__init__(msg, *args, **kwargs)
