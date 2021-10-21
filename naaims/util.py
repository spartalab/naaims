"""
Contains miscellaneous utility classes that makes other stuff work.
"""

from __future__ import annotations
from enum import Enum
from typing import TYPE_CHECKING, NamedTuple, Optional, Tuple
from math import sqrt, erf

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


def t_to_v(v0: float, a: float, vf: float) -> float:
    """Given v0, acceleration, and vf, find the time to reach vf."""
    return (vf - v0)/a


def x_over_constant_a(v0: float, a: float, t: float) -> float:
    """Given speed, acceleration, and time, find the distance covered."""
    return v0*t + (a/2)*t**2


def free_flow_exit(v0: float, a: float, v_max: float, t_to_v_max: float,
                   x_to_v_max: float, x_to_exit: float) -> Tuple[float,
                                                                 float]:
    """Return the time and velocity of the free flow exit."""
    if x_to_v_max > x_to_exit:
        # The vehicle will exit before reaching the speed limit.
        t_fastest_exit = (-v0 + (v0**2 + 2*a*x_to_exit)**.5) / a
        v_fastest_exit = v0 + a*t_fastest_exit
    else:
        # The vehicle will exit after reaching the speed limit.
        t_fastest_exit = t_to_v_max + \
            (x_to_exit - x_to_v_max)/v_max
        v_fastest_exit = v_max
    return t_fastest_exit, v_fastest_exit


def phi(z: float) -> float:
    """Return CDF value of z from the standard normal distribution."""
    return (1.0 + erf(z / sqrt(2.0))) / 2.0


def phi_mu_sigma(x: float, mu: float, sigma: float) -> float:
    """Return CDF value of x from the normal distribution."""
    return phi((x-mu)/sigma)
