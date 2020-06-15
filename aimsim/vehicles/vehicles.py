"""
This module holds all vehicle types used in the AIM simulator.
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, TypeVar, Type, List
from copy import copy

import aimsim.shared as SHARED
from ..util import Coord

if TYPE_CHECKING:
    from ..lanes import RoadLane

V = TypeVar('V', bound='Vehicle')


class Vehicle(ABC):
    """
    Default vehicle behavior assumes full automation, which this abstract base
    class implements. Subclasses like semi-autonomous vehicles and human-
    driven vehicles should implement themselves as deviation from this base
    behavior.

    In this implementation, a Vehicle can't change its own properties, instead
    relying on the Lane object it's traveling in to do it via setters.
    """

    @abstractmethod
    def __init__(self,
                 vin: int,
                 destination: int,
                 max_accel: float = 3,
                 max_braking: float = -3.4,
                 length: float = 4.5,
                 width: float = 3,
                 throttle_score: float = 0,
                 tracking_score: float = 0,
                 vot: float = 0
                 ) -> None:
        """Construct a vehicle instance.

        Parameters
            vin: int
                The unique ID (vehicle identification number) of the vehicle.
            destination: int
                The ID of destination VehicleRemover
            max_accel: float
                The vehicle's maximum acceleration in m/s^2
            max_braking: float
                The vehicle's maximum braking speed in m/s^2. Must be as good
                or better than the global config SHARED.max_braking.
            length: float
                Length of the vehicle in meters
            width: float
                Width of the vehicle in meters
            throttle_score: float
                Describes how well this vehicle should generally do at
                following the acceleration instructions it's given. A positive
                value means that it generally accelerates more and brakes less
                than it should so it reaches tiles ahead of expected, while a
                negative value means that it generally accelerates less and
                brakes more than it should, so it's late to its reserved tiles.
                The magnitude describes how much it generally deviates from the
                proper throttling.
            tracking_score: float
                Describes how well this vehicle should generally do at
                following an intersection trajectory. For the meaning of its
                values, consider a vector pointing from south to north.
                Positive values mean to deviate from the trajectory laterally
                to the east, while negative values mean that the vehicle tends
                to deviate laterally to the west. The magnitude describes how
                strong this lateral deviation is typically.
            vot: float
                The vehicle's value of time, to be used in auctions.
        """

        if max_accel <= 0:
            raise ValueError("max_accel must be positive")
        if max_braking >= SHARED.max_braking:
            raise ValueError("max_braking must be as good or better than the "
                             "max_braking set in the global config.")

        self.vin = vin

        # initialize properties
        self.pos = Coord(0, 0)
        self.v = 0
        self.a = 0
        self.heading = 0.0
        self.permission_to_enter_intersection = False
        self.has_reservation = False

        # save vehicle characteristics
        self.destination = destination
        self.max_accel = max_accel
        self.max_braking = max_braking
        self.length = length
        self.width = width
        self.throttle_score = throttle_score
        self.tracking_score = tracking_score
        self.vot = vot

    @property
    def pos(self) -> Coord:
        """The vehicle's position in real (x,y) Coord."""
        return self._pos

    @pos.setter
    def pos(self, new_pos: Coord) -> None:
        self._pos: Coord = new_pos

    @property
    def v(self) -> float:
        """The vehicle's speed in m/s."""
        return self._v

    @v.setter
    def v(self, new_v: float) -> None:
        if new_v < 0:
            raise ValueError("Speed must be nonnegative.")
        self._v: float = new_v

    @property
    def a(self) -> float:
        """Vehicle's current acceleration in m/s^2."""
        return self._a

    @a.setter
    def a(self, new_a: float) -> None:
        # TODO: (runtime) Should this error or just clip a to 0?
        if self.v <= 0 and new_a < 0:
            raise ValueError("Vehicle already stopped.")
        self._a: float = new_a

    @property
    def heading(self) -> float:
        """The orientation of the vehicle in degrees."""
        return self._heading

    @heading.setter
    def heading(self, new_heading: float) -> None:
        if new_heading < 0 or new_heading >= 360:
            raise ValueError("Heading must be within [0,360) degrees")
        self._heading = new_heading

    @property
    def permission_to_enter_intersection(self) -> bool:
        """Check whether this vehicle has permission to enter an intersection.

        In the event of a traffic signal or similar control scheme, a vehicle
        may enter an intersection without a reservation.
        """
        return self._permission

    @permission_to_enter_intersection.setter
    def permission_to_enter_intersection(self, permission: bool) -> None:
        self._permission = permission

    @property
    def has_reservation(self) -> bool:
        """Check whether this vehicle has an intersection reservation."""
        return self._has_reservation

    @has_reservation.setter
    def has_reservation(self, has_reservation: bool) -> None:
        self._has_reservation = has_reservation

    # TODO: (sequencing) Add chain_forward and chain_backward properties.
    #       Allow the chaining action to temporarily override a vehicle's max
    #       acceleration with the slowest acceleration in the chain.

    def stopping_distance(self) -> float:
        """Return the vehicle's stopping distance in meters."""
        return self.v**2/(-2*self.max_braking)

    def next_movements(self, start_coord: Coord, at_least_one: bool = True
                       ) -> List[Coord]:
        """Return the vehicle's exit Coords for the next intersection.

        If the at_least_one flag is enabled, always return at least one
        possible coordinate, even if it can't possible get the vehicle to its
        destination.
        """
        return SHARED.pathfinder.next_movements(start_coord, self.destination,
                                                at_least_one)

    def clone_for_request(self: V) -> V:
        """Return a clone of this vehicle to test a reservation request."""
        return copy(self)

    def __hash__(self) -> int:
        return hash(self.vin)


class AutomatedVehicle(Vehicle):
    """
    Since the default vehicle is fully automated, all this class does is change
    the name of the base class so it's clear what type of vehicle we're using.
    """


class HumanDrivenVehicle(Vehicle):
    """
    Like an (Automated)Vehicle, but with more properties, like how accurate
    they are at following directions.
    """
    raise NotImplementedError("TODO")


class SimpleCCVehicle(Vehicle):
    """
    A connected vehicle with simple (fixed speed) cruise control.
    """
    raise NotImplementedError("TODO")


class AdaptiveCCVehicle(Vehicle):
    """
    A connected vehicle with adaptive (variable speed) cruise control.
    """
    raise NotImplementedError("TODO")
