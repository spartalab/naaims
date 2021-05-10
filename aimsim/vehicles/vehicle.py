"""
This module holds all vehicle types used in the AIM simulator.
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Tuple, TypeVar, List, Optional
from copy import copy
from math import cos, pi, sin

import aimsim.shared as SHARED
from aimsim.util import Coord

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
                or better than the global config SHARED.max_braking. Negative.
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
        if max_braking > SHARED.SETTINGS.min_braking:
            raise ValueError("max_braking must be as good or better than the "
                             "max_braking set in the global config.")
        if max_accel < SHARED.SETTINGS.min_acceleration:
            raise ValueError("max_accel must be as good or better than the "
                             "min_accel set in the global config.")

        self._vin = vin

        # initialize properties
        self.pos = Coord(0, 0)
        self.velocity = 0
        self.acceleration = 0
        self.heading = 0.0
        self.permission_to_enter_intersection = False
        self.has_reservation = False

        # save vehicle characteristics
        self.__destination = destination
        self.__max_acceleration = max_accel
        self.__max_braking = max_braking
        self.__length = length
        self.__width = width
        self.__throttle_score = throttle_score
        self.__tracking_score = tracking_score
        self.__vot = vot

    @property
    def vin(self) -> int:
        """The vehicle's identification number."""
        return self._vin

    @property
    def pos(self) -> Coord:
        """The vehicle's position in real (x,y) Coord."""
        return self._pos

    @pos.setter
    def pos(self, new_pos: Coord) -> None:
        self._pos: Coord = new_pos

    @property
    def velocity(self) -> float:
        """The vehicle's speed in m/s."""
        return self._v

    @velocity.setter
    def velocity(self, new_v: float) -> None:
        if new_v < 0:
            raise ValueError("Speed must be nonnegative.")
        self._v: float = new_v

    @property
    def acceleration(self) -> float:
        """Vehicle's current acceleration in m/s^2."""
        return self._a

    @acceleration.setter
    def acceleration(self, new_a: float) -> None:
        # TODO: (runtime) Should this error or just clip a to 0?
        if self.velocity <= 0 and new_a < 0:
            raise ValueError("Vehicle already stopped.")
        self._a: float = new_a

    @property
    def heading(self) -> float:
        """The orientation of the vehicle in radians."""
        return self._heading

    @heading.setter
    def heading(self, new_heading: float) -> None:
        if new_heading < 0 or new_heading >= 2*pi:
            raise ValueError("Heading must be in [0,2*pi)")
        self._heading = new_heading

    @property
    def destination(self) -> int:
        return self.__destination

    @property
    def max_acceleration(self) -> float:
        return self.__max_acceleration

    @property
    def max_braking(self) -> float:
        """The max braking rate for this vehicle in m/s^2. Always negative."""
        return self.__max_braking

    @property
    def length(self) -> float:
        return self.__length

    @property
    def width(self) -> float:
        return self.__width

    @property
    def throttle_score(self) -> float:
        return self.__throttle_score

    @property
    def tracking_score(self) -> float:
        return self.__tracking_score

    @property
    def vot(self) -> float:
        return self.__vot

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

    def stopping_distance(self, speed: Optional[float] = None) -> float:
        """Return the vehicle's stopping distance in meters.

        Defaults to the vehicle's current speed if a new one isn't provided.
        """
        if speed is None:
            speed = self.velocity
        return speed**2/(-2*self.__max_braking)

    def next_movements(self, enters_intersection_at: Coord,
                       at_least_one: bool = True) -> List[Coord]:
        """Return the vehicle's exit Coords for the next intersection.

        If the at_least_one flag is enabled, always return at least one
        possible coordinate, even if it can't possibly get the vehicle to its
        destination.
        """
        return SHARED.SETTINGS.pathfinder.next_movements(
            enters_intersection_at, self.__destination, at_least_one)

    def get_outline(self, buffer: bool = False, static_buffer: float = 0
                    ) -> Tuple[Coord, Coord, Coord, Coord]:
        """Return the vehicle's rectangular outline as four coordinates.

        Starts with the front left corner and works its way around clockwise.
        """
        if static_buffer < 0:
            raise ValueError("Static buffer must be nonnegative.")

        # Heading is the angle of the front of the car
        # Length vector from center to car front
        forward_vector = self.vector_forward()
        length_correction = Coord(forward_vector.x*(1+static_buffer),
                                  forward_vector.y*(1+static_buffer))
        # Width vector from center to car right
        right_vector = self.vector_right()
        width_correction = Coord(right_vector.x*(1+static_buffer),
                                 right_vector.y*(1+static_buffer))
        return (Coord(
            self.pos.x + length_correction.x - width_correction.x,
            self.pos.y + length_correction.y - width_correction.y
        ), Coord(
            self.pos.x + length_correction.x + width_correction.x,
            self.pos.y + length_correction.y + width_correction.y
        ), Coord(
            self.pos.x - length_correction.x + width_correction.x,
            self.pos.y - length_correction.y + width_correction.y
        ), Coord(
            self.pos.x - length_correction.x - width_correction.x,
            self.pos.y - length_correction.y - width_correction.y
        ))

    def vector_forward(self) -> Coord:
        """Return the vector of the car's front half as a relative Coord.

        Points from the center of the vehicle to the center of the front
        bumper.
        """
        return Coord(cos(self.heading)*self.length/2,
                     sin(self.heading)*self.length/2)

    def vector_rear(self) -> Coord:
        """Return the vector of the car's rear half as a relative Coord.

        Points from the center of the vehicle to the center of the rear bumper.
        """
        front = self.vector_right()
        return Coord(-front.x, -front.y)

    def vector_right(self) -> Coord:
        """Return the vector of the car's right half as a relative Coord.

        Points from the center of the vehicle to the center of its right side.
        """
        return Coord(cos(self.heading-pi/2)*self.width/2,
                     sin(self.heading-pi/2)*self.width/2)

    def vector_left(self) -> Coord:
        """Return the vector of the car's left half as a relative Coord.

        Points from the center of the vehicle to the center of its left side.
        """
        right = self.vector_right()
        return Coord(-right.x, -right.y)

    def clone_for_request(self: V) -> V:
        """Return a clone of this vehicle to test a reservation request."""
        return copy(self)

    def __hash__(self) -> int:
        return hash(self.vin)


# class HumanDrivenVehicle(Vehicle):
#     """
#     Like an (automated) vehicle, but with more properties, like how accurate
#     they are at following directions.
#     """
#     raise NotImplementedError("TODO")


# class SimpleCCVehicle(Vehicle):
#     """
#     A connected vehicle with simple (fixed speed) cruise control.
#     """
#     raise NotImplementedError("TODO")


# class AdaptiveCCVehicle(Vehicle):
#     """
#     A connected vehicle with adaptive (variable speed) cruise control.
#     """
#     raise NotImplementedError("TODO")
