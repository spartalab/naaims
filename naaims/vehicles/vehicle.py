"""
This module holds all vehicle types used in the AIM simulator.
"""

from __future__ import annotations
from abc import abstractmethod
from typing import Tuple, TypeVar, List, Optional, Dict, Any, Type
from copy import copy
from math import cos, pi, sin

import naaims.shared as SHARED
from naaims.util import Coord
from naaims.archetypes import Configurable

V = TypeVar('V', bound='Vehicle')


class Vehicle(Configurable):
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
                 throttle_mn: float = 0,
                 throttle_sd: float = 0,
                 tracking_mn: float = 0,
                 tracking_sd: float = 0,
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
                or better than the SHARED.SETTINGS.max_braking. Negative.
            length: float
                Length of the vehicle in meters
            width: float
                Width of the vehicle in meters
            throttle_mn: float
            throttle_sd: float
                Describes how well this vehicle tends to do at matching the
                acceleration profile it's given using a normal/Gaussian
                distribution. A positive value means that it generally
                accelerates more than it should so it moves faster and arrives
                earlier than it should, while a negative value means that it
                generally accelerates less than it should, so it takes longer
                than expected to cross an intersection and it's late to its
                reserved tiles. The magnitude describes how much faster or
                slower it completes its trajectory relative to the acceleration
                profile it was given.
            tracking_mn: float
            tracking_sd: float
                Describes how well this vehicle tends to do at tracking the
                trajectory it's given laterally through an intersection using a
                normal/Gaussian distribution. A positive value means that it
                deviates toward the right of the trajectory, while negative
                values mean that the vehicle tends to deviate to the left. The
                magnitude describes how much the deviation is relative to the
                length of the trajectory.
            vot: float
                The vehicle's value of time, for use in auction mechanisms.
        """

        if max_accel <= 0:
            raise ValueError("max_accel must be positive")
        if max_braking > SHARED.SETTINGS.min_braking:
            raise ValueError("max_braking must be as good or better than the "
                             "max_braking set in the shared settings.")
        if max_accel < SHARED.SETTINGS.min_acceleration:
            raise ValueError("max_accel must be as good or better than the "
                             "min_accel set in the shared settings.")

        self._vin = vin

        # initialize properties
        self.pos = Coord(0, 0)
        self.velocity = 0
        self.acceleration = 0
        self.heading = 0.0
        self.permission_to_enter_intersection = False
        self.has_reservation = False
        self.trailing = False

        # collect vehicle payments through intersection (auctions only)
        self.payment: float = 0.

        # save vehicle characteristics
        self.__destination = destination
        self.__max_acceleration = max_accel
        self.__max_braking = max_braking
        self.__length = length
        self.__length_buffered = length * \
            (1 + 2*SHARED.SETTINGS.length_buffer_factor)
        self.__length_half_buffered = length * \
            (.5 + SHARED.SETTINGS.length_buffer_factor)
        self.__width = width
        self.__throttle_mn = throttle_mn
        self.__throttle_sd = throttle_sd
        self.__tracking_mn = tracking_mn
        self.__tracking_sd = tracking_sd
        self.__vot = vot

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a manager spec dict."""
        # TODO: (spec) Interpret the string into the spec dict.
        raise NotImplementedError("TODO")

    @classmethod
    def from_spec(cls: Type[V], spec: Dict[str, Any]) -> V:
        return cls(vin=spec['vin'],
                   destination=spec['destination'],
                   max_accel=spec.get('max_accel',
                   SHARED.SETTINGS.min_acceleration),
                   max_braking=spec.get('max_braking',
                                        SHARED.SETTINGS.min_braking),
                   length=spec['length'],
                   width=spec['width'],
                   throttle_mn=spec['throttle_mn'],
                   throttle_sd=spec['throttle_sd'],
                   tracking_mn=spec['tracking_mn'],
                   tracking_sd=spec['tracking_sd'],
                   vot=spec['vot'])

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
    def length_buffered(self) -> float:
        return self.__length_buffered

    @property
    def length_half_buffered(self) -> float:
        return self.__length_half_buffered

    @property
    def width(self) -> float:
        return self.__width

    @property
    def throttle_mn(self) -> float:
        return self.__throttle_mn

    @property
    def throttle_sd(self) -> float:
        return self.__throttle_sd

    @property
    def tracking_mn(self) -> float:
        return self.__tracking_mn

    @property
    def tracking_sd(self) -> float:
        return self.__tracking_sd

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

    @property
    def trailing(self) -> bool:
        """Whether this vehicle is a trailing vehicle in a sequence."""
        return self._trailing

    @trailing.setter
    def trailing(self, trailing: bool) -> None:
        self._trailing = trailing

    # TODO: (sequence) Add chain_forward and chain_backward properties.
    #       Allow the chaining action to temporarily override a vehicle's max
    #       acceleration with the slowest acceleration in the chain.

    def stopping_distance(self, speed: Optional[float] = None) -> float:
        """Return the vehicle's stopping distance in meters.

        Defaults to the vehicle's current speed if a new one isn't provided.
        """
        if speed is None:
            speed = self.velocity
        return speed**2/(-2*SHARED.SETTINGS.min_braking)

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
