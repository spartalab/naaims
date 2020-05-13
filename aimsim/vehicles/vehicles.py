"""
The `vehicles` module tracks the properties of vehicles.

Assume routes are static (pre-emptive rerouting is a big ask, forced rerouting
same but a little less so)
Add routingerror is a vehicle gets off routes

vehicle stores its desired route through the intersection, updating pointers
along the way
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING
from __future__ import annotations

from ..util import Coord

if TYPE_CHECKING:
    from ..lanes import RoadLane


class Vehicle(ABC):
    """
    Vehicles are by default fully automated. So you can think of Vehicle as an
    AutomatedVehicle, minus the weirdness of having HumanDrivenVehicle be a
    subclass of an AutomatedVehicle.

    In this implementation, a Vehicle can't change its own properties, instead
    relying on the Lane objects it's traveling in to do it via setters.
    """

    def __init__(self,
                 vin: int,  # unique ID
                 start_pos,  # Coord in which this vehicle originates
                 end_pos,  # Coord that this vehicle wants to exit from
                 a_max=3,  # maximum acceleration, in m/s^2
                 # maximum (comfortable) braking, in m/s^2
                 # (4.5 for uncomfortable braking)
                 b_max=3.4,
                 l=4.5,  # length in meters
                 w=3,  # width in meters
                 v=11,  # vehicle speed, in meters per second
                 a=0,
                 vot=0  # value of time
                 ):
        """Construct a vehicle instance."""

        self.vin = vin

        self.pos: Coord = start_pos
        self.v: float = v
        self.a: float = a  # zero m/s^2
        self.heading: float = 0.0

        self.destination_lane = end_pos
        self.a_max = a_max
        self.b_max = b_max
        self.l = l
        self.w = w

        self.enter_intersection = False

    @property
    def pos(self) -> Coord:
        return self._pos

    @pos.setter
    def pos(self, new_pos: Coord) -> None:
        self._pos: Coord = new_pos

    # TODO: also store front of car, back of car, and box? or return live?
    #       they get returned each vis step so maybe cache them.

    #   A vehicle is always accelerating to or at the speed limit, unless,
    #   if between intersections, if the preceding vehicle were to brake at
    #   full power, if this vehicle were also to brake at full power, this
    #   vehicle would collide with the other vehicle.

    @property
    def v(self) -> float:
        return self._v

    # @v.setter
    # def v(self, new_v: float) -> None:
    #     if new_v < 0:
    #         raise ValueError("Speed must be nonnegative.")
    #     self._v: float = new_v

    @property
    def a(self) -> float:
        return self._a

    @a.setter
    def a(self, new_a: float) -> None:
        # TODO: error check the case where you try to slow down a stopped vehi
        self._a: float = new_a

    # def update_v(self) -> None:
    #     self.v =

    @property
    def heading(self) -> float:
        return self._heading

    @heading.setter
    def heading(self, new_heading: float) -> None:
        if new_heading < 0 or new_heading >= 360:
            raise ValueError("Heading must be within [0,360) degrees")
        self._heading = new_heading

    # TODO: should it have a property that says yes i have a reservation
    #       or is there a reservation object
    @property
    def has_reservation(self) -> bool:
        return self._has_reservation

    @has_reservation.setter
    def has_reservation(self, has_reservation: bool) -> None:
        self._has_reservation = has_reservation

    # TODO: does veh need a lane-i'm-in property
    #       or does it just need a reference to the Coord of the next transfer?
    # @property
    # def lane()

    def next_movement(self, lane: RoadLane):
        # tell a vehicle the lane it's in (we could derive it from its actual
        # position but that seems long and unnecessary) and make it to do a
        # shortest path calculation to its destination.

        # or just have a fixed list of movements, but that requires telling the
        # vehicle when it's completed a movement so it can switch the pointer
        # to its next movement
        raise NotImplementedError("TODO")

    # def update_next_movement(self):
    #     """The vehicle completed its last movement. Update for next time."""
    #     raise NotImplementedError("TODO")

    def __hash__(self):
        return hash(self.vin)


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


class Platoon(Vehicle):
    raise NotImplementedError("TODO")


class Sequence(Vehicle):
    """
    A group of sequential vehicles that behaves like a single vehicle, formed
    when a sequence of vehicles reserves together.

    It takes the worst acceleration and braking of the vehicles in the group.
    """
    raise NotImplementedError("TODO")

    # when a sequence gets a reservation, does it need to propogate it down to
    # its component vehicles? not sure. this might need more thinking.

    # contact platoons need to be treated carefully because if consecutive
    # vehicles are far away and have different braking and acceleration
    # capabilities, we can't have a "Vehicle" that changes length as they
    # accelerate differently
