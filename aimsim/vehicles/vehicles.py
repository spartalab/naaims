"""
The `vehicles` module tracks the properties of vehicles.

Assume routes are static (pre-emptive rerouting is a big ask, forced rerouting
same but a little less so)
Add routingerror is a vehicle gets off routes

vehicle stores its desired route through the intersection, updating pointers
along the way
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Union, TypeVar, Type

from ..util import Coord

if TYPE_CHECKING:
    from ..lanes import RoadLane
    from ..intersections.reservations import Reservation

V = TypeVar('V', bound='Vehicle')


class Vehicle(ABC):
    """
    Default vehicle behavior assumes full automation, which this abstract base
    class implements. Subclasses like semi-autonomous vehicles and human-
    driven vehicles should implement themselves as deviation from this base
    behavior.

    In this implementation, a Vehicle can't change its own properties, instead
    relying on the Lane objects it's traveling in to do it via setters.
    """

    @abstractmethod
    def __init__(self,
                 vin: int,  # unique ID
                 start_coord: Coord,
                 destination: int,  # ID of target VehicleRemover
                 max_accel: float = 3,  # maximum acceleration, in m/s^2
                 max_braking: float = -3.4,  # or -4.5, braking in m/s^2
                 length: float = 4.5,  # length in meters
                 width: float = 3,  # width in meters
                 vot: float = 0  # value of time
                 ):
        """Construct a vehicle instance."""

        if max_accel <= 0:
            raise ValueError("max_accel must be positive")
        if max_braking >= 0:
            raise ValueError("max_braking must be negative")

        self.vin = vin

        # initialize properties
        self.pos = start_coord
        self.v: float = 0
        self.a: float = 0
        self.heading: float = 0.0
        self.enter_intersection = False

        # save vehicle characteristics
        self.destination = destination
        self.max_accel = max_accel
        self.max_braking = max_braking
        self.length = length
        self.width = width

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

    @v.setter
    def v(self, new_v: float) -> None:
        if new_v < 0:
            raise ValueError("Speed must be nonnegative.")
        self._v: float = new_v

    @property
    def a(self) -> float:
        """Vehicle's current acceleration. Used only for plotting."""
        return self._a

    @a.setter
    def a(self, new_a: float) -> None:
        # TODO: error check the case where you try to slow down a stopped vehi
        self._a: float = new_a

    @property
    def heading(self) -> float:
        return self._heading

    @heading.setter
    def heading(self, new_heading: float) -> None:
        if new_heading < 0 or new_heading >= 360:
            raise ValueError("Heading must be within [0,360) degrees")
        self._heading = new_heading

    @property
    def can_enter_intersection(self) -> bool:
        """Check whether a vehicle has permission to enter a reservation.

        In the event of a traffic signal or similar control scheme, a vehicle
        may enter an intersection without a reservation.
        """
        return self._permission

    @can_enter_intersection.setter
    def can_enter_intersection(self, permission: bool) -> None:
        self._permission = permission

    @property
    def has_reservation(self) -> bool:
        return self._has_reservation

    @has_reservation.setter
    def has_reservation(self, has_reservation: bool) -> None:
        self._has_reservation = has_reservation

    def stopping_distance(self) -> float:
        raise NotImplementedError("TODO")

    def clone_for_request(self: Type[V]) -> V:
        """Return a clone of this vehicle to test a reservation request."""
        raise NotImplementedError("TODO")
        v = self()
        v.has_reservation = True
        return v

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


class AutomatedVehicle(Vehicle):
    """
    Since the default vehicle is fully automated, all this class does is change
    the name of the base class so it's clear what type of vehicle we're using.
    """

    def __init__(self, ):
        super.__init__(self)  # TODO: finish


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
