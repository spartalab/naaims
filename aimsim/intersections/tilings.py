"""
The intersection manager determines how AIM handles the intersection conflict
area (often called the "box", at least for 4-way 90 degree intersections). This
includes managing when vehicles are allowed to enter an intersection as well as
how they proceed through the intersection. (For automated vehicles, this is an
accurate reflection of how they would be controlled using AIM, while for human-
driven vehicles, this module handles both what an AIM manager would do in that
scenario as well as simulates the imprecise movement of human-driven vehicles.)

The two primary methods are arc-based (ArcAIM), where vehicles reserve the area
around a conflict point (the intersection of two trajectories), and tile-based,
where the entire intersection is divided into tiles that can be reserved
(Dresner and Stone 2008).
"""


from abc import abstractmethod
from typing import (Optional, Union, Iterable, Set, Deque, NamedTuple, Dict,
                    Tuple, Type, TypeVar, Any)
from collections import deque

from ..archetypes import Configurable, Upstream, Downstream
from ..util import Coord
from ..lanes import IntersectionLane
from ..vehicles import Vehicle
from ..roads import Road
from .reservations import ReservationRequest, Reservation
from .tiles import Tile, DeterministicTile

T = TypeVar('T', bound='Tiling')


class Tiling(Configurable):

    @abstractmethod
    def __init__(self,
                 upstream_road_by_coord: Dict[Coord, Road],
                 downstream_road_by_coord: Dict[Coord, Road],
                 lanes: Dict[Coord, IntersectionLane],
                 tile_type: Type[Tile] = Tile,
                 granularity: Optional[int] = None,
                 cycle: Optional[
                     Iterable[
                         Tuple[
                             Iterable[IntersectionLane], int
                         ]
                     ]
                 ] = None
                 ) -> None:
        """Should instantiate a new Tiling.

        Takes the lanes from the Intersection and figures out how to tile them.

        `cycle`, if given, takes an list of (intersection lane lists and ints).
        For each tuple in this list, the first item describes which lanes
        should have their associated tiles marked, for the amount of timesteps
        described by the second item, an int.
        """
        # TODO: figure out cycle typing, and if it should be moved up to
        #       manager

        self.upstream_road_by_coord = upstream_road_by_coord
        self.downstream_road_by_coord = downstream_road_by_coord
        self.lanes = lanes
        # only save granularity in SquareTiling
        self.cycle = cycle

        # initiate reservations dict
        self.reservations: Dict[Vehicle, Reservation] = {}

        # Child tilings should call this for initial setup, then continue in
        # their own init to set up whatever they need to.

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a tiling spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: interpret the string into the spec dict
        raise NotImplementedError("TODO")

        # TODO: enforce provision of tile_type field in manager spec string

        # Based on the spec, identify the correct tiling type
        if tile_type.lower() in {'', 'stochastic', 'stochastictile'}:
            spec['tile_type'] = Tile
        elif tile_type.lower() in {'deterministic', 'deterministictile'}:
            spec['tile_type'] = DeterministicTile
        else:
            raise ValueError("Unsupported Tile type.")

        return spec

    @classmethod
    def from_spec(cls: Type[T], spec: Dict[str, Any]) -> T:
        """Should interpret a spec dict to call the manager's init."""
        return cls(
            upstream_road_by_coord=spec['upstream_road_by_coord'],
            downstream_road_by_coord=spec['downstream_road_by_coord'],
            lanes=spec['lanes'],
            tile_type=spec['tile_type'],
            granularity=spec['granularity'],
            cycle=spec['cycle']
        )

    # Begin simulation cycle methods

    @abstractmethod
    def reserve_lanes(self, lanes: Iterable[IntersectionLane],
                      start: int, end: int) -> None:
        """Reserve all tiles associated with these lanes from start to end.

        Reserve all tiles associated with the provided intersection lanes from
        the timestep `start` steps away from now, to the timestep `end` steps
        away from now. (This will primarily be used for FCFS-Signals.)

        If the tile is already 
        """
        pass

    @abstractmethod
    def check_request(self,
                      requests: ReservationRequest,
                      delay: int = 0,
                      mark: bool = False
                      ) -> Optional[Reservation]:
        """If a request can work, return a PotentialReservation.

        Check if a reservation can work. If so, return a PotentialReservation
        and mark the tiles with this potential reservation (for checking for
        conflicts with contemporary requests) if the mark parameter is true.

        (Deterministic will reject if tiles are marked at all, while stochastic
        will reject if the variable point value used by this res's tile exceeds
        some threshold point value.)

        Parameters
            request: Iterable[ReservationRequest]
                One or more requests that must be collectively checked for
                validity.
            confirm: bool
                If true, actually make the reservation. Otherwise, don't.
            delay: int
                How many steps from the current timestep to start checking this
                reservation request.
            mark: bool
                Whether or not to mark tiles to check for potential conflicts
                _between_ potential reservations. Only necessary for managers
                approving multiple requests at once, such as auctions.

        """

        # check every tile the reservation request touches

        # As each tile is checked, save a dict of the tile to the proportion of
        # the tile the request wants to use. Dump this dict and reject the
        # request on the first tile that rejects this request.

        # If they all work, turn the request into a potential reservation using
        # the dict of tiles to proportions and log it onto all of the tiles.
        pass

    @abstractmethod
    def confirm_reservation(self, reservation: Reservation
                            ) -> Union[bool, Iterable[float]]:
        """Confirm a potential reservation and return the acceleration profile.

        Confirm a potential reservation and clear any attached potential
        reservations attached to the used tiles.

        Return types
            True                Accelerate to speed limit.
            False               Stay at constant speed.
            Iterable[float]     Follow a specific acceleration profile.
        """

        # TODO: bank reservation and keep track of it as it moves through the
        #       intersection

        reservation.request.vehicle.has_reservation = True
        raise NotImplementedError("TODO")

    @abstractmethod
    def find_best_batch(self,
                        requests: Iterable[Iterable[Reservation]]
                        ) -> Iterable[Reservation]:
        """Take potential reservation permutations and find the best batch.

        Take a list of potential reservations sequence permuatations, where the
        permutations are of one lane's reservations (e.g., first vehicle in a
        lane, first and second, first through third). (This allows us to start
        looking for compatible sequences with each lane's longest sequence
        and work our way down.)

        Run through these reservations and return the highest value compatible
        subset of reservations by sum total VOT.
        """

        # sort all permutations so that it's ordered longest to shortest
        # start comparisons with the longest permutations. if you can't find
        # compatibility, go down to the next longest permutation until there's
        # none left.

        raise NotImplementedError("TODO lower priority")

    @abstractmethod
    def clear_potential_reservations(self):
        """Go through all tiles and empty out all potential reservations."""
        raise NotImplementedError("Should be implemented in child classes.")

    def reservation_active(self, vehicle: Vehicle):
        """Tell tiling this reservation is now active."""
        raise NotImplementedError("TODO")

    def reservation_complete(self, vehicle: Vehicle):
        """Tell tiling this reservation is complete."""
        raise NotImplementedError("TODO")

    @abstractmethod
    def update_active_reservations(self) -> None:
        """After vehicles inside move, revise their reserved tiles.

        Only necessary for stochastic reservations, but it might be nice for
        deterministic methods with nonzero buffers.
        """
        pass

    @abstractmethod
    def step(self):
        """Prepare for the next timestep (remove last step's tiles).

        Don't forget to add logic to support pre-reserved cycles.
        """
        raise NotImplementedError("Should be implemented in child classes.")


class ArcTiling(Tiling):

    # TODO: points at the edge of the intersection should be considered a
    #       conflict object even if they aren't literally a conflict point,
    #       so we can enforce spacing between vehicles.

    def __init__(self,
                 upstream_road_by_coord: Dict[Coord, Road],
                 downstream_road_by_coord: Dict[Coord, Road],
                 lanes: Dict[Coord, IntersectionLane],
                 tile_type: Type[Tile] = Tile,
                 granularity: Optional[int] = None,
                 cycle: Optional[
                     Iterable[
                         Tuple[
                             Iterable[IntersectionLane], int
                         ]
                     ]
                 ] = None
                 ) -> None:
        # TODO: does arctiling need a buffer argument? anything else?
        super().__init__(
            upstream_road_by_coord=upstream_road_by_coord,
            downstream_road_by_coord=downstream_road_by_coord,
            lanes=lanes,
            tile_type=tile_type,
            granularity=granularity,
            cycle=cycle
        )
        raise NotImplementedError("TODO")

    # TODO: old code, fix or replace
        # # TODO: figure out how to find lane conflicts
        # # find conflicts
        # for l1, l2 in itertools.combinations(self.intersection_lanes, 2):
        #     t1, t2, point = l1.trajectory.get_intersection_point(l2.trajectory)
        #     if point == None:
        #         continue
        #     angle = l1.trajectory.get_intersection_angle(l2.trajectory, t1, t2)
        #     self.conflicts.add(ConflictRegion(l1, l2, point, angle, t1, t2))

    # class ConflictRegion:

    #     def __init__(self, traj1, traj2, point, angle, t1, t2):
    #         self.point = point
    #         self.angle = angle
    #         self.traj1 = traj1
    #         self.traj2 = traj2
    #         self.t1 = t1
    #         self.t2 = t2

    #     def get_conflict_region(self, vehicle):
    #         pass


class SquareTiling(Tiling):
    pass
