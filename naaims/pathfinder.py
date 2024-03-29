from __future__ import annotations
from typing import TYPE_CHECKING, Dict, Iterable, Tuple, List, Optional
from warnings import warn

if TYPE_CHECKING:
    from naaims.util import Coord
    # from naaims.vehicles import Vehicle
    from naaims.road import Road
    from naaims.intersection import Intersection


class Pathfinder:
    """
    Given a lane's ending Coord, find a vehicle's shortest path to its
    destination, including a fallback if there is no path to its destination.
    """

    def __init__(self,
                 roads: Iterable[Road],
                 intersections: Iterable[Intersection],
                 provided: Optional[Dict[Tuple[Coord, int],
                                         List[Coord]]] = {}) -> None:
        """Create a new Pathfinder instance.

        Parameters
            roads: Iterable[Road]
            intersections: Iterable[Intersection]
                The roads and intersections in the road network.
            provided: Optional[Dict[Tuple[Coord, int], List[Coord]]] = None
                If provided, overrides default behavior by providing hardcoded
                paths through the intersection network, obviating the need for
                shortest path calculations. See next_movements for further
                explanation of the argument structure.
        """

        # Save predetermined pairs for use instead of a true routing
        # implementation.
        self.provided = provided

        # TODO: (low) Use the connectivity of each road to build a network that
        #       we can use with shortest path algorithms and infer destinations

    def next_movements(self, enters_intersection_at: Coord,
                       destination: int, at_least_one: bool = False
                       ) -> List[Coord]:
        """Return a set of valid Coords from which to exit.

        Given the Coord at which a vehicle is entering an intersection (the
        end Coord of the RoadLane it's exiting from) and the ID of its
        destination, calculate a vehicle's shortest path to its destination
        and return the Coords of intersection exit lanes that will put it on
        its shortest path.

        The first Coord returned should be the ideal exit trajectory (e.g., the
        one requiring the fewest lane changes), in case the intersection
        manager doesn't want to explore multiple options.

        If at_least_one, return at least one Coord that's a valid turning
        movement, even if it can't possibly reach the specified destination.
        (This can be chosen randomly or be the quickest way out of the system.)
        For use in case of non-fully-connected networks since all modules using
        this method (except VehicleSpawner) needs a target in order to work.
        """

        # Bypass full routing check if the requested source-destination pair
        # has already been provided.
        if self.provided is not None:
            pair = (enters_intersection_at, destination)
            if not at_least_one:
                return self.provided.get(pair, [])
            elif pair in self.provided:
                return self.provided[pair]

        # TODO: Inferred destinations
        raise NotImplementedError("This source destination pair was not "
                                  "pre-calculated/provided and inferred "
                                  "destinations haven't been implemented yet.")

    # TODO: ? Why did I think I needed this?
    # def lane(self, vehicle: Vehicle) -> Coord:
    #     """Return upstream Coord """
    #     raise NotImplementedError("TODO")

    # TODO: (low) Support link cost updates after each sim step.
    def update(self, inputs: Optional[Dict[int, float]]) -> None:
        """Update link costs for better pathing."""
        pass
