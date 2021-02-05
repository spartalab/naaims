from __future__ import annotations
from typing import TYPE_CHECKING, Dict, Iterable, Tuple, List, Optional

if TYPE_CHECKING:
    from aimsim.util import Coord
    from aimsim.vehicles import Vehicle
    from aimsim.road import Road
    from aimsim.intersection import Intersection


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
            lane_destination_pairs:
                Optional[Dict[Tuple[Coord, int], List[Coord]]] = None
                If provided, overrides default behavior by providing hardcoded
                paths through the intersection network, obviating the need for
                shortest path calculations.
        """

        # Save predetermined pairs for use instead of a true routing
        # implementation.
        self.provided = provided

        # TODO: (low) Use the connectivity of each road to build a network that
        #       we can use with shortest path algorithms.

    def next_movements(self, coord: Coord, destination: int, at_least_one: bool
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
        For use in case of non-fully-connected networks since all modules using
        this method (except VehicleSpawner) needs a target in order to work.
        """

        # Bypass full routing check if the requested source-destination pair
        # has already been provided.
        pair = (coord, destination)
        if (self.provided is not None) and (pair in self.provided):
            return self.provided[pair]

        raise NotImplementedError("TODO")

    def lane(self, vehicle: Vehicle) -> Coord:
        """Return upstream Coord """

    # TODO: (low) Support link cost updates after each sim step.
    def update(self, inputs: Optional[Dict[int, float]]) -> None:
        """Update link costs for better pathing."""
        pass
