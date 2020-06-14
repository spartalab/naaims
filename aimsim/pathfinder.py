from typing import overload, Dict, Any, Iterable, Tuple, List, Literal, Union

from .util import Coord
from .vehicles import Vehicle


class Pathfinder:

    def __init__(self,
                 road_specs: Iterable[Dict[str, Any]],
                 intersection_specs: Iterable[Dict[str, Any]],
                 spawner_specs: Iterable[Dict[str, Any]],
                 remover_specs: Iterable[Dict[str, Any]],
                 provided: Dict[Tuple[Coord, int],
                                List[Coord]] = {}) -> None:

        # Save predetermined pairs for use instead of a true routing
        # implementation.
        self.provided = provided

        # TODO: Read the specs and create a network with link costs to use with
        #       some shortest path algorithm. Note that this implementation
        #       will need to take into account turn restrictions.
        raise NotImplementedError("TODO")

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
        if pair in self.provided:
            return self.provided[pair]

        raise NotImplementedError("TODO")

    def lane(self, vehicle: Vehicle) -> Coord:
        """Return upstream Coord """

    # TODO: (low) Support link cost updates after each sim step.
    def update(self, inputs) -> None:
        """Update link costs for better pathing."""
        pass
