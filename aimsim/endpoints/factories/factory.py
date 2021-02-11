"""
The VehicleFactory module allows for the procedural generation of new vehicles
according to set patterns (e.g., a set of fixed vehicles, vehicles with random
dimensions and acceleration, etc.).
"""


from __future__ import annotations
from abc import abstractmethod
from typing import TYPE_CHECKING, Dict, Any, Type, List, Optional
from random import choices

import aimsim.shared as SHARED
from aimsim.archetypes import Configurable

if TYPE_CHECKING:
    from aimsim.vehicles import Vehicle


class VehicleFactory(Configurable):
    """
    Generate new vehicles with parameters determined by the implementation of
    child vehicle factories.
    """

    @abstractmethod
    def __init__(self,
                 vehicle_type: Type[Vehicle],
                 num_destinations: int,
                 destination_probabilities: Optional[List[float]] = None,
                 source_node_id: Optional[int] = None) -> None:
        """Should create a new VehicleFactory.

        Parameters
            vehicle_type: Type[Vehicle]
                Type of vehicle spawned by this factory.
            num_destinations: int
                The total number of possible destinations.
            destination_probabilities: Optional[List[float]]
                The probability of spawning a vehicle directed to each
                destination. If not provided, assumes all destinations have an
                equal chance of being chosen. (If source_node_id is provided,
                it's excluded.)
            source_node_id: Optional[int]
                Spawners and removers are often created in pairs (e.g., all
                two-lane roads). If they are, this parameter can be used to
                prevent vehicles from spawning with a destination the same as
                this node that they're starting from.
        """

        self.vehicle_type = vehicle_type

        # Validate or infer destination spawn probabilities
        if destination_probabilities is not None:
            if (num_destinations != len(destination_probabilities)) or \
                    (sum(destination_probabilities) != 1.):
                raise ValueError('Each destination must have a probability '
                                 'and they must sum to 1.')
            if (source_node_id is not None) and \
                    (destination_probabilities[source_node_id] > 0):
                raise ValueError('Supplied destination probabilities have '
                                 'nonzero chance of selecting the node this '
                                 'trip originates from.')
            self.destination_probabilities = destination_probabilities
        else:
            # Exclude source node ID from possible destinations if provided
            if source_node_id is not None:
                self.destination_probabilities = [1/(num_destinations-1)
                                                  ]*num_destinations
                self.destination_probabilities[source_node_id] = 0
            else:
                self.destination_probabilities = [
                    1/num_destinations]*num_destinations
        self.destinations = num_destinations

    @staticmethod
    @abstractmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        raise NotImplementedError("Must be implemented in child classes.")

    @classmethod
    @abstractmethod
    def from_spec(cls, spec: Dict[str, Any]) -> VehicleFactory:
        raise NotImplementedError("Must be implemented in child classes.")

    @abstractmethod
    def create_vehicle(self) -> Vehicle:
        """Should create a new vehicle."""
        raise NotImplementedError("Must be implemented in child classes.")

    def _assign_new_vin(self) -> int:
        """Return the latest VIN for assignment to a newly created vehicle."""
        # TODO: not thread-safe. must be changed for multiprocessing.
        vin = SHARED.vin_counter
        SHARED.vin_counter += 1
        return vin

    def _pick_vehicle_type(self) -> int:
        """Randomly choose a destination."""
        return choices(list(range(self.destinations)),
                       weights=self.destination_probabilities)[0]
