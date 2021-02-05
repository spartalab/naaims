"""
The VehicleFactory module allows for the procedural generation of new vehicles
according to set patterns (e.g., a set of fixed vehicles, vehicles with random
dimensions and acceleration, etc.).
"""


from __future__ import annotations
from abc import abstractmethod
from typing import TYPE_CHECKING, Dict, Any, Type, List, Optional, Tuple
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
                 vehicle_types: List[Type[Vehicle]],
                 destinations: int,
                 vehicle_type_probabilities: Optional[List[float]] = None,
                 destination_probabilities: Optional[List[float]] = None,
                 source_node_id: Optional[int] = None) -> None:
        """Should create a new VehicleFactory.

        Parameters
            vehicle_types: List[Type[Vehicle]]
                List of potential vehicle types to spawn.
            destinations: int
                The total number of possible destinations.
            vehicle_type_probabilities: Optional[List[float]]
                The probability of spawning each of the provided vehicle_types.
                If not provided, assumes all types have an equal chance of
                spawning.
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

        assert len(vehicle_types) > 0

        # TODO: (heterogeneous vehicles) Change vehicle_types to take the same
        #       vehicle type but with multiple size and shape profiles.

        # Validate or infer vehicle type spawn probabilities
        if vehicle_type_probabilities is not None:
            if ((len(vehicle_types) != len(vehicle_type_probabilities))
                    or (sum(vehicle_type_probabilities) != 1.)):
                raise ValueError('If not all vehicle types have equal '
                                 'likelihood, each type must have a '
                                 'probability and they must sum to 1.')
            self.type_probs = vehicle_type_probabilities
        else:
            self.type_probs = [1/len(vehicle_types)]*len(vehicle_types)
        self.vehicle_types = vehicle_types

        # Validate or infer destination spawn probabilities
        if destination_probabilities is not None:
            if (destinations != len(destination_probabilities)) or \
                    (sum(destination_probabilities) != 1.):
                raise ValueError('Each destination must have a probability and'
                                 'they must sum to 1.')
            if (source_node_id is not None) and \
                    (destination_probabilities[source_node_id] > 0):
                raise ValueError('Supplied destination probabilities have '
                                 'nonzero chance of selecting the node this'
                                 'trip originates from.')
            self.destination_probabilities = destination_probabilities
        else:
            # Exclude source node ID from possible destinations if provided
            if source_node_id is not None:
                self.destination_probabilities = [1/(destinations-1)
                                                  ]*destinations
                self.destination_probabilities[source_node_id] = 0
            else:
                self.destination_probabilities = [1/destinations]*destinations
        self.destinations = destinations

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

    def _generate_type_and_dest(self) -> Tuple[Type[Vehicle], int]:
        """Randomly generate a valid vehicle type and destination."""
        return (choices(self.vehicle_types, weights=self.type_probs)[0],
                choices(list(range(self.destinations)),
                        weights=self.destination_probabilities)[0])
