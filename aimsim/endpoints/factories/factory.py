"""
The VehicleFactory module allows for the procedural generation of new vehicles
according to set patterns (e.g., a set of fixed vehicles, vehicles with random
dimensions and acceleration, etc.).
"""


from __future__ import annotations
from abc import abstractmethod
from typing import TypeVar, Dict, Any, Type, List, Optional, Tuple
from random import choices, gauss

import aimsim.shared as SHARED
from aimsim.archetypes import Configurable
from aimsim.vehicles import Vehicle

F = TypeVar('F', bound='VehicleFactory')


class VehicleFactory(Configurable):
    """
    Generate new vehicles with parameters determined by the implementation of
    child vehicle factories.
    """

    @abstractmethod
    def __init__(self,
                 vehicle_types: List[Type[Vehicle]],
                 destinations: int,
                 type_probs: Optional[List[float]] = None,
                 d_probs: Optional[List[float]] = None,
                 pair_id: Optional[int] = None) -> None:
        """Should create a new VehicleFactory."""

        if type_probs is not None:
            if ((len(vehicle_types) != len(type_probs))
                    or (sum(type_probs) != 1.)):
                raise ValueError('If not all vehicle types have equal '
                                 'likelihood, each type must have a '
                                 'probability and they must sum to 1.')
            self.type_probs = type_probs
        else:
            self.type_probs = [1/len(vehicle_types)]*len(vehicle_types)
        self.vehicle_types = vehicle_types

        if d_probs is not None:
            if (destinations != len(d_probs)) or (sum(d_probs) != 1.):
                raise ValueError('If not all destinations have equal '
                                 'likelihood, each destination must have a '
                                 'probability and they must sum to 1.')
            self.d_probs = d_probs
        else:
            if pair_id is not None:
                # Spawners and removers are created in pairs (e.g., all
                # two-lane roads). Prevent vehicles from trying to u-turn.
                self.d_probs = [1/(destinations-1)]*destinations
                self.d_probs[pair_id] = 0
            else:
                self.d_probs = [1/destinations]*destinations
        self.destinations = destinations

    @staticmethod
    @abstractmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        raise NotImplementedError("Must be implemented in child classes.")

    @classmethod
    @abstractmethod
    def from_spec(cls: Type[F], spec: Dict[str, Any]) -> F:
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
                        weights=self.d_probs)[0])
