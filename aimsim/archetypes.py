"""
Archetypes are abstract classes (interfaces) that road objects implement in
order to ensure consistent communication between each other.

These archetypes enforce the main simulation loop:
    1. update_speeds (Facilities)
    2. step (Upstreams)
    3. process_transfers (Downstreams)
    4. handle_logic (Facilities)
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Optional, Any, List, Iterable, Union, Dict

from .util import VehicleTransfer, Coord, SpeedUpdate
from .vehicles import Vehicle


class Configurable(ABC):

    @staticmethod
    @abstractmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Should interpret a string representation into an usable dict."""
        raise NotImplementedError("Must be implemented in child class.")

    @classmethod
    @abstractmethod
    def from_spec(cls, spec: Dict[str, Any]) -> Configurable:
        """Should interpret a spec dict to call the Configurable's init.

        NOTE: most Configurables' specs need additional processing between
              creation in spec_from_str and ingestion here.
        """
        # TODO: will spec be a filename, JSON, or string?
        raise NotImplementedError("Must be implemented in child classes.")


class Facility(ABC):
    @abstractmethod
    def update_speeds(self) -> Dict[Vehicle, SpeedUpdate]:
        """
        For roads and intersections, goes through and calculates the speed and
        acceleration of its responsible vehicles. If a vehicle is in any part
        inside an intersection, the intersection calculates it, otherwise
        the road is responsible for it.
        """
        raise NotImplementedError("Must be implemented in child classes.")

    @abstractmethod
    def handle_logic(self) -> None:
        """Should handle all the class-unique stuff like scheduling."""
        raise NotImplementedError('Must be implemented in child classes.')


class Upstream(ABC):
    """
    If an object is "upstream" of other objects, it sends vehicles to an
    object downstream once they go past this upstream object.
    """

    @abstractmethod
    def step_vehicles(self) -> Optional[Vehicle]:
        """Should progress all vehicles over one timestep.

        Returns nothing except in the case of a VehicleSpawner, which could
        return a Vehicle pointer if it decides to spawn one.
        """
        raise NotImplementedError('Must be implemented in child class.')


class Downstream(ABC):
    """
    If an object is "downstream" of other objects, it receives vehicles from an
    object upstream that it must process.
    """

    @abstractmethod
    def __init__(self) -> None:
        """Init an empty transfer buffer and an upstream property."""
        self.entering_vehicle_buffer: List[VehicleTransfer] = []

    def transfer_vehicle(self, transfer: VehicleTransfer) -> None:
        """Called by Upstream objects to transfer a vehicle to this object."""
        self.entering_vehicle_buffer.append(transfer)

    @abstractmethod
    def process_transfers(self) -> Optional[Iterable[Vehicle]]:
        """Should incorporate new vehicles into the Downstream object.

        Returns nothing except in the case of a VehicleRemover, which could
        return one or more Vehicle pointers if gets vehicles to remove.
        """
        self.entering_vehicle_buffer = []
        return None
