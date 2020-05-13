"""
Archetypes are abstract classes (interfaces) that road objects implement in
order to ensure consistent communication between each other.
"""

from abc import ABC, abstractmethod
from typing import Optional, Any, Type, List, Iterable, NamedTuple, Union, Dict
from __future__ import annotations

from .util import VehicleTransfer, Coord
from .vehicles import Vehicle


class Configurable(ABC):

    @staticmethod
    @abstractmethod
    def create_from_spec(self, spec) -> Configurable:
        """Should create an instance of itself from a (portion of a) file."""
        # TODO: will spec be a filename, JSON, or string?
        raise NotImplementedError("Must be implemented in child classes.")


class Facility(ABC):
    @abstractmethod
    def update_speeds(self) -> None:
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
    def __init__(self):
        """Init a Downstream property."""
        self.downstream: Optional[Union[
            Dict[Coord, Downstream], Downstream
        ]] = None

    @abstractmethod
    def unconnected_downstreams(self) -> Optional[Iterable[Coord]]:
        """Should return unconnected downstream coordinates, if any."""
        raise NotImplementedError("Must be implemented in child class.")

    @abstractmethod
    def connect_downstreams(self, upstreams: Iterable[Upstream]) -> None:
        """Finalize connecting downstream object(s)."""
        raise NotImplementedError("Must be implemented in child class.")

    @abstractmethod
    def step(self) -> Optional[Vehicle]:
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
        self.upstream: Optional[Union[
            Dict[Coord, Downstream], Downstream
        ]] = None

    @abstractmethod
    def unconnected_upstreams(self) -> Optional[Iterable[Coord]]:
        """Should return unconnected upstream coordinates, if any."""
        raise NotImplementedError("Must be implemented in child class.")

    @abstractmethod
    def connect_upstreams(self, upstreams: Iterable[Downstream]) -> None:
        """Finalize connecting upstream object(s)."""
        raise NotImplementedError("Must be implemented in child class.")

    def transfer_vehicle(self, transfer: VehicleTransfer) -> None:
        self.entering_vehicle_buffer.append(transfer)

    @abstractmethod
    def process_transfers(self) -> Optional[Iterable[Vehicle]]:
        """Should incorporate new vehicles into the Downstream object.

        Returns nothing except in the case of a VehicleRemover, which could
        return one or more Vehicle pointers if gets vehicles to remove.
        """
        self.entering_vehicle_buffer = []
        return None
