'''
A vehicle spawner behaves like the former half of a lane, in that it accepts a
vehicle from an incoming lane using the `EnteringInterface`. Once it has the
vehicle, it removes the vehicle from the system, with the added option of
counting the vehicle before removal so the user can gauge the throughput of the
system.
'''

from typing import Iterable, Dict, Any
from __future__ import annotations

from ..archetypes import Configurable, Downstream
from ..roads import Road
from ..vehicles import Vehicle


class VehicleRemover(Configurable, Downstream):

    def __init__(self, upstream: Road) -> None:
        """Create a new vehicle remover."""
        # TODO: check and return lane as well as vehicles?
        self.upstream = upstream

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a remover spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: Find the remover's id and upstream road_id and place into spec.
        raise NotImplementedError("TODO")

        return spec

    @classmethod
    def from_spec(cls, spec: Dict[str, Any]) -> VehicleRemover:
        """Create a new VehicleRemover from the given spec.

        The output of spec_from_str needs to get the actual upstream Road
        after it's created.
        """
        return cls(
            upstream=spec['upstream']
        )

    def process_transfers(self) -> Iterable[Vehicle]:
        """Process transfers by returning the vehicles to be removed."""
        to_return = [vt.vehicle for vt in self.entering_vehicle_buffer]
        super().process_transfers()
        return to_return
