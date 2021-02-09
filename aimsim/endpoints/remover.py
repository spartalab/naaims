'''
A vehicle remover accepts vehicles using the Downstream interface and removes
them from the road network, passing them back to the simulator for final
processing and record-keeping.
'''

from __future__ import annotations
from typing import List, Dict, Any

from aimsim.archetypes import Configurable, Downstream
from aimsim.util import VehicleSection
from aimsim.road import Road
from aimsim.vehicles import Vehicle


class VehicleRemover(Configurable, Downstream):

    def __init__(self, upstream: Road) -> None:
        """Create a new vehicle remover."""
        self.upstream = upstream
        super().__init__()

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

    def process_transfers(self) -> List[Vehicle]:
        """Process transfers by returning the vehicles to be removed.

        We remove vehicles as soon as their center section exits since
        otherwise they'd need their position updated, which we can't do inside
        the vehicle remover.
        """

        # Filter vehicle transfers for center section exits.
        exited: List[Vehicle] = []
        for transfer in self.entering_vehicle_buffer:
            if transfer.section is VehicleSection.CENTER:
                exited.append(transfer.vehicle)
                # TODO: (multiple) Return whether the vehicle succeeded in
                #       making it to its intended destination.

        # Clear the buffer.
        super().process_transfers()

        return exited
