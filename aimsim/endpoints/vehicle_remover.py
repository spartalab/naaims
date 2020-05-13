'''
A vehicle spawner behaves like the former half of a lane, in that it accepts a
vehicle from an incoming lane using the `EnteringInterface`. Once it has the
vehicle, it removes the vehicle from the system, with the added option of
counting the vehicle before removal so the user can gauge the throughput of the
system.
'''

from typing import Iterable

from ..archetypes import Downstream
from ..vehicles import Vehicle


class VehicleRemover(Downstream):

    def __init__(self) -> None:
        """Create a new vehicle remover."""
        # TODO: check and return lane as well as vehicles?

        super().__init__()

    def process_transfers(self) -> Iterable[Vehicle]:
        """Process transfers by returning the vehicles to be removed."""
        to_return = [vt.vehicle for vt in self.entering_vehicle_buffer]
        super().process_transfers()
        return to_return
