"""
A vehicle spawner behaves like the latter half of a road, in that it creates a
vehicle and uses the lane `LeavingInterface` to send the vehicle onto a lane
in the network.
"""

from typing import TYPE_CHECKING, Dict, Optional, Iterable, Any
from __future__ import annotations

import aimsim.settings as SETTINGS
from ..archetypes import Configurable, Upstream
from ..util import VehicleTransfer, LinkError, Coord
from ..vehicles import Vehicle
from ..roads import Road


class VehicleSpawner(Configurable, Upstream):

    def __init__(self,
                 downstream: Road,
                 rates: Dict[Vehicle, float]
                 ):
        """Create a new vehicle spawner.

        Keyword arguments:
        rates -- Gives a Poisson arrival rate for each vehicle class
        """

        for rate in rates.values():
            if rate < 0:
                raise ValueError('Negative spawning rate encountered.')

        self.downstream = downstream
        self.rates = rates

        # TODO: finish implementation. How to attach lanes? How to make the
        #       rates specify how often to spawn for certain destinations?

        raise NotImplementedError("TODO")

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a remover spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: interpret the string into the spec dict
        raise NotImplementedError("TODO")

        return spec

    @classmethod
    def from_spec(cls, spec: Dict[str, Any]) -> VehicleSpawner:
        """Create a new VehicleSpawner from the given spec.

        The output of spec_from_str needs to get the actual downstream Road
        after it's created.
        """
        return cls(
            downstream=spec['downstream'],
            rates=spec['rates']
        )

    def step(self) -> Optional[Vehicle]:
        """Decides if to spawn a vehicle. If so, processes and returns it."""

        if self.downstream is None:
            raise LinkError("No downstream object.")
        elif self.downstream is not Road:
            raise LinkError("Downstream object is not RoadLane.")

        # roll a number and compare it to rates to see if and which vehicle(s?)
        # should be spawned in this timestep.
        raise NotImplementedError("TODO")

        # do some logic to determine what lane to spawn it in and where its
        # destination is

        # check if the lane it's trying to spawn into has enough space. it
        # needs to have, at the end of the lane, free space greater than or
        # equal to half the length of the vehicle spawning.

        # if there's room, create a new vehicle object and assign it the latest
        # vin
        # TODO: not thread-safe. must be changed for multiprocessing.
        vin = SETTINGS.vin_counter
        SETTINGS.vin_counter += 1

        # separately put the front and center of the vehicle into the road's
        # buffer
        # TODO: this is tricky, update the API for transferring first then
        #       accomodate.
        # self.downstream.transfer_vehicle()

        # TODO: hold onto the last vehicle spawned until it fully enters the
        #       upcoming road. then and only then start checking if there's
        #       room to spawn the next vehicle.
        #       vehicles spawn at 0 velocity so collisions not an issue this
        #       way

        # return the spawned vehicle, if there is one
