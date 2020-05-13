"""
A vehicle spawner behaves like the latter half of a road, in that it creates a
vehicle and uses the lane `LeavingInterface` to send the vehicle onto a lane
in the network.
"""

from typing import TYPE_CHECKING, Dict, Optional, Iterable
from __future__ import annotations

import aimsim.settings as SETTINGS
from ..archetypes import Upstream
from ..util import VehicleTransfer, DownstreamError, Coord
from ..vehicles import Vehicle
from ..roads import Road


class VehicleSpawner(Upstream):

    def __init__(self,
                 rates: Dict[Vehicle, float],
                 lane_coords: Iterable[Coord]
                 ):
        """Create a new vehicle spawner.

        Keyword arguments:
        rates -- Gives a Poisson arrival rate for each vehicle class
        """

        for rate in rates.values():
            if rate < 0:
                raise ValueError('Negative spawning rate encountered.')

        self.rates = rates

        # TODO: finish implementation. How to attach lanes? How to make the
        #       rates specify how often to spawn for certain destinations?

        raise NotImplementedError("TODO")

    def step(self) -> Optional[Vehicle]:
        """Decides if to spawn a vehicle. If so, processes and returns it."""

        if self.downstream is None:
            raise DownstreamError("No downstream object.")
        elif self.downstream is not Road:
            raise DownstreamError("Downstream object is not RoadLane.")

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
