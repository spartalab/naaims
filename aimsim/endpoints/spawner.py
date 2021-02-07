"""
A vehicle spawner creates vehicles using one or more random vehicle generators
and uses the Upstream interface to send the vehicle onto a lane in the road
it's connected to.
"""

from __future__ import annotations
from typing import TYPE_CHECKING, Dict, Optional, List, Any, Type, Tuple, Set
from random import random, choices, shuffle

import aimsim.shared as SHARED
from aimsim.archetypes import Configurable, Upstream
from aimsim.util import VehicleTransfer, MissingConnectionError, VehicleSection
from aimsim.endpoints.factories import (VehicleFactory,
                                        GaussianVehicleFactory)

if TYPE_CHECKING:
    from aimsim.vehicles import Vehicle
    from aimsim.road import Road, RoadLane


class VehicleSpawner(Configurable, Upstream):

    def __init__(self,
                 downstream: Road,
                 vpm: float,  # vehicles per minute
                 generator_ps: List[float],
                 generator_type: List[Type[VehicleFactory]],
                 generator_specs: List[Dict[str, Any]]
                 ) -> None:
        """Create a new vehicle spawner.

        Parameters
            downstream: Road
                The road to spawn vehicles on to.
            vpm: float
                Target number of vehicles to spawn per minute. Used in a
                Poisson distribution.
            generator_ps: List[float]
                The probability of using a specific VehicleFactory.
            generator_type: List[Type[VehicleFactory]]
                The types of generators to init.
            generator_specs: List[Dict[str, Any]]
                The specs of the generators to init.
        """

        if len(generator_type) != len(generator_specs) != len(generator_ps):
            raise ValueError("The number of generator types and specs must "
                             "match.")
        if sum(generator_ps) != 1:
            raise ValueError("The generator probabilities must sum to 1.")

        self.downstream = downstream

        # Given vehicles per minute and a Poisson process, calculate the
        # probability of spawning a vehicle in each timestep.
        # veh/min * min/sec * s
        self.p = (vpm/60) * SHARED.SETTINGS.TIMESTEP_LENGTH
        # Specifically, this is the probability of spawning at least one
        # vehicle in each timestep, but we assume that the probability of
        # spawning more than one vehicle is so low and difficulty of spawning
        # multiple vehicles at once that we combine the small probabilities for
        # spawns > 1 together with the probability of spawns == 1.

        # Record generator use probabilities and create the vehicle generators
        # from the given specifications.
        self.generator_ps: List[float] = generator_ps
        self.vehicle_factories: List[VehicleFactory] = []
        for i in range(len(generator_type)):
            self.vehicle_factories.append(
                generator_type[i].from_spec(generator_specs[i])
            )

        # Prepare a queued spawn to fill later.
        self.queued_spawn: Optional[Vehicle] = None
        self.queue: List[Tuple[Vehicle, List[RoadLane]]] = []

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a spawner spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: (spec) Interpret the string into the spec dict.
        raise NotImplementedError("TODO")

        # TODO: (spec) Enforce provision of separate lists of generator_type
        #       and generator_config in spawner spec string.
        generator_type_strs: List[str]
        generator_spec_strs: List[str]
        if len(generator_type_strs) != len(generator_spec_strs):
            raise ValueError("Number of VehicleFactory types and specs don't "
                             "match.")
        # Based on the spec, identify the correct generator types
        spec['generator_type'] = []
        spec['generator_specs'] = []
        for i in range(generator_type_strs):
            gen_str: str = generator_type_strs[i]
            gen_spec_str: str = generator_spec_strs[i]
            if gen_str.lower() in {'normal', 'normalgenerator'}:
                generator: VehicleFactory = GaussianVehicleFactory
                generator_spec: Dict[str, Any
                                     ] = GaussianVehicleFactory.str_from_spec(
                    gen_spec_str)
            else:
                raise ValueError("Unsupported VehicleFactory type.")
            spec['generator_type'].append(GaussianVehicleFactory)
            spec['generator_specs'].append()

        return spec

    @classmethod
    def from_spec(cls, spec: Dict[str, Any]) -> VehicleSpawner:
        """Create a new VehicleSpawner from the given spec.

        The output of spec_from_str needs to get the actual downstream Road
        after it's created.
        """
        return cls(
            downstream=spec['downstream'],
            vpm=spec['vpm'],
            generator_ps=spec['generator_ps'],
            generator_type=spec['generator_types'],
            generator_specs=spec['generator_specs']
        )

    def step_vehicles(self) -> Optional[Vehicle]:
        """Decides if to spawn a vehicle. If so, processes and returns it."""

        if self.downstream is None:
            raise MissingConnectionError("No downstream object.")
        elif self.downstream is not Road:
            raise MissingConnectionError("Downstream object is not a Road.")

        # Roll to spawn a new vehicle. If the roll is successful, pick a
        # generator to use based on the distribution of generators and use it
        # to spawn a vehicle.
        spawn = choices(self.vehicle_factories, self.generator_ps
                        )[0].create_vehicle() if (random() < self.p) else None

        # Find every downstream lane that this vehicle can enter and still
        # reach its destination. Add both to the queue.
        if spawn is not None:
            spawnable_lanes: List[RoadLane] = []
            for lane in self.downstream.lanes:
                if len(spawn.next_movements(lane.end_coord,
                                            at_least_one=False)) > 0:
                    spawnable_lanes.append(lane)
            # If we find that no lanes work, ever, error.
            if len(spawnable_lanes) == 0:
                raise RuntimeError("Spawned vehicle has no eligible lanes.")
            self.queue.append((spawn, spawnable_lanes))

        # Loop through queue to check for vehicles we can dispatch.
        blocked_lanes: Set[RoadLane] = set()
        for vehicle_to_transfer, eligible_lanes in self.queue:

            # Sort eligible lanes by those that have the fewest options, with
            # ties broken randomly so we don't systematically prefer a lane for
            # spawning, e.g., prefer through only lanes for vehicles that just
            # want to go forward, and leave multiuse lanes for turning vehicles
            # unless the through lanes are full.
            # TODO: edit eligible_lanes to prefer as above
            #       but is leaving the right lane empty always realistic?
            #       perhaps it should be some sort of probability distribution
            #       based on the relative number of movements
            shuffle(eligible_lanes)

            # Check if any of the eligible lanes have room for this vehicle.
            # If one of them does, transfer this vehicle onto that lane and
            # block it from accepting new transfers this timestep. If none of
            # them do, block all eligible lanes from accepting vehicles later
            # in the queue from spawning in those lanes so this queued vehicle
            # can enter one of them later.
            vehicle_can_transfer: bool = False
            for lane in eligible_lanes:
                if (lane not in blocked_lanes) and (lane.room_to_enter() > (
                    vehicle_to_transfer.length * 2 *
                    SHARED.SETTINGS.length_buffer_factor
                )):
                    vehicle_can_transfer = True

                    self.downstream.transfer_vehicle(VehicleTransfer(
                        vehicle=vehicle_to_transfer,
                        section=VehicleSection.FRONT,
                        d_left=None,
                        pos=lane.end_coord
                    ))
                    self.downstream.transfer_vehicle(VehicleTransfer(
                        vehicle=vehicle_to_transfer,
                        section=VehicleSection.CENTER,
                        d_left=None,
                        pos=lane.end_coord
                    ))
                    self.downstream.transfer_vehicle(VehicleTransfer(
                        vehicle=vehicle_to_transfer,
                        section=VehicleSection.REAR,
                        d_left=None,
                        pos=lane.end_coord
                    ))

                    blocked_lanes.add(lane)

            if not vehicle_can_transfer:
                blocked_lanes.update(eligible_lanes)

            # Quit if all lanes are blocked
            if len(blocked_lanes) == len(self.downstream.lanes):
                break

        # Pass newly spawned vehicle back to the Simulator if there is one
        return spawn
