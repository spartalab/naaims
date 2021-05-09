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
from aimsim.road import Road

if TYPE_CHECKING:
    from aimsim.vehicles import Vehicle
    from aimsim.road import RoadLane


class VehicleSpawner(Configurable, Upstream):

    def __init__(self,
                 downstream: Road,
                 vpm: float,  # vehicles per minute
                 factory_types: List[Type[VehicleFactory]],
                 factory_specs: List[Dict[str, Any]],
                 factory_selection_probabilities: List[float],
                 fixed_interval_spawns: List[int] = []
                 ) -> None:
        """Create a new vehicle spawner.

        Parameters
            downstream: Road
                The road to spawn vehicles on to.
            vpm: float
                Target number of vehicles to spawn per minute. Used in a
                Poisson distribution.
            factory_types: List[Type[VehicleFactory]]
                The types of vehicle factories to create.
            factory_specs: List[Dict[str, Any]]
                The specs of the vehicle factories to create.
            factory_selection_probabilities: List[float]
                The probability of using a specific VehicleFactory.
            fixed_interval_spawns: List[int]
                Override random behavior and spawn a vehicle at this timestep.
        """

        if len(factory_types) != len(factory_specs) != \
                len(factory_selection_probabilities):
            raise ValueError("The number of generator types, specs, and "
                             "probabilities must match.")
        if sum(factory_selection_probabilities) != 1:
            raise ValueError("The generator probabilities must sum to 1.")

        self.downstream = downstream

        # Given vehicles per minute and a Poisson process, calculate the
        # probability of spawning a vehicle in each timestep.
        # veh/min *  1min/60s * s/timestep
        self.spawn_probability = (vpm/60) * SHARED.SETTINGS.TIMESTEP_LENGTH
        # Specifically, this is the probability of spawning at least one
        # vehicle in each timestep, but we assume that the probability of
        # spawning more than one vehicle is so low and difficulty of spawning
        # multiple vehicles at once that we combine the small probabilities for
        # spawns > 1 together with the probability of spawns == 1.

        # Record generator use probabilities and create the vehicle generators
        # from the given specifications.
        self.factory_selection_probabilities = factory_selection_probabilities
        self.factories: List[VehicleFactory] = []
        for i in range(len(factory_types)):
            self.factories.append(factory_types[i].from_spec(factory_specs[i]))

        self.fixed_interval_spawns = fixed_interval_spawns

        # Prepare a queued spawn to fill later.
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
            factory_selection_probabilities=spec[
                'factory_selection_probabilities'],
            factory_types=spec['factory_types'],
            factory_specs=spec['factory_specs']
        )

    def step_vehicles(self) -> Tuple[Optional[Vehicle], List[Vehicle]]:
        """Decides whether to spawn and returns spawned and entering vehicles.

        Returns
            Optional[Vehicle]   The vehicle spawned in this timestep, if there
                                was one.
            List[Vehicle]       The vehicle(s) that exit the spawner and enter
                                the simulated scope. Usually the same as the
                                spawned vehicle but may be different if road
                                traffic prevents vehicles spawned in prior
                                timesteps from entering at their time of spawn.
        """

        if self.downstream is None:
            raise MissingConnectionError("No downstream object.")
        elif type(self.downstream) is not Road:
            raise MissingConnectionError("Downstream object is not a Road.")

        # Roll to spawn a new vehicle (or spawn one anyway if the fixed
        # interval spawn says to). If the roll is successful, pick a generator
        # to use based on the distribution of generators and use it to spawn a
        # new vehicle.
        spawn: Optional[Vehicle]
        if (len(self.fixed_interval_spawns) > 0) and \
                (self.fixed_interval_spawns[0] == SHARED.t):
            spawn = choices(self.factories,
                            self.factory_selection_probabilities
                            )[0].create_vehicle()
            self.fixed_interval_spawns.pop(0)
        elif random() < self.spawn_probability:
            spawn = choices(self.factories,
                            self.factory_selection_probabilities
                            )[0].create_vehicle()
        else:
            spawn = None

        # Find every downstream lane that this vehicle can enter and still
        # reach its destination. Add both to the queue.
        if spawn is not None:
            spawnable_lanes: List[RoadLane] = []
            for lane in self.downstream.lanes:
                if len(spawn.next_movements(lane.trajectory.end_coord,
                                            at_least_one=False)) > 0:
                    spawnable_lanes.append(lane)
            # If we find that no lanes work, ever, error.
            if len(spawnable_lanes) == 0:
                raise RuntimeError("Spawned vehicle has no eligible lanes.")
            self.queue.append((spawn, spawnable_lanes))

        # Loop through queue to check for vehicles we can dispatch.
        blocked_lanes: Set[RoadLane] = set()
        vehicles_transferred: List[Vehicle] = []
        queue_entries_to_delete: List[int] = []
        for i, (vehicle_to_transfer, eligible_lanes) in enumerate(self.queue):

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
                    vehicle_to_transfer.length *
                    (1 + 2 * SHARED.SETTINGS.length_buffer_factor)
                )):
                    vehicle_can_transfer = True

                    self.downstream.transfer_vehicle(VehicleTransfer(
                        vehicle=vehicle_to_transfer,
                        section=VehicleSection.FRONT,
                        distance_left=None,
                        pos=lane.trajectory.start_coord
                    ))
                    self.downstream.transfer_vehicle(VehicleTransfer(
                        vehicle=vehicle_to_transfer,
                        section=VehicleSection.CENTER,
                        distance_left=None,
                        pos=lane.trajectory.start_coord
                    ))
                    self.downstream.transfer_vehicle(VehicleTransfer(
                        vehicle=vehicle_to_transfer,
                        section=VehicleSection.REAR,
                        distance_left=None,
                        pos=lane.trajectory.start_coord
                    ))

                    blocked_lanes.add(lane)
                    vehicles_transferred.append(vehicle_to_transfer)
                    queue_entries_to_delete.append(i)
                    break

            if not vehicle_can_transfer:
                blocked_lanes.update(eligible_lanes)

            # Quit if all lanes are blocked
            if len(blocked_lanes) == len(self.downstream.lanes):
                break

        # Delete transferred vehicles from queue
        for i in queue_entries_to_delete:
            del self.queue[i]

        # Pass newly spawned vehicle back to the Simulator if there is one
        return spawn, vehicles_transferred
