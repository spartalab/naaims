"""
A vehicle spawner creates vehicles using one or more random vehicle generators
and uses the Upstream interface to send the vehicle onto a lane in the road
it's connected to.
"""

from __future__ import annotations
from typing import TYPE_CHECKING, Dict, List, Any, Type, Tuple, Set
from random import random, choices, shuffle
from math import floor

import naaims.shared as SHARED
from naaims.archetypes import Configurable, Upstream
from naaims.util import VehicleTransfer, MissingConnectionError, VehicleSection
from naaims.endpoints.factories import (VehicleFactory,
                                        GaussianVehicleFactory)

if TYPE_CHECKING:
    from naaims.vehicles import Vehicle
    from naaims.road import Road, RoadLane


class VehicleSpawner(Configurable, Upstream):

    def __init__(self,
                 downstream: Road,
                 vpm: float,  # vehicles per minute
                 factory_types: List[Type[VehicleFactory]],
                 factory_specs: List[Dict[str, Any]],
                 factory_selection_probabilities: List[float],
                 predetermined_spawns: List[Tuple[float, Vehicle]] = []
                 ) -> None:
        """Create a new vehicle spawner.

        Parameters
            downstream: Road
                The road to spawn vehicles on to.
            vpm: float
                Target number of vehicles to spawn per minute using a Poisson
                distribution. Doesn't take into account predetermined spawns.
            factory_types: List[Type[VehicleFactory]]
                The types of VehicleFactory to create and use for Poisson
                spawns.
            factory_specs: List[Dict[str, Any]]
                The specs of VehicleFactory to create and use for Poisson
                spawns.
            factory_selection_probabilities: List[float]
                The probability of using a specific VehicleFactory.
            predetermind_spawns: List[Tuple[float, Vehicle]]
                Specifies predetermined spawns that occur outside of the
                Poisson process. The tuple details the spawn time in
                seconds and the vehicle that will spawn.
        """

        if len(factory_types) != len(factory_specs) != \
                len(factory_selection_probabilities):
            raise ValueError("The number of generator types, specs, and "
                             "probabilities must match.")
        p_factory = sum(factory_selection_probabilities)
        if p_factory != 1:
            if len(predetermined_spawns) == 0:
                raise ValueError("If the generator probabilities sum to 0, "
                                 "there must be predetermined spawns set.")
            elif p_factory != 0:
                raise ValueError("The generator probabilities must sum to 1, "
                                 "or 0 if there are predetermined spawns set.")

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

        # Adjust spawning logic if spawns are pre-determined. Make sure that
        # it's ordered by timestep.
        self.predetermined_spawns: List[Tuple[int, Vehicle]] = \
            [(floor(s/SHARED.SETTINGS.TIMESTEP_LENGTH), v) for s, v in
             predetermined_spawns]
        self.predetermined_spawns.sort(key=lambda s: s[0])

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
            factory_specs=spec['factory_specs'],
            predetermined_spawns=spec.get('predetermined_spawns', [])
        )

    def step_vehicles(self) -> Tuple[List[Vehicle], List[Vehicle]]:
        """Decides whether to spawn and returns spawned and entering vehicles.

        Returns
            List[Vehicle]   The vehicle(s) spawned in this timestep.
            List[Vehicle]   The vehicle(s) that exit the spawner and enter
                            the simulated scope. Usually the same as the
                            spawned vehicle but may be different if road
                            traffic prevents vehicles spawned in prior
                            timesteps from entering at their time of spawn.
        """

        if self.downstream is None:
            raise MissingConnectionError("No downstream object.")

        # Initialize a list of vehicle spawns.
        spawns_this_timestep: List[Vehicle] = []

        # Check if there are predetermined vehicle spawns this timestep.
        while len(self.predetermined_spawns) > 0:
            # Check if it's time for the next spawn.
            if self.predetermined_spawns[0][0] <= 0:
                # If so, take it out of the list and spawn it.
                spawns_this_timestep.append(
                    self.predetermined_spawns.pop(0)[1])
            else:
                # If not, we're done with predetermined spawns this timestep.
                break

        # Roll to spawn a new vehicle.
        if random() < self.spawn_probability:
            # Pick a generator to use based on the distribution of generators
            # and spawn a new vehicle with it.
            spawns_this_timestep.append(
                choices(self.factories, self.factory_selection_probabilities
                        )[0].create_vehicle())

        # Find every downstream lane that vehicle(s) spawned this timestep can
        # enter and still reach its destination. Add both vehicle and lanes to
        # the queue.
        for spawn in spawns_this_timestep:
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
                room_to_spawn = lane.room_to_enter()
                effective_length = vehicle_to_transfer.length * \
                    (1 + 2 * SHARED.SETTINGS.length_buffer_factor)
                if (lane not in blocked_lanes) and \
                        (room_to_spawn > effective_length):
                    vehicle_can_transfer = True

                    # Adjust the entering vehicle's speed so that it'll be
                    # going just fast enough to brake to a stop at the end of
                    # its vailable space. (Or just set it to the speed limit if
                    # it's too fast.)
                    vehicle_to_transfer.velocity = min(
                        self.downstream.speed_limit,
                        VehicleSpawner._fastest_v_no_collision(
                            lane.room_to_enter(False), effective_length,
                            SHARED.SETTINGS.min_braking))

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

        # Delete transferred vehicles from queue starting from the end of the
        # queue so indices don't get messed up
        queue_entries_to_delete.reverse()
        for i in queue_entries_to_delete:
            del self.queue[i]

        # Pass newly spawned vehicles and newly transferred vehicles back to
        # the Simulator if there are any
        return spawns_this_timestep, vehicles_transferred

    @staticmethod
    def _fastest_v_no_collision(room_to_enter: float, effective_length: float,
                                b: float) -> float:
        """The fastest speed a vehicle can fully brake from in x_to_brake."""
        x_to_brake = room_to_enter - effective_length
        return (2*-b*x_to_brake)**.5
