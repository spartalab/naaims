"""
The tiling determines how AIM handles the intersection conflict area (often
called the "box", at least for 4-way 90 degree intersections). It tracks the
management of traffic signal cycle (if there is one) and the process of
testing, confirming, and executing reservations through the intersection.

The two primary methods are arc-based (ArcAIM), where vehicles reserve the area
around a conflict point (the intersection of two trajectories), and square
tiling, where the entire intersection is divided into square tiles with
adjustable granularity (Dresner and Stone 2008).
"""


from __future__ import annotations
from abc import abstractmethod
from math import ceil
from typing import (TYPE_CHECKING, Optional, List, Set, Dict, Tuple, Type,
                    TypeVar, Any, OrderedDict)

import naaims.shared as SHARED
from naaims.archetypes import Configurable
from naaims.util import Coord, VehicleSection, VehicleTransfer
from naaims.lane import ScheduledExit
from naaims.intersection.reservation import Reservation
from naaims.intersection.tilings.tiles import StochasticTile, DeterministicTile

if TYPE_CHECKING:
    from naaims.road import RoadLane
    from naaims.intersection.tilings.tiles import Tile
    from naaims.vehicles import Vehicle
    from naaims.intersection import IntersectionLane

T = TypeVar('T', bound='Tiling')


class Tiling(Configurable):

    @abstractmethod
    def __init__(self,
                 incoming_road_lane_by_coord: Dict[Coord, RoadLane],
                 outgoing_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Tuple[IntersectionLane, ...],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 crash_probability_tolerance: float,
                 cycle: Optional[List[
                     Tuple[Set[IntersectionLane], int]
                 ]] = None,
                 timeout: bool = False,
                 tile_type: Type[Tile] = DeterministicTile,
                 misc_spec: Dict[str, Any] = {}
                 ) -> None:
        """Should instantiate a new Tiling.

        Takes the lanes from the Intersection and figures out how to tile them.

        cycle, if given, takes an list of intersection lane sets and int
        tuples. For each tuple in this list, the first item describes which
        lanes have a green light during the cycle step and should have their
        associated tiles marked, and the second item describes the number of
        timesteps the cycle step lasts for.

        Child tilings should call this init for initial setup, then continue in
        their own init to set up whatever they need to.
        """
        self.incoming_road_lane_by_coord = incoming_road_lane_by_coord
        self.outgoing_road_lane_by_coord = outgoing_road_lane_by_coord
        self.lanes = lanes
        self.lanes_by_endpoints = lanes_by_endpoints
        self.tile_type = tile_type

        # Initialize reservation dicts.
        self.active_reservations: Dict[Vehicle, Reservation] = {}
        self.queued_reservations: Dict[Vehicle, Reservation] = {}

        # Declare tiling stack variable.
        # (Must be implemented in child classes.)
        self.tiles: List[Tuple[Tile, ...]] = []

        # Start up the cycle and save relevant info.
        self.cycle = cycle
        if self.cycle is not None:
            # Loop through the signal cycle and calculate the total length of
            # one full cycle in timesteps.
            self.total_cycle_time: int = sum(c[1] for c in self.cycle)

            # Initialize the first cycle's info.
            self.current_step_index: int = 0
            self.greenlit: Dict[RoadLane,
                                Set[Coord]] = self.greenlit_movements(
                self.cycle[0][0])
            self.time_left_in_cycle: int = self.cycle[0][1]

        # Initialize a dict to track request checking timeouts
        self.timeout_until: Optional[Dict[Vehicle, int]] = {} if timeout else \
            None

        # Initialize rejection threshold. This must be set by child classes.
        self._threshold: float = 0
        self.threshold_registered: bool = False

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a tiling spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: (spec) Interpret the string into the spec dict
        raise NotImplementedError("TODO")

        # TODO: (spec) Enforce provision of tile_type field in manager spec
        #       string
        tile_type: str

        # Based on the spec, identify the correct tiling type
        if tile_type.lower() in {'', 'stochastic', 'stochastictile'}:
            spec['tile_type'] = StochasticTile
        elif tile_type.lower() in {'deterministic', 'deterministictile'}:
            spec['tile_type'] = DeterministicTile
        else:
            raise ValueError("Unsupported Tile type.")

        return spec

    @classmethod
    def from_spec(cls: Type[T], spec: Dict[str, Any]) -> T:
        """Should interpret a spec dict to call the manager's init."""
        return cls(
            incoming_road_lane_by_coord=spec['incoming_road_lane_by_coord'],
            outgoing_road_lane_by_coord=spec[
                'outgoing_road_lane_by_coord'],
            lanes=spec['lanes'],
            crash_probability_tolerance=spec['crash_probability_tolerance'],
            lanes_by_endpoints=spec['lanes_by_endpoints'],
            cycle=spec.get('cycle'),
            timeout=spec.get('timeout', False),
            tile_type=spec.get('tile_type', DeterministicTile),
            misc_spec=spec.get('misc_spec', {})
        )

    # Begin simulation cycle methods

    def start_reservation(self, vehicle: Vehicle) -> IntersectionLane:
        """Move reservation from scheduled to active and return its lane."""
        if vehicle in self.queued_reservations:
            reservation: Reservation = self.queued_reservations[vehicle]
            del self.queued_reservations[vehicle]
            self.active_reservations[vehicle] = reservation
            return reservation.lane
        else:
            raise ValueError("No record of a reservation for this vehicle.")

    def clear_reservation(self, vehicle: Vehicle) -> None:
        """Clear a completed reservation from the tiling's memory."""
        # TODO: (low) Consider clearing off any remaining tiles associated with
        #       this reservation.
        del self.active_reservations[vehicle]

    # Handle logic support methods

    def handle_new_timestep(self, visualize: bool = False
                            ) -> Optional[List[Tuple[List[Coord],
                                                     float, int]]]:
        """Check for crashes. Update tile stack and existing reservations.

        Can return a list of (outline, float, int) tuples, where an outline is
        defined as a list of Coords, the float determines the shape's
        transparency, and the int determines its color.
        """
        layer: Optional[Tuple[Tile, ...]] = None

        # 1. Check for collisions
        self.check_for_collisions()

        # 2. Update tiling for the new timestep
        if len(self.tiles) > 0:
            layer = self.tiles.pop(0)

        # 3. Update the traffic signal cycle
        self.update_cycle()

        # 4. Update existing reservations
        self.update_active_reservations()

        if visualize and (layer is not None):
            return self.tile_layer_to_shape(layer)
        else:
            return None

    @abstractmethod
    def check_for_collisions(self) -> None:
        """Check for collisions in the intersection."""
        # TODO: (low) May only be necessary if we have stochastic movement.
        #       Otherwise collisions should not occur if implemented correctly.
        # vehicles = []
        # for lane in self.lanes:
        #     vehicles += [vp.vehicles for vp in lane.vehicles]
        # # draw every vehicle to check for collisions?
        pass

    def update_cycle(self) -> None:
        """Update the traffic signal cycle if necessary."""
        if self.cycle is not None:
            self.time_left_in_cycle -= 1
            if self.time_left_in_cycle == 0:
                # Go to the next step in the cycle.
                self.current_step_index += 1

                # Roll back to the start of the cycle if we go past the end.
                self.current_step_index %= len(self.cycle)

                # Change greenlit movements and reset the timer.
                self.greenlit = self.greenlit_movements(
                    self.cycle[self.current_step_index][0])
                self.time_left_in_cycle = self.cycle[
                    self.current_step_index][1]

    def greenlit_movements(self, greenlit_lanes: Set[IntersectionLane]
                           ) -> Dict[RoadLane, Set[Coord]]:
        """Return the road lane and permitted movements for a signal cycle."""
        greenlit: Dict[RoadLane, Set[Coord]] = {}
        for lane in greenlit_lanes:
            road_lane: RoadLane = self.incoming_road_lane_by_coord[
                lane.trajectory.start_coord]
            end: Coord = lane.trajectory.end_coord
            if road_lane not in greenlit:
                greenlit[road_lane] = {end}
            else:
                greenlit[road_lane].add(end)
        return greenlit

    @abstractmethod
    def update_active_reservations(self) -> None:
        """Given vehicle movement in the last step, update their reservations.

        Have manager and tiling compare the current positions and velocities
        of all vehicles in intersection. Use the difference to update
        reservations to reflect resolved stochasticity. Should only be
        necessary for stochastic reservations, but it might be nice for
        deterministic methods with nonzero buffers, including when a vehicle
        exits the intersection.
        """
        # TODO: (low) Adjust tiles reserved based on resolved movement.
        pass

    # Methods used by manager during reservation request handling

    @staticmethod
    def issue_permission(vehicle: Vehicle, lane: RoadLane,
                         rear_exit: ScheduledExit) -> None:
        """Issue permission to enter the intersection to a vehicle.

        Gives vehicles permission to enter the intersection and registers the
        scheduled exit of their rear section with the lane of origin.
        """
        vehicle.permission_to_enter_intersection = True
        lane.register_latest_scheduled_exit(rear_exit)

    def check_request(self, incoming_road_lane_original: RoadLane,
                      mark: bool = False, sequence: bool = False
                      ) -> Optional[Tuple[Reservation, int]]:
        """If a lane's request(s) work, return its first reservation and index.

        Check if a lane's reservation (plural if the sequence flag is True)
        can work. If so, return the workable reservation of the first vehicle
        in the reservation sequence, which itself contains a symbolic link to
        the reservations of trailing vehicles.

        If the mark flag is True, mark the tiles used by each potential
        reservation with the probability that the reservation will use the tile
        (for later checking of permissible combinations of reservations from
        different incoming road lanes).

        The test is run by projecting the simulator forward assuming that the
        first vehicle in the sequence accelerates uncontested down the
        intersection lane and out onto the downstream lane, and any trailing
        vehicles in the sequence observe lane-following behavior. The test ends
        when the last vehicle in the sequence whose reservation hasn't been
        rejected exits the intersection in the test.

        If the sequence is long, we could conceivably have many vehicles on the
        downstream lane. In case vehicles start exiting the downstream road
        lane as well, assume they simply disappear.

        Parameters
            incoming_road_lane_original: RoadLane
                The incoming road lane to check for working reservations.
                (It'll be duplicated so that's why it's called _original.)
            mark: bool
                Whether or not to mark tiles to check for conflicts between
                potential reservations on different intersection lanes.
            sequence: bool
                Whether to check if vehicles immediately following the first
                vehicle without a reservation that
                    a. have the same movement, and if so
                    b. can follow it into the intersection
                should be included in the returned list of reservations.
        """

        # TODO: (performance) Implement some request timeout function, like
        #       (t_current-t_arrival)/2.

        # Fetch the vehicle index or indices of the lane's request.
        indices = incoming_road_lane_original.first_without_permission(
            sequence=sequence)
        if indices is None:
            # There are no requesting vehicles in this lane, so return.
            return None
        else:
            counter = start = indices[0]
            end_at = indices[1]
            originals = incoming_road_lane_original.vehicles[counter:end_at]

        # Enforce a cooldown on the frequency with which vehicles can make new
        # requests. See Dresner 2008 section 3.4.4 Timeouts.
        leader: Vehicle = incoming_road_lane_original.vehicles[counter]
        if (self.timeout_until is not None) and \
                (leader in self.timeout_until):
            t_timeout_expired = self.timeout_until[leader]
            if t_timeout_expired <= SHARED.t:
                # Timeout complete. Remove vehicle from the timeout list.
                del self.timeout_until[leader]
            else:
                # Timeout still active. Skip checking the request of this
                # vehicle.
                return None

        # Fetch the projected entrance of the first vehicle in the request
        # sequence and set its time as the start of the reservation test.
        new_exit: Optional[ScheduledExit] = \
            incoming_road_lane_original.soonest_exit(counter)
        if new_exit is None:
            # The leader needs to stop for a while before they can make a
            # workable reservation.
            return None
        test_t = new_exit.t
        leader_arrival = new_exit.t  # for timeout calculation

        # Fetch and clone the request's incoming lane, IntersectionLane, and
        # outgoing lane.
        incoming_road_lane = incoming_road_lane_original.clone()
        outgoing_coord: Coord = new_exit.vehicle.next_movements(
            incoming_road_lane.trajectory.end_coord)[0]
        intersection_lane_original: IntersectionLane = self.lanes_by_endpoints[
            (incoming_road_lane.trajectory.end_coord, outgoing_coord)]
        intersection_lane = intersection_lane_original.clone()
        outgoing_road_lane: RoadLane = self.outgoing_road_lane_by_coord[
            outgoing_coord].clone()

        # Initialize data structures used for reservations.
        clone_to_original: Dict[Vehicle, Vehicle] = {}
        test_reservations: Dict[Vehicle, Reservation] = {}
        valid_reservations: OrderedDict[Vehicle, Reservation] = OrderedDict()
        last_exit: Optional[ScheduledExit] = None

        # Mock the simulation loop until all vehicles in the test sequence
        # have spawned, progressed, and exited the intersection lane.
        while (len(intersection_lane.vehicles) > 0) or (counter < end_at):
            test_complete, counter, test_t, last_exit, new_exit = \
                self._mock_step(start, counter, end_at, test_t, new_exit,
                                incoming_road_lane, intersection_lane,
                                outgoing_road_lane, clone_to_original,
                                test_reservations, valid_reservations,
                                last_exit, originals,
                                incoming_road_lane_original,
                                intersection_lane_original,
                                mark)
            if test_complete:
                break

        if (self.timeout_until is not None) and len(valid_reservations) == 0:
            # Enforce a timeout until the vehicle can make its next request.
            # This timeout will be the smaller of half a second (in timesteps)
            # or half the difference between the current time and the vehicle's
            # arrival time.
            self.timeout_until[leader] = SHARED.t + \
                round(min(.5*SHARED.SETTINGS.steps_per_second,
                          (leader_arrival - SHARED.t)/2))

        # Return the reservation in the sequence and its vehicle's index in the
        # incoming road lane, if there are any valid reservations.
        return (next(iter(valid_reservations.values())), start) if \
            (len(valid_reservations) > 0) else None

    def _mock_step(self, start: int, counter: int, end_at: int, test_t: int,
                   new_exit: Optional[ScheduledExit],
                   incoming_road_lane: RoadLane,
                   intersection_lane: IntersectionLane,
                   outgoing_road_lane: RoadLane,
                   clone_to_original: Dict[Vehicle, Vehicle],
                   test_reservations: Dict[Vehicle, Reservation],
                   valid_reservations: OrderedDict[Vehicle, Reservation],
                   last_exit: Optional[ScheduledExit],
                   originals: List[Vehicle],
                   incoming_road_lane_original: RoadLane,
                   intersection_lane_original: IntersectionLane,
                   mark: bool = False) -> Tuple[bool, int, int,
                                                Optional[ScheduledExit],
                                                Optional[ScheduledExit]]:
        """Mock the simulation loop.

        Refer to simulator.step() for more details on the simulation loop. Note
        that clones are only spawned into the test environment at the timestep
        when they first enter the intersection, and are removed as soon as they
        exit the intersection.

        Returns:
            test_complete: bool
                Whether or not the check_request test is complete.
            counter: int
            test_t: int
            last_exit: Optional[ScheduledExit]
                The latest scheduled exit from the incoming road lane onto the
                intersection lane, if there is one.
            new_exit: Optional[ScheduledExit]
                The exit of the next vehicle to enter the intersection, if
                there is one on deck.
        All other sim step variables are modified in place.
        """

        # 1.  Update speeds for all vehicles.
        self._mock_update_speeds(intersection_lane)

        # 2a. Use the updated speeds to progress the position of the clone on
        #     the downstream lane.
        self._mock_outgoing_step_vehicles(outgoing_road_lane)

        # 2b. Progress positions for clones in the intersection lane and
        # 3a. transfer exiting vehicle sections onto the downstream lane.
        # 4a. If a clone has fully exited, check the edge buffer tiles of their
        #     reservation, and if they work separately delete the clone from
        #     the downstream road lane as we no longer need to check its
        #     position in the intersection.
        test_complete = self._mock_intersection_step_vehicles(
            intersection_lane, outgoing_road_lane, test_reservations,
            valid_reservations, test_t, mark)
        if test_complete:
            return True, -1, -1, None, None

        # 2c. Progress the position of the clone on the incoming road and
        # 3b. transfer section(s) transitioning from road to intersection. If a
        #     vehicle finishes transitioning, update last_exit with its exit.
        last_exit = self._mock_incoming_step_vehicles(
            incoming_road_lane, intersection_lane, clone_to_original,
            test_reservations, last_exit, test_t)

        # 4b. Check and log the tiles used by all clones still in the
        #     intersection after the speed and position update. If a clone's
        #     used tiles are incompatible with a confirmed reservation, reject
        #     its request and all following.
        counter = self._all_pos_to_tile(intersection_lane,
                                        incoming_road_lane,
                                        clone_to_original,
                                        test_reservations,
                                        valid_reservations, counter,
                                        end_at, test_t, mark)
        if counter < 0:
            return True, -1, -1, None, None

        # 4d. Spawn the next clone if it's time, it's possible, and there are
        #     still clones to spawn.
        if (len(incoming_road_lane.vehicles) == 0) and (counter < end_at):
            # Find and cache when and how the next clone enters.
            if new_exit is None:
                new_exit = incoming_road_lane_original.soonest_exit(
                    counter, last_exit)

            assert new_exit is not None
            if test_t >= new_exit.t:
                # The next vehicle's time has come.
                test_complete, counter = self._spawn_next_clone(
                    intersection_lane, incoming_road_lane, originals,
                    clone_to_original, test_reservations,
                    valid_reservations, new_exit, start, counter, end_at,
                    test_t, intersection_lane_original, mark)
                new_exit = None
                if test_complete:
                    return True, -1, -1, None, None

        # 5. Update the test environment time step.
        test_t += 1

        return False, counter, test_t, last_exit, new_exit

    @staticmethod
    def _mock_update_speeds(intersection_lane: IntersectionLane) -> None:
        """Update speeds for all vehicles in test reservation in place.

        Speed updates are controlled by the intersection so only calling the
        intersection lane's get_new_speeds function is necessary.
        """
        for vehicle, update in intersection_lane.get_new_speeds().items():
            vehicle.velocity = update.velocity
            vehicle.acceleration = update.acceleration

    @staticmethod
    def _mock_outgoing_step_vehicles(outgoing_road_lane: RoadLane) -> None:
        """Progress the outgoing road lane clone's position in place.

        Errors if the clone's center section exits the road lane as we'll no
        longer be able to update its position. This shouldn't happen; if it
        does the outgoing road is way too short.
        """
        transfers = outgoing_road_lane.step_vehicles()
        if len(transfers) > 0:
            for transfer in transfers:
                if transfer.section is VehicleSection.CENTER:
                    raise RuntimeError("Outgoing road too short.")

    def _mock_intersection_step_vehicles(self,
                                         intersection_lane: IntersectionLane,
                                         outgoing_road_lane: RoadLane,
                                         test_reservations: Dict[Vehicle,
                                                                 Reservation],
                                         valid_reservations: OrderedDict[
                                             Vehicle, Reservation],
                                         test_t: int, mark: bool) -> bool:
        """Handle progression on intersection lane, including transfers.

        Progress positions for clones in the intersection lane and transfer
        exiting vehicle sections onto the downstream lane.

        If a clone has fully exited, check the edge buffer tiles of their
        reservation, and if they work separately delete the clone from the
        downstream road lane as we no longer need to check its position in the
        intersection.
        """
        transfers = intersection_lane.step_vehicles()
        for transfer in transfers:
            if transfer.section is VehicleSection.REAR:
                # Clone is fully exiting the incoming road lane and fully
                # entering the intersection lane in this timestep.
                clone: Vehicle = transfer.vehicle
                reservation: Reservation = test_reservations[clone]

                # Attach buffer tiles to this exiting reservation request
                # and check if these tiles are free.
                edge_buffer_tiles = self.io_tile_buffer(
                    intersection_lane, test_t, clone, reservation,
                    prepend=False, mark=mark)
                if edge_buffer_tiles is None:
                    # Not only is this reservation not possible, all clones
                    # behind it also have invalid reservations, so we can
                    # terminate the test here and return the valid list after
                    # removing the last reservation's reference to this invalid
                    # request.
                    if reservation.dependent_on is not None:
                        reservation.dependent_on.dependency = None
                    # Remove tile markings associated with invalid
                    # test_reservations if they had any.
                    if mark:
                        for r in test_reservations.values():
                            self.clear_marked_tiles(r)
                    return True
                else:
                    # This reservation request is compatible with existing
                    # reservations and can be confirmed. Attach the buffer
                    # tiles to reservation, finalize it, and delete it from the
                    # downstream road lane now that it's exited the
                    # intersection.
                    reservation.tiles.update(edge_buffer_tiles)
                    reservation.exit_rear = ScheduledExit(
                        reservation.entrance_front.vehicle,
                        VehicleSection.REAR, test_t, clone.velocity)
                    res = test_reservations[clone]
                    valid_reservations[res.vehicle] = res
                    del test_reservations[clone]
                    outgoing_road_lane.vehicles = []
                    del outgoing_road_lane.vehicle_progress[clone], clone, res
            else:
                outgoing_road_lane.enter_vehicle_section(transfer)
        return False

    @staticmethod
    def _mock_incoming_step_vehicles(incoming_road_lane: RoadLane,
                                     intersection_lane: IntersectionLane,
                                     clone_to_original: Dict[Vehicle, Vehicle],
                                     test_reservations: Dict[Vehicle,
                                                             Reservation],
                                     last_exit: Optional[ScheduledExit],
                                     test_t: int) -> Optional[ScheduledExit]:
        """Progress the incoming road lane clone's position in place.

        Also transfers clone sections onto the intersection lane if necessary.
        """
        transfers = incoming_road_lane.step_vehicles()
        for transfer in transfers:
            intersection_lane.enter_vehicle_section(transfer)
            clone = transfer.vehicle
            if transfer.section is VehicleSection.REAR:
                last_exit = ScheduledExit(vehicle=clone_to_original[clone],
                                          section=VehicleSection.REAR,
                                          t=test_t, velocity=clone.velocity)
                test_reservations[clone].entrance_rear = last_exit
        return last_exit

    def _all_pos_to_tile(self, intersection_lane: IntersectionLane,
                         incoming_road_lane: RoadLane,
                         clone_to_original: Dict[Vehicle, Vehicle],
                         test_reservations: Dict[Vehicle,
                                                 Reservation],
                         valid_reservations: OrderedDict[Vehicle, Reservation],
                         counter: int, end_at: int,
                         test_t: int, mark: bool) -> int:
        """Log the tiles used by all clones at this test_t.

        Check and log the tiles used by all clones still in the intersection
        after the speed and position update. If a clone's used tiles are
        incompatible with a confirmed reservation, reject its request and all
        following.

        Returns:
            counter (unchanged) in most cases
        """
        to_remove: List[Vehicle] = []
        for i, clone in enumerate(intersection_lane.vehicles):
            reservation = test_reservations[clone]
            tiles_used = self.pos_to_tiles(intersection_lane, test_t,
                                           clone, reservation, mark=mark)
            if tiles_used is not None:
                reservation.tiles[test_t] = tiles_used
            else:
                # Scrap all test_reservations after this one and stop spawning
                # new ones.

                # If this is the first clone still in the intersection, we can
                # simply terminate the test here since no clone still in the
                # intersection can have a valid reservation.
                if i == 0:
                    if reservation.dependent_on is not None:
                        # Need to decouple from the last confirmed res.
                        reservation.dependent_on.dependency = None

                    # Remove tile markings associated with all invalid test
                    # reservations if there were any.
                    if mark:
                        for r in test_reservations.values():
                            self.clear_marked_tiles(r)

                    return -1
                # Otherwise we need to scrap the data for all clones at or
                # behind this one with the rejected request. Start by
                # decoupling from the prior reservation request.
                test_reservations[intersection_lane.vehicles[i-1]
                                  ].dependency = None

                # Delete the clones after this one, their progress, and
                # unmarked their marked tiles if necessary.
                to_remove += intersection_lane.vehicles[i:]
                for clone_to_del in intersection_lane.vehicles[i:]:
                    if mark:
                        self.clear_marked_tiles(
                            test_reservations[clone_to_del])
                    del clone_to_original[clone_to_del]
                    del test_reservations[clone_to_del]

                # Delete the clone on the upstream lane and stop spawning new
                # clones.
                if len(incoming_road_lane.vehicles) > 0:
                    clone = incoming_road_lane.vehicles[0]
                    del incoming_road_lane.vehicle_progress[clone]
                    incoming_road_lane.vehicles = []
                counter = end_at

                # Break out of the loop since we don't need to update
                # the positions of any following clones (but we still
                # need to continue the test to verify that the remaining
                # requests ahead of this one are valid).
                break

        # Finalize removals from the intersection lane.
        for clone in to_remove:
            intersection_lane.vehicles.remove(clone)
            del intersection_lane.vehicle_progress[clone]

        return counter

    def _spawn_next_clone(self, intersection_lane: IntersectionLane,
                          incoming_road_lane: RoadLane,
                          originals: List[Vehicle],
                          clone_to_original: Dict[Vehicle, Vehicle],
                          test_reservations: Dict[Vehicle,
                                                  Reservation],
                          valid_reservations: OrderedDict[Vehicle,
                                                          Reservation],
                          new_exit: ScheduledExit,
                          start: int, counter: int, end_at: int,
                          test_t: int,
                          intersection_lane_original: IntersectionLane,
                          mark: bool
                          ) -> Tuple[bool, int]:
        """Spawn the next clone. Check and log its starting tiles.

        Returns
            whether the test is complete or not
            an updated counter
        """
        # Clone the vehicle and initialize its reservation.
        original: Vehicle = new_exit.vehicle
        clone = original.clone_for_request()
        clone.velocity = new_exit.velocity
        idx = counter - start - 1
        preceding_vehicle = originals[idx] if (idx >= 0) else None
        preceding_res: Optional[Reservation] = None
        if preceding_vehicle is not None:
            preceding_res = test_reservations.get(preceding_vehicle)
            if preceding_res is None:
                preceding_res = valid_reservations.get(preceding_vehicle)
        reservation = Reservation(
            vehicle=original,
            res_pos=intersection_lane.trajectory.start_coord,
            tiles={},
            lane=intersection_lane_original,
            entrance_front=new_exit,
            dependent_on=preceding_res,
            dependency=None
        )

        # At this time, the front of the vehicle should just be entering the
        # intersection lane, and the rest of the vehicle should still be
        # upstream. (This isn't exactly 1:1 with simulated behavior as vehicles
        # can travel slightly into the intersection as they enter, but it'll be
        # accurate to within one timestep.)
        # TODO: (low) Replace the just-entering heuristic with
        #       RoadLane.progress_at_exit(counter, new_exit)

        # Load its center and rear sections onto the incoming lane.
        incoming_road_lane.add_vehicle(clone)
        incoming_road_lane.enter_vehicle_section(VehicleTransfer(
            clone, VehicleSection.CENTER,
            incoming_road_lane.trajectory.length - clone.length *
            (.5 + SHARED.SETTINGS.length_buffer_factor),
            incoming_road_lane.trajectory.start_coord))
        incoming_road_lane.enter_vehicle_section(VehicleTransfer(
            clone, VehicleSection.REAR,
            incoming_road_lane.trajectory.length - clone.length *
            (1 + 2*SHARED.SETTINGS.length_buffer_factor),
            incoming_road_lane.trajectory.start_coord))

        # Transfer its front section onto the intersection lane.
        intersection_lane.enter_vehicle_section(VehicleTransfer(
            clone, VehicleSection.FRONT, 0,
            intersection_lane.trajectory.start_coord))

        # Check if its tiles work.
        edge_tiles_used = self.io_tile_buffer(
            intersection_lane, test_t, clone, reservation,
            prepend=True, mark=mark)
        if edge_tiles_used is not None:
            tiles_used = self.pos_to_tiles(intersection_lane, test_t,
                                           clone, reservation,
                                           mark=mark)
        else:
            tiles_used = None
        if (edge_tiles_used is not None) and (tiles_used is not None):
            # Register these edge buffer tiles and regular tiles to the new
            # reservation, which has no tiles used yet.
            reservation.tiles = edge_tiles_used
            reservation.tiles[test_t] = tiles_used

            # Register this vehicle's reservation as dependent on its preceding
            # clone's reservation, if there is one.
            if reservation.dependent_on is not None:
                reservation.dependent_on.dependency = reservation

            # Add the new clone to the test data structures.
            clone_to_original[clone] = original
            test_reservations[clone] = reservation

            # Move the clone tracker up.
            counter += 1

            # Once done, clear for the next clone to add.
            return False, counter
        else:
            # Scrap this reservation and stop spawning new ones. We didn't
            # register this clone any structures, so there's less to do than in
            # other rejection cases.

            if len(intersection_lane.vehicles) == 1:
                # No reservation still being tested is workable, so scrap them
                # all and just return the reservations already confirmed.
                return True, end_at

            # Stop spawning new clones.
            counter = end_at
            return False, counter

    @abstractmethod
    def pos_to_tiles(self, lane: IntersectionLane, t: int,  # type: ignore
                     clone: Vehicle, reservation: Reservation,
                     force: bool = False, mark: bool = False
                     ) -> Optional[Dict[Tile, float]]:
        """Return a vehicle's tiles and percentage used if it works.

        Given a vehicle with a test-updated position, its in-lane progress, and
        a future timestep t, return the tiles used by this vehicle in this
        timestep given its position if the reservation is possible. If not,
        return None (unless the force flag is True, in which case return the
        tiles used regardless of whether the tiles allow it).

        Parameters
            lane: IntersectionLane
                The lane the clone is tested on.
            t: int
                The future timestep we're checking tiles for.
            clone: Vehicle
                The vehicle whose position we're translating to tiles used.
            progress: VehicleProgress
                How far along the lane the clone is at this timestep.
            reservation: Reservation
                The reservation associated with this clone.
            force: bool = False
                If force, don't bother checking if a tile is compatible with
                this vehicle's reservation before returning.
            mark: bool = False
                Whether to mark the tiles used with this potential reservation.
        """
        if force and mark:
            raise ValueError("Can't force and mark tiles at the same time.")

        # At this point in the cycle, everything is fully resolved at timestep
        # SHARED.t, so the first layer in the tile stack represents time
        # SHARED.t+1. t > SHARED.t so we need to extend the tile stack to t. If
        # the tilings created so far doesn't cover this future timestep yet,
        # keep creating new layers until we reach this timestep.
        if t <= SHARED.t:
            raise ValueError("t must be a future timestep.")
        timesteps_from_now = t - SHARED.t
        while len(self.tiles) < timesteps_from_now:
            self._add_new_layer()

        # Find the tiles this vehicle is estimated to use at this timestep by
        # using the clone's properties and proportional progress along the
        # intersection trajectory. Mark the tiles used by this reservation if
        # mark is True. Ignore feasibility if force is True.

        # TODO: Expand the number of tiles based on the vehicle's throttle
        #       and tracking scores. Use tile.will_reservation_work() if not
        #       force and tile.mark() if mark.
        # TODO: (sequence) If the vehicle is following another in a sequence,
        #       usage of a tile by a preceding vehicle shouldn't disqualify a
        #       reservation. Tweak tile implementation to account for this.

    @abstractmethod
    def io_tile_buffer(self, lane: IntersectionLane, t: int,  # type: ignore
                       clone: Vehicle, reservation: Reservation,
                       prepend: bool, force: bool = False, mark: bool = False
                       ) -> Optional[Dict[int, Dict[Tile, float]]]:
        """Should return edge buffer tiles and percentages used if it works.

        Given a vehicle with a test-updated position, its in-lane progress, and
        a future timestep t, return the edge tiles used by this vehicle a few
        timesteps before or after (depending on the prepend flag), provided the
        tiles can accept this reservation. If not, return None (unless the
        force flag is True, in which case return the tiles used regardless of
        whether the tiles allow it).

        Note that this does not return edge tiles AT the timestep t, only edge
        buffer tiles immediately before or after depending on prepend's value.

        Reserving a buffer of edge tiles ensures that there is proper spacing
        between vehicles on entry and exit to an automated intersection. See
        Dresner and Stone 2008 for more information.

        Parameters
            lane: IntersectionLane
                The lane the clone is tested on.
            t: int
                The future timestep we're checking tiles for.
            clone: Vehicle
                The vehicle whose position we're translating to tiles used.
            progress: VehicleProgress
                How far along the lane the clone is at this timestep.
            reservation: Reservation
                The reservation associated with this clone.
            prepend: bool
                If true, return edge tiles before timestep. If false, return
                edge tiles after timestep.
            force: bool = False
                If force, don't bother checking if a tile is compatible with
                this vehicle's reservation before returning.
            mark: bool = False
                Whether to mark the tiles used with this potential reservation.
        """
        if t <= SHARED.t:
            raise ValueError("t must be a future timestep.")
        if force and mark:
            raise ValueError("Can't force and mark tiles at the same time.")

    @staticmethod
    def _exit_res_timesteps_forward(velocity: float) -> int:
        """Return the timesteps forward the exiting tile buffer should go.

        Based on Toader's modification to AIM4 by Au, Stone, and Dresner.
        http://www.cs.utexas.edu/~aim/aim4sim/versions/AIM4-release-1.0.4-fixed-collisions-notes.pdf
        """
        return ceil((.3 + max(0, velocity - 15)*.2
                     )*SHARED.SETTINGS.steps_per_second)

    def clear_marked_tiles(self, reservation: Reservation) -> None:
        """Clear tiles marked with this reservation before discarding."""
        for tile_layer in self.tiles:
            for tile in tile_layer:
                tile.remove_mark(reservation)

    def confirm_reservation(self, reservation: Reservation, lane: RoadLane,
                            ) -> None:
        """Confirm a potential reservation."""

        # Confirm the reservation on the tiles it uses.
        for tiles_dict in reservation.tiles.values():
            for tile, p in tiles_dict.items():
                tile.confirm_reservation(reservation, p)

        vehicle: Vehicle = reservation.vehicle
        vehicle.has_reservation = True
        self.queued_reservations[vehicle] = reservation
        assert reservation.entrance_rear is not None
        self.issue_permission(vehicle, lane, reservation.entrance_rear)

    @abstractmethod
    def find_best_batch(self, requests: Dict[RoadLane, List[Reservation]]
                        ) -> List[Tuple[RoadLane, Reservation]]:
        """Take potential reservation permutations and find the best batch.

        Take a list of potential reservations from each road lane with a
        reservation. For each road lane, the reservations are listed in order
        of priority, i.e., the second reservation in a RoadLane's list follows
        the first's reservation and so on.

        Run through these road lanes' reservations and return the highest value
        compatible subset of reservations by sum total VOT and the road lane
        they originate from (for use in spacing out road lane exits).
        """

        # TODO: (auction) Sort through the requests from each RoadLane and
        #       find the maximum value combination. Remember that we can't
        #       selectively reject reservations from a RoadLane; instead, we
        #       have to accept the first n consecutive reservations. If we
        #       accept less than the full list of reservations from a lane,
        #       remember to decouple the dependency from the last accepted
        #       reservation.

        raise NotImplementedError("TODO")

    def clear_potential_reservations(self) -> None:
        """Go through all tiles and clear potential reservation markings."""
        for tile_layer in self.tiles:
            for tile in tile_layer:
                tile.remove_all_marks()

    # Support methods used by the Tiling

    @abstractmethod
    def _add_new_layer(self) -> None:
        """Should extend the tiling stack one more timestep.

        Tiling depth is created as needed. Calling this function will create a
        new layer in the tiling process.

        At this point in the cycle, everything is fully resolved at timestep
        SHARED.t, so the first layer in the tile stack (i.e., index 0)
        represents time SHARED.t+1.
        """
        raise NotImplementedError("Must be implemented in child classes.")

    @property
    def threshold(self) -> float:
        """Return the probability threshold per tile."""
        if not self.threshold_registered:
            raise RuntimeError("Probability threshold not yet registered.")
        return self._threshold

    @abstractmethod
    def tile_layer_to_shape(self, layer: Tuple[Tile, ...]
                            ) -> List[Tuple[List[Coord], float, int]]:
        """Convert a tile layer to plottable Coords.

        Returns a list of (outline, float, int) tuples, where an outline is
        defined as a list of Coords, the float determines the shape's
        transparency, and the int determines its color.
        """
        raise NotImplementedError("Must be implemented in child classes.")
