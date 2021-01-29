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


from abc import abstractmethod
from typing import (Optional, Iterable, Set, Dict, Tuple, Type, TypeVar, Any,
                    List)
from collections import deque

import aimsim.shared as SHARED
from aimsim.archetypes import Configurable
from aimsim.util import Coord, SpeedUpdate, VehicleSection
from aimsim.lanes import (IntersectionLane, RoadLane, VehicleProgress,
                          ScheduledExit)
from aimsim.vehicles import Vehicle
from aimsim.intersections.reservations import Reservation
from aimsim.intersections.tiles import Tile, DeterministicTile

T = TypeVar('T', bound='Tiling')


class Tiling(Configurable):

    @abstractmethod
    def __init__(self,
                 upstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 downstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Dict[Coord, IntersectionLane],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tile_type: Type[Tile] = Tile,
                 cycle: Optional[List[
                     Tuple[Set[IntersectionLane], int]
                 ]] = None
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
        self.upstream_road_lane_by_coord = upstream_road_lane_by_coord
        self.downstream_road_lane_by_coord = downstream_road_lane_by_coord
        self.lanes = lanes
        self.lanes_by_endpoints = lanes_by_endpoints

        # Initialize reservation dicts.
        self.active_reservations: Dict[Vehicle, Reservation] = {}
        self.queued_reservations: Dict[Vehicle, Reservation] = {}

        # Declare tiling stack variable.
        # (Must be implemented in child classes.)
        self.tiles: List

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
            spec['tile_type'] = Tile
        elif tile_type.lower() in {'deterministic', 'deterministictile'}:
            spec['tile_type'] = DeterministicTile
        else:
            raise ValueError("Unsupported Tile type.")

        return spec

    @classmethod
    def from_spec(cls: Type[T], spec: Dict[str, Any]) -> T:
        """Should interpret a spec dict to call the manager's init."""
        return cls(
            upstream_road_lane_by_coord=spec['upstream_road_lane_by_coord'],
            downstream_road_lane_by_coord=spec[
                'downstream_road_lane_by_coord'],
            lanes=spec['lanes'],
            lanes_by_endpoints=spec['lanes_by_endpoints'],
            tile_type=spec['tile_type'],
            cycle=spec['cycle']
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

    def handle_new_timestep(self) -> None:
        """Check for crashes. Update tile stack and existing reservations."""

        # 1. Check for collisions
        self.check_for_collisions()

        # 2. Update tiling for the new timestep
        self.tiles.pop(0)

        # 3. Update the traffic signal cycle
        self.update_cycle()

        # 4. Update existing reservations
        self.update_active_reservations()

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
            road_lane: RoadLane = self.upstream_road_lane_by_coord[
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

    def issue_permission(self, vehicle: Vehicle, lane: RoadLane,
                         rear_exit: ScheduledExit) -> None:
        """Issue permission to enter the intersection to a vehicle.

        Gives vehicles permission to enter the intersection and registers the
        scheduled exit of their rear section with the lane of origin.
        """
        vehicle.permission_to_enter_intersection = True
        lane.register_latest_scheduled_exit(rear_exit)

    def check_request(self, incoming_lane: RoadLane, mark: bool = False,
                      sequence: bool = False) -> List[Reservation]:
        """If a lane's request(s) work, return the potential Reservation(s).

        Check if a lane's reservation (plural if the sequence flag is True)
        can work. If so, return the workable reservation(s). If the mark flag
        is True, mark the tiles used by each potential reservation with the
        probability that the reservation will use the tile (for later checking
        of permissible combinations of reservations from different incoming
        road lanes).

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
            incoming_lane: RoadLane
                The incoming road lane to check for working reservations.
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

        # Fetch the vehicle index or indices of the lane's request.
        indices = incoming_lane.first_without_permission(sequence=sequence)
        if indices is None:
            # There are no requesting vehicles in this lane, so return.
            return []
        else:
            counter = indices[0]
            end_at = indices[1]

        # Find when the test starts by fetching the time of the projected
        # entrance of the first vehicle in the request sequence.
        new_exit: Optional[ScheduledExit] = incoming_lane.soonest_exit(counter)
        if new_exit is None:
            raise RuntimeError("Soonest exit should have returned a "
                               "ScheduledExit but didn't.")

        # Fetch the desired IntersectionLane we'll be operating on and its
        # corresponding downstream lane.
        downstream_coord: Coord = new_exit.vehicle.next_movements(
            incoming_lane.end_coord)[0]
        lane: IntersectionLane = self.lanes_by_endpoints[
            (incoming_lane.end_coord, downstream_coord)]
        downstream_lane: RoadLane = self.downstream_road_lane_by_coord[
            downstream_coord]

        # Track what timestep our test is in. The test starts when the first
        # vehicle in the sequence is scheduled to exit its road lane and enter
        # the intersection.
        test_t = new_exit.t

        # Initialize data structures used for reservations.
        originals: Dict[Vehicle, Vehicle] = {}
        test_reservations: Dict[Vehicle, Reservation] = {}
        valid_reservations: List[Reservation] = []

        # Initialize data structures used for clones in the intersection.
        in_clones: List[Vehicle] = []
        in_progress: Dict[Vehicle, VehicleProgress] = {}

        # Declare data structures used for clones still entering.
        incoming_progress: Optional[VehicleProgress] = None
        last_exit: ScheduledExit

        # Initialize data structures used for clones that have partially or
        # fully exited the intersection.
        downstream_clones: List[Vehicle] = []
        downstream_progress: Dict[Vehicle, VehicleProgress] = {}

        # Each pass of this while loop is one pseudo-timestep.
        # (This is like a miniature version of the whole simulation loop.)
        while (len(in_clones) > 0) and (counter < end_at):

            # TODO: (low) A lot of this is repetitive. Consider modularizing.
            # TODO: (runtime) There is a lot of namespace reuse. Might cause
            #       some unintentional errors if values aren't reset as
            #       expected. Consider addressing.

            # Update speeds for clones on the downstream road lane.
            preceding_vehicle_progress: Optional[float] = None
            preceding_vehicle_stopping_distance: Optional[float] = None
            clone_on_seam: bool = False
            for clone in downstream_clones:
                progress_front: Optional[float
                                         ] = downstream_progress[clone].front
                assert progress_front is not None
                if downstream_progress[clone].rear is not None:
                    # The downstream lane is responsible for this clone's speed
                    progress: Optional[float
                                       ] = downstream_progress[clone].center
                    assert progress is not None

                    # Update the clone's speed and acceleration.
                    a: float
                    if preceding_vehicle_progress is None:
                        a = downstream_lane.accel_update_uncontested(clone,
                                                                     progress)
                    else:
                        assert preceding_vehicle_stopping_distance is not None
                        a = downstream_lane.accel_update_following(
                            clone, progress,
                            downstream_lane.effective_stopping_distance(
                                preceding_vehicle_progress, progress_front,
                                preceding_vehicle_stopping_distance))
                    su: SpeedUpdate = downstream_lane.speed_update(clone,
                                                                   progress,
                                                                   a)
                    clone.velocity = su.velocity
                    clone.acceleration = su.acceleration

                    # Update preceding data for the next loop.
                    preceding_vehicle_progress = downstream_progress[clone
                                                                     ].rear
                    preceding_vehicle_stopping_distance = clone.stopping_distance()
                else:
                    # This clone is still transferring from intersection to
                    # road, so the intersection is responsible for its speed.
                    # This is the last clone on the downstream lane.
                    if preceding_vehicle_progress is not None:
                        # Calculate the stopping distance for this clone for
                        # the intersection lane to use after exiting this loop.
                        assert preceding_vehicle_stopping_distance is not None
                        preceding_vehicle_stopping_distance = downstream_lane.\
                            effective_stopping_distance(
                                preceding_vehicle_progress,
                                progress_front,
                                preceding_vehicle_stopping_distance)
                    clone_on_seam = True
                    break

            # Calculate the stopping distance across the road-intersection seam
            # Case 1: There are no vehicles downstream, so preceding_sd is
            #         None. This is the default sd so nothing to do.
            # Case 2: There's a vehicle on the seam. preceding_sd is not None
            #         because it was already defined in the downstream loop's
            #         else block.
            # Case 3: There are vehicles downstream but none on the seam.
            #         Calculate the stopping distance on the downstream lane
            #         here and add the stopping distance in the intersection
            #         lane distance in the next loop.
            if (preceding_vehicle_stopping_distance is not None) and (
                    not clone_on_seam):
                assert preceding_vehicle_progress is not None
                preceding_vehicle_stopping_distance = downstream_lane.effective_stopping_distance(
                    preceding_vehicle_progress, 0,
                    preceding_vehicle_stopping_distance)

            # Update speeds for clones on the intersection lane.
            preceding_vehicle_progress = None
            for clone in in_clones:
                # The intersection lane is responsible for the speed update of
                # all vehicles even partially in itself.
                progress = in_progress[clone].front
                if progress is not None:
                    if (preceding_vehicle_progress is None) and (
                            preceding_vehicle_stopping_distance is not None):
                        # See Case 3 in the preceding code block. Add the
                        # stopping distance between this vehicle and the end of
                        # the lane.
                        preceding_vehicle_stopping_distance = lane.effective_stopping_distance(
                            1, progress, preceding_vehicle_stopping_distance)
                elif in_progress[clone].rear is not None:
                    progress = in_progress[clone].rear
                else:
                    raise ValueError("Can't fetch p, clone has already exited")

                # Update the clone's speed and acceleration.
                assert progress is not None
                a = lane.accel_update_following(clone, progress,
                                                stopping_distance=preceding_vehicle_stopping_distance)
                su = lane.speed_update(clone, progress, a)
                clone.velocity = su.velocity
                clone.acceleration = su.acceleration

                # Update preceding data for the next cycle.
                preceding_vehicle_progress = downstream_progress[clone].rear
                preceding_vehicle_stopping_distance = clone.stopping_distance()

            # There is at most one vehicle on the incoming lane, and it will
            # always be at least partially in the intersection lane, so no
            # speed update necessary.

            # Progress clones on the downstream lane.
            last_p: float = 1.1
            to_remove: List[Vehicle] = []
            for clone in downstream_clones:
                # Update the clone's progress in the downstream road lane,
                # binning the transfers.
                (downstream_progress[clone], last_p, _
                 ) = downstream_lane.update_vehicle_progress(
                     clone, downstream_progress[clone], last_p)

                # Check if the vehicle has started to exit the lane. If it
                # starts exiting at all, just disappear it entirely.
                if downstream_progress[clone].front is None:
                    if clone in in_progress:
                        raise RuntimeError("Clone is exiting the downstream "
                                           "lane before it's left the "
                                           "intersection. Downstream road "
                                           "might be too short.")
                    del downstream_progress[clone]
                    to_remove.append(clone)

                # Update the clone's position using the downstream road lane if
                # it's partially in the intersection but its center isn't.
                # Update its reservation with its used tiles as well.
                if clone in in_progress:
                    progress = downstream_progress[clone].center
                    if progress is not None:
                        downstream_lane.update_vehicle_position(
                            clone, progress)
                        tiles_used = self.pos_to_tiles(lane, test_t, clone,
                                                       in_progress[clone],
                                                       test_reservations[clone
                                                                         ],
                                                       mark=mark)
                        if tiles_used is not None:
                            test_reservations[clone].tiles[test_t] = tiles_used
                        else:
                            # No test reservation is valid, so scrap them all
                            # and just return the complete list after removing
                            # the dependency on the last valid reservation if
                            # it exists.
                            if len(valid_reservations) > 0:
                                valid_reservations[-1].dependency = None

                            # Remove tile markings associated with invalid
                            # test_reservations if they had any.
                            if mark:
                                for r in test_reservations.values():
                                    self.clear_marked_tiles(r)

                            return valid_reservations

            # Finalize removals from the downstream lane.
            for clone in to_remove:
                downstream_clones.remove(clone)

            # Progress clones in the intersection lane.
            last_p = 1.1
            to_remove = []
            for i, clone in enumerate(in_clones):
                # Update the clone's progress in the intersection.
                (in_progress[clone], last_p, transfers
                 ) = lane.update_vehicle_progress(clone, in_progress[clone],
                                                  last_p)

                # Check if the vehicle has fully exited the intersection.
                if all((p is None) for p in in_progress[clone]):
                    # If so, check if its edge buffer tiles are usable.
                    reservation: Reservation = test_reservations[clone]
                    edge_buffer_tiles = self.edge_tile_buffer(
                        lane, test_t, clone, reservation, prepend=False,
                        mark=mark)
                    if edge_buffer_tiles is not None:
                        # Attach tiles and finalize the reservation.
                        reservation.tiles = {**reservation.tiles,
                                             **edge_buffer_tiles}
                        del in_progress[clone]
                        valid_reservations.append(test_reservations[clone])
                        del test_reservations[clone]
                        to_remove.append(clone)
                    else:
                        # No test reservation is valid, so scrap them all
                        # and just return the complete list after removing the
                        # dependency on the last valid reservation if it exists
                        if len(valid_reservations) > 0:
                            valid_reservations[-1].dependency = None

                        # Remove tile markings associated with invalid
                        # test_reservations if they had any.
                        if mark:
                            for r in test_reservations.values():
                                self.clear_marked_tiles(r)

                        return valid_reservations

                # Move vehicle sections onto the downstream road lane on exit.
                center_transitioned: bool = False
                for transfer in transfers:
                    if clone not in downstream_progress:
                        downstream_clones.append(clone)
                        downstream_progress[clone] = VehicleProgress()
                    downstream_progress[clone] = downstream_lane.\
                        transfer_to_progress(transfer,
                                             downstream_progress[clone])
                    if transfer.section is VehicleSection.CENTER:
                        # The clone's center section transitioned and its
                        # position must be updated by the downstream lane.
                        progress = downstream_progress[clone].center
                        assert progress is not None
                        downstream_lane.update_vehicle_position(
                            clone, progress)
                        center_transitioned = True

                # Update its position if its center is in the intersection.
                progress = in_progress[clone].center
                if progress is not None:
                    lane.update_vehicle_position(clone, progress)

                # Update its reservation with its used tiles if its position
                # was calculated in this loop.
                if (progress is not None) or center_transitioned:
                    tiles_used = self.pos_to_tiles(lane, test_t, clone,
                                                   in_progress[clone],
                                                   test_reservations[clone],
                                                   mark=mark)
                    if tiles_used is not None:
                        test_reservations[clone].tiles[test_t] = tiles_used
                    else:
                        # Scrap all test_reservations after this one and stop
                        # spawning new ones.

                        # Figure out where to look to decouple from the
                        # preceding reservation.
                        if i == 0:
                            # We don't just have to decouple from the preceding
                            # reservation, we know for sure that no test
                            # reservation is valid, so we can scrap them all
                            # and just return the valid reservations list.

                            if len(valid_reservations) > 0:
                                # Need to look in valid_reservations for the
                                # prior reseration to decouple.
                                valid_reservations[-1].dependency = None

                            # Remove tile markings associated with invalid
                            # test_reservations if they had any.
                            if mark:
                                for r in test_reservations.values():
                                    self.clear_marked_tiles(r)

                            return valid_reservations
                        else:
                            # Decouple from the prior reservation.
                            test_reservations[in_clones[i-1]].dependency = None

                        # Delete the clones after this one, their progress, and
                        # remove their marked tiles.
                        to_remove += in_clones[i:]
                        for clone_to_del in in_clones[i:]:
                            if mark:
                                self.clear_marked_tiles(
                                    test_reservations[clone_to_del])
                            del originals[clone_to_del]
                            del test_reservations[clone_to_del]
                            del in_progress[clone_to_del]
                        incoming_progress = None

                        # Stop spawning new clones.
                        counter = end_at

                        # Break out of the loop since we don't need to update
                        # the positions of any following clones (but we still
                        # need to continue looping to verify that reservations
                        # ahead of this one are valid).
                        break

            # Finalize removals from the intersection lane.
            for clone in to_remove:
                in_clones.remove(clone)

            # Progress the clone in the incoming road lane forward, if there is
            # one there. Transfer exiting section(s) to the intersection lane.
            if incoming_progress is not None:
                clone = in_clones[-1]

                # Need to determine if and how to update clone position.
                # There are three cases:
                #   1. Center started and ended in intersection.
                #      Position and tile update handled earlier.
                #   2. Center started in road and ended in intersection.
                #      Position update to be done but handled by intersection.
                #   3. Center started and ended in road.
                #      Position update to be done, handled by road.
                center_started_in_road: bool = (incoming_progress.center
                                                is not None)
                pos_updated: bool = False

                (incoming_progress, _, transfers
                 ) = incoming_lane.update_vehicle_progress(
                    clone, incoming_progress, 1.1)

                # Handle the forward transfer of any sections exiting the road
                # lane into the intersection
                for transfer in transfers:
                    # Note: Clones spawn partially in the intersection, so
                    #       there's no need to create an entry for it.
                    in_progress[clone] = lane.transfer_to_progress(
                        transfer, in_progress[clone])

                    if transfer.section is VehicleSection.CENTER:
                        # Case 2. Update vehicle position using intersection.
                        progress = in_progress[clone].center
                        assert progress is not None
                        lane.update_vehicle_position(clone, progress)
                        pos_updated = True

                if center_started_in_road and (not pos_updated):
                    # Case 3. Update vehicle position using road.
                    progress = incoming_progress.center
                    assert progress is not None
                    incoming_lane.update_vehicle_position(clone, progress)

                # If the clone has fully entered the intersection, delete its
                # incoming progress, find the scheduled exit of its rear
                # section, and update the clone's reservation with it.
                if all((p is None) for p in incoming_progress):
                    last_exit = ScheduledExit(vehicle=originals[clone],
                                              section=VehicleSection.REAR,
                                              t=test_t, v=clone.velocity)
                    test_reservations[clone].its_exit = last_exit
                    incoming_progress = None

                if center_started_in_road:
                    # Case 2 and 3. Find the tiles used by this clone.
                    tiles_used = self.pos_to_tiles(lane, test_t, clone,
                                                   in_progress[clone],
                                                   test_reservations[clone],
                                                   mark=mark)
                    if tiles_used is not None:
                        test_reservations[clone].tiles[test_t] = tiles_used
                    else:
                        # Scrap this reservation and stop spawning new ones.
                        # This is the last test reservation so there are no
                        # others to delete.

                        # Figure out where to look to decouple from the
                        # preceding reservation.
                        if in_clones[0] is clone:
                            # This is the only clone with an incomplete
                            # reservation.

                            if len(valid_reservations) > 0:
                                # Need to look in valid_reservations for the
                                # prior reseration to decouple.
                                valid_reservations[-1].dependency = None

                            # Remove tile markings associated with invalid
                            # test_reservations.
                            if mark:
                                self.clear_marked_tiles(
                                    test_reservations[clone])

                            return valid_reservations
                        else:
                            # Decouple from the prior reservation.
                            test_reservations[in_clones[-1]].dependency = None

                        # Delete this clone and its records.
                        to_remove.append(clone)
                        if mark:
                            self.clear_marked_tiles(test_reservations[clone])
                        del originals[clone]
                        del test_reservations[clone]
                        del in_progress[clone]
                        incoming_progress = None

                        # Stop spawning new clones.
                        counter = end_at

            # Try to spawn the next clone during the same timestep as when the
            # last one fully enters.
            if incoming_progress is None:

                # Check if there's a clone ready to enter (or if we already
                # found one and are just waiting for its time to enter).
                if (counter < end_at) and (new_exit is None):
                    new_exit = incoming_lane.soonest_exit(counter, last_exit)

                # Check if it's time for new_exit to enter the intersection.
                # TODO: (runtime) Should this be an equality check?
                if (new_exit is not None) and (new_exit.t >= test_t):
                    # It's time for this vehicle to enter.
                    # Fetch the new exit's progress and transfer.
                    (incoming_progress, transfers
                     ) = incoming_lane.progress_at_exit(counter, new_exit)

                    # Grab the original vehicle, its testing clone, and the
                    # ordered list of all clones ahead of it in sequence.
                    original: Vehicle = new_exit.vehicle
                    clone = transfers[0].vehicle
                    prior_clones: List[Vehicle] = downstream_clones + in_clones

                    # Initialize its reservation. We can dump this later if it
                    # turns out adding this vehicle doesn't work.
                    reservation = Reservation(
                        vehicle=original,
                        res_pos=lane.trajectory.start_coord,
                        tiles={},
                        lane=lane,
                        dependent_on=set(
                            originals[c] for c in prior_clones),
                        dependency=None,
                        its_exit=new_exit
                    )

                    # Find the progress of the clone in the intersection.
                    for transfer in transfers:
                        progress: VehicleProgress = lane.transfer_to_progress(
                            transfer)

                    # Update this clone's position.
                    if incoming_progress.center is not None:
                        incoming_lane.update_vehicle_position(
                            clone, incoming_progress.center)
                    elif in_progress[clone].center is not None:
                        progress = in_progress[clone].center
                        assert progress is not None
                        lane.update_vehicle_position(clone, progress)
                    else:
                        raise RuntimeError("Can't find vehicle center.")

                    # Check if its tiles work.
                    tiles_used = self.pos_to_tiles(lane, test_t, clone,
                                                   progress,
                                                   reservation,
                                                   mark=mark)
                    edge_tiles_used = self.edge_tile_buffer(lane, test_t,
                                                            clone,
                                                            reservation,
                                                            prepend=True,
                                                            mark=mark)
                    if ((edge_tiles_used is not None)
                            and (tiles_used is not None)):
                        # Register these edge buffer tiles and regular tiles to
                        # the new reservation, which has no tiles used yet.
                        reservation.tiles = edge_tiles_used
                        reservation.tiles[test_t] = tiles_used

                        # Register this vehicle as the dependency of its
                        # preceding clone, if there is one.
                        if len(in_clones) > 0:
                            # The last reservation hasn't been confirmed as
                            # valid yet, so look in test_reservations.
                            test_reservations[in_clones[-1]
                                              ].dependency = original
                        elif len(valid_reservations) > 0:
                            # The last reservation has been confirmed as valid,
                            # so look in valid_reservations.
                            valid_reservations[-1].dependency = original
                        # Otherwise, this is the first reservation, so there's
                        # no prior reservation to be dependent on.

                        # Add the new clone to the test data structures.
                        in_clones.append(clone)
                        originals[clone] = original
                        in_progress[clone] = progress
                        test_reservations[clone] = reservation

                        # Move the clone tracker up.
                        counter += 1

                        # Once done, clear for the next clone to add.
                        new_exit = None
                    else:
                        # Scrap this reservation and stop spawning new ones.
                        # We didn't register this clone to most of the usual
                        # structures, so there's less to do than usual.

                        if len(in_clones) == 0:
                            # No test reservation is valid, so scrap them all
                            # and just return the valid reservations list.
                            return valid_reservations

                        # Delete this clone's progress
                        incoming_progress = None

                        # Stop spawning new clones
                        counter = end_at

            # Increment the timesteps ahead counter
            test_t += 1

        return valid_reservations

    @abstractmethod
    def pos_to_tiles(self, lane: IntersectionLane, t: int,
                     clone: Vehicle, progress: VehicleProgress,
                     reservation: Reservation,
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

        timesteps_from_now = t - SHARED.t

        # If the tilings created so far doesn't cover this future timestep yet,
        # keep creating new layers until we reach this timestep.
        while len(self.tiles) < timesteps_from_now:
            self._add_new_layer()

        # Find the tiles this vehicle is estimated to use at this timestep by
        # using the clone's properties and proportional progress along the
        # intersection trajectory. Mark the tiles used by this reservation if
        # mark is True. Ignore feasibility if force is True.
        raise NotImplementedError("TODO")
        # TODO: Expand the number of tiles based on the vehicle's throttle
        #       and tracking scores. Use tile.will_reservation_work() if not
        #       force and tile.mark() if mark.
        # TODO: (sequence) If the vehicle is following another in a sequence,
        #       usage of a tile by a preceding vehicle shouldn't disqualify a
        #       reservation. Tweak tile implementation to account for this.

    @abstractmethod
    def edge_tile_buffer(self, lane: IntersectionLane, t: int,
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
        if force and mark:
            raise ValueError("Can't force and mark tiles at the same time.")
        raise NotImplementedError("TODO")

    @abstractmethod
    def clear_marked_tiles(self, reservation: Reservation) -> None:
        """Clear tiles marked with this reservation before discarding."""
        raise NotImplementedError("Must be implemented in child classes.")
        # TODO: (auction) Use tile.remove_mark(reservation)

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
        self.issue_permission(vehicle, lane, reservation.its_exit)

    @abstractmethod
    def find_best_batch(self,
                        requests: Dict[RoadLane, Iterable[Reservation]]
                        ) -> Iterable[Tuple[RoadLane, Reservation]]:
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

    @abstractmethod
    def clear_potential_reservations(self) -> None:
        """Go through all tiles and clear potential reservations markings."""
        raise NotImplementedError("Should be implemented in child classes.")
        # TODO: (auction) Use tile.clear_all_marks()

    # Support methods used by the Tiling

    @abstractmethod
    def _add_new_layer(self) -> None:
        """Should extend the tiling stack one more timestep.

        Tiling depth is created as needed. Calling this function will create a
        new layer in the tiling process.
        """
        raise NotImplementedError("Must be implemented in child classes.")


class ArcTiling(Tiling):

    # TODO: (arc) Points at the edge of the intersection should be considered a
    #       conflict object even if they aren't literally a conflict point,
    #       so we can enforce spacing between vehicles.

    def __init__(self,
                 upstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 downstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Dict[Coord, IntersectionLane],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tile_type: Type[Tile] = Tile,
                 cycle: Optional[List[
                     Tuple[Set[IntersectionLane], int]
                 ]] = None
                 ) -> None:
        super().__init__(
            upstream_road_lane_by_coord=upstream_road_lane_by_coord,
            downstream_road_lane_by_coord=downstream_road_lane_by_coord,
            lanes=lanes,
            lanes_by_endpoints=lanes_by_endpoints,
            tile_type=tile_type,
            cycle=cycle
        )
        # TODO: (arc) Does ArcTiling need a buffer argument? Anything else?
        raise NotImplementedError("TODO")

        # TODO: (arc) Old code, fix or replace.
        # # find conflicts
        # for l1, l2 in itertools.combinations(self.intersection_lanes, 2):
        #     t1, t2, point = l1.trajectory.get_intersection_point(l2.trajectory)
        #     if point == None:
        #         continue
        #     angle = l1.trajectory.get_intersection_angle(l2.trajectory, t1, t2)
        #     self.conflicts.add(ConflictRegion(l1, l2, point, angle, t1, t2))

    # class ConflictRegion:

    #     def __init__(self, traj1, traj2, point, angle, t1, t2):
    #         self.point = point
    #         self.angle = angle
    #         self.traj1 = traj1
    #         self.traj2 = traj2
    #         self.t1 = t1
    #         self.t2 = t2

    #     def get_conflict_region(self, vehicle):
    #         pass


class SquareTiling(Tiling):
    def __init__(self,
                 upstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 downstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Dict[Coord, IntersectionLane],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tile_type: Type[Tile] = Tile,
                 cycle: Optional[List[
                     Tuple[Set[IntersectionLane], int]
                 ]] = None,
                 granularity: Optional[int] = None
                 ) -> None:
        super().__init__(
            upstream_road_lane_by_coord=upstream_road_lane_by_coord,
            downstream_road_lane_by_coord=downstream_road_lane_by_coord,
            lanes=lanes,
            lanes_by_endpoints=lanes_by_endpoints,
            tile_type=tile_type,
            cycle=cycle
        )
        # TODO: (square) Use the granularity input and the start and end points
        #       of every IntersectionLane to create a grid of tiles.
        raise NotImplementedError("TODO")
