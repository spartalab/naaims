"""
The lanes module handles the progression of vehicles along 1-dimensional lines,
"lanes", used on roads and within intersections (e.g., as a left turn
trajectory), making them the fundamental building block of aimsim.

Vehicles in lanes look forward to make sure that they don't collide with the
vehicle ahead of them even if the preceding vehicle immediately starts braking
as hard as it can. This is true even across road lanes to intersection lanes
and vice versa.

The geometry of a lane is determined by what type of Trajectory it's provided
at init. It determines how vehicles move in 2D space.
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import (Tuple, Iterable, Optional, List, Dict, Set,
                    NamedTuple)
from warnings import warn

import aimsim.shared as SHARED
from aimsim.archetypes import Upstream
from aimsim.util import (Coord, CollisionError, VehicleSection, SpeedUpdate,
                         TooManyProgressionsError, VehicleTransfer)
from aimsim.vehicles import Vehicle
from aimsim.trajectories import Trajectory
from aimsim.intersections import Intersection


class LateralDeviation:
    """Describes how far a vehicle has deviated perpendicular to a lane.

    lane: Lane
        The lane in which this deviation is in reference to. Useful in the case
        of lane changes where a vehicle may be in two lanes at once.
    d: float
        How far the vehicle is perpendicular to the lane's trajectory.
    """
    lane: Lane
    d: float


class VehicleProgress(NamedTuple):
    """Track the progress of all three sections of a vehicle.

    Note that the index 0, 1, 2 of these sections in this NamedTuple
    correspond to the values of the section in the enum VehicleSection.
    """
    front: Optional[float] = None
    center: Optional[float] = None
    rear: Optional[float] = None


class Lane(ABC):

    @abstractmethod
    def __init__(self,
                 trajectory: Trajectory,
                 width: float,
                 speed_limit: float = SHARED.speed_limit) -> None:
        """Should create a new lane.

        Parameters
            trajectory: Trajectory
                The trajectory of the road that contains the lane.
            width: float
                The width of the lane.
            speed_limit: float = SHARED.speed_limit
                The speed limit for the lane.
        """

        # Save inputs
        self.trajectory = trajectory
        self.width = width
        self.speed_limit = speed_limit

        # Initialize data structures
        self.vehicles: List[Vehicle] = []
        self.vehicle_progress: Dict[Vehicle, VehicleProgress] = {}
        self.hash: int

        raise NotImplementedError("TODO")

    # Support functions for speed updates

    def effective_speed_limit(self, p: float, vehicle: Vehicle) -> float:
        """Return the effective speed limit at the given progression.

        Some trajectories may require a lower speed limit, e.g. on sharper
        turns. Overridden in IntersectionLane to prevent crashes just
        downstream when control is done by signals instead of by reservation.

        Parameters
        p: float
            Progress to use on the intersection trajectory to calculate the
            effective speed limit (e.g. on tight turns). (This is provided
            instead of vehicle because the vehicle has front, middle, and rear
            progress values so it would be ambiguous which one to use.)
        vehicle: Vehicle
            (Used only by IntersectionLane) Provide the vehicle to allow us to
            adjust the effective speed limit based on its characteristics.
            TODO: (stochasticity) remove if unused.
        """
        return min(self.speed_limit, self.trajectory.effective_speed_limit(p))

    def update_speeds(self, to_slow: Set[Vehicle] = set()
                      ) -> Dict[Vehicle, SpeedUpdate]:
        """Return all vehicles on this lane and their speed updates.

        Note that this function does NOT change a vehicle's own record of its
        speed and acceleration. That should be handled outside this function.

        Parameters
            to_slow: Set[Vehicle]
                A set of vehicles for which to override lane following behavior
                and slow down to allow for another vehicle to merge in.
        """
        new_speed: Dict[Vehicle, SpeedUpdate] = {}

        # Track preceding vehicle in order to avoid colliding with it.
        preceding: Optional[Vehicle] = None
        # self.vehicles should be in order of decreasing progress
        for vehicle in self.vehicles:
            vehicle_in_jurisdiction, p, section = self.controls_this_speed(
                vehicle)
            if vehicle_in_jurisdiction:  # update its speed
                # A vehicle being in to_slow overrides any acceleration logic
                # defined in accel_update, instead telling the vehicle to start
                # braking no matter what.

                # TODO: (runtime) Due to the seams between intersections and
                #       roads, the acceleration behavior during the window
                #       where a leading vehicle transfers but a following
                #       vehicle hasn't may be an issue. The following vehicle,
                #       not seeing any vehicles ahead, will floor it. If this
                #       following vehicle's acceleration is much higher than
                #       both its braking ability and the preceding vehicle's
                #       acceleration, this could be an issue.
                a_new = (vehicle.max_braking if (vehicle in to_slow)
                         else self.accel_update(vehicle, section, p, preceding)
                         )
                new_speed[vehicle] = self.speed_update(vehicle, p, a_new)

            preceding = vehicle

        return new_speed

    @abstractmethod
    def controls_this_speed(self, vehicle: Vehicle) -> Tuple[bool, float,
                                                             VehicleSection]:
        """Should return if the lane controls this vehicle and other data."""
        raise NotImplementedError("Must be implemented in child class.")

    @abstractmethod
    def accel_update(self, vehicle: Vehicle, section: VehicleSection, p: float,
                     preceding: Optional[Vehicle]) -> float:
        """Should return a vehicle's new acceleration.

        Note that this function does NOT change a vehicle's own record of its
        acceleration. That should be handled outside this function.

        preceding is the vehicle ahead of the vehicle we're calculating for. If
        there is none, the vehicle is at the head of the queue and must stop at
        the intersection line if it doesn't have permission to enter the
        intersection.
        """

        stopping_distance: float

        if preceding is None:
            # Call downstream to see if there's a preceding vehicle across the
            # road/intersection seam.
            downstream_stopping_distance: Optional[float] = \
                self.downstream_stopping_distance(vehicle, section)

            if downstream_stopping_distance is None:
                # There's nothing to stop for. Full speed forward.
                return self.accel_update_uncontested(vehicle, p)
            elif section is VehicleSection.FRONT:
                # The stopping distance is the sum of the downstream stopping
                # distance plus the length left in this lane.
                stopping_distance = downstream_stopping_distance + \
                    (1-p)*self.trajectory.length
            elif section is VehicleSection.REAR:
                # The stopping distance is just what we have downstream because
                # it's straddling the seam.
                stopping_distance = downstream_stopping_distance
            else:
                raise ValueError("Did not receive front or rear section p.")
        else:
            if section is not VehicleSection.FRONT:
                raise ValueError("Not given vehicle's front progress even "
                                 "though there is a preceding vehicle.")

            # Prepare to lane-follow the preceding vehicle in this lane.
            preceding_vehicle_progress = self.vehicle_progress[preceding].rear
            if preceding_vehicle_progress is None:
                raise ValueError("Preceding vehicle not in lane.")
            else:
                stopping_distance = self.effective_stopping_distance(
                    preceding_vehicle_progress, p,
                    preceding.stopping_distance())
        return self.accel_update_following(vehicle, p,
                                           stopping_distance=stopping_distance)

    def effective_stopping_distance(self, pre_p: float, p: float,
                                    vehicle_stopping_distance: float) -> float:
        """Return the effective stopping distance.

        Calculates the effective stopping distance for a vehicle given its
        gap to the next waypoint plus the stopping distance of that waypoint
        if it's a vehicle. (This is 0 if the waypoint is the end of the lane.)

        Parameters
            pre_p: float
                The proportional progress of the rear of the preceding vehicle.
            p: float
                The proportional progress of the front of the current vehicle.
            vehicle_stopping_distance: float
                The stopping distance of the preceding vehicle.
        """
        return (pre_p - p)*self.trajectory.length + vehicle_stopping_distance

    @abstractmethod
    def downstream_stopping_distance(self, vehicle: Vehicle,
                                     section: VehicleSection
                                     ) -> Optional[float]:
        """Should check the downstream object's required stopping distance."""
        raise NotImplementedError("Must be implemented in child classes.")

    def stopping_distance_to_last_vehicle(self) -> Optional[float]:
        """Return the distance before collision with the last vehicle, if any.

        We assume that this distance is the stopping distance of the last
        vehicle plus how far it is along the trajectory.
        """
        if len(self.vehicles) == 0:
            return None
        else:
            vehicle: Vehicle = self.vehicles[-1]
            pre_p: Optional[float] = self.vehicle_progress[vehicle].rear
            p: Optional[float] = 0
            if pre_p is None:
                # This vehicle is the vehicle we're checking the stopping
                # distance for. Check if there's another vehicle ahead of it.
                if len(self.vehicles) > 1:
                    p = self.vehicle_progress[vehicle].front
                    vehicle = self.vehicles[-2]
                    pre_p = self.vehicle_progress[vehicle].rear
                else:
                    # There's no vehicle ahead of this one. The lane is free.
                    return None
            assert p is not None
            assert pre_p is not None
            return self.effective_stopping_distance(
                pre_p, p, vehicle.stopping_distance())

    def accel_update_uncontested(self, vehicle: Vehicle, p: float) -> float:
        """Return accel update if there are no vehicles ahead.

        p is the proportional progress associated with this vehicle. Because it
        could be the front or rear of the vehicle depending on the situation,
        it's presented here as an input argument.
        """
        effective_speed_limit = self.effective_speed_limit(p, vehicle)
        if vehicle.velocity > effective_speed_limit:
            return vehicle.max_braking
        elif vehicle.velocity == effective_speed_limit:
            return 0
        else:  # vehicle.v < effective_speed_limit
            return vehicle.max_acceleration

    def accel_update_following(self,
                               vehicle: Vehicle,
                               p: float,
                               stopping_distance: Optional[float] = None
                               ) -> float:
        """Return speed update to prevent collision with a preceding object.

        p is the proportional progress associated with this vehicle. Because it
        could be the front or rear of the vehicle depending on the situation,
        it's presented here as an input argument.

        stopping_distance is how much distance the vehicle has to come to a
        complete stop, e.g., if the preceding vehicle stops braking this
        timestep, how much room does this vehicle have to brake before it
        collides with the one in front? If no value is provided this method
        assumes that there are no vehicles preceding this one and defaults to
        calculating the stopping distance as the length of lane left ahead of
        this vehicle.
        """

        sd = ((1-p)*self.trajectory.length if stopping_distance is None
              else stopping_distance)

        effective_speed_limit = self.effective_speed_limit(p, vehicle)

        # TODO: when implementing, take into account that time is discrete so
        #       round down when spacing.

        a_maybe = self.accel_update_uncontested(vehicle, p)
        if a_maybe < 0:  # need to brake regardless of closeness
            return a_maybe
        else:
            # TODO: Calculate the stopping distance of this vehicle assuming
            #       it accelerates to get closer to or is already at the speed
            #       limit. If this distance is longer than sd, brake. If not,
            #       accelerate or brake as was calculated.
            raise NotImplementedError("TODO")

    def speed_update(self, vehicle: Vehicle, p: float,
                     accel: float) -> SpeedUpdate:
        """Given a vehicle and its acceleration, update speed and return both.

        Note acceleration and speed aren't 1-to-1 because of discrete time.

        p is the proportional progress associated with this vehicle. Because it
        could be the front or rear of the vehicle depending on the situation,
        it's presented here as an input argument.
        """
        v_new = vehicle.velocity + accel*SHARED.TIMESTEP_LENGTH
        if v_new < 0:
            return SpeedUpdate(velocity=0, acceleration=accel)
        else:
            effective_speed_limit = self.effective_speed_limit(p, vehicle)
            if v_new > effective_speed_limit:
                return SpeedUpdate(velocity=effective_speed_limit,
                                   acceleration=accel)
            else:
                return SpeedUpdate(velocity=v_new, acceleration=accel)
        # TODO: Consider enforcing the speed limit clip in accel_update instead
        #       of here to make perturbing stochastic speed and acceleration
        #       easier. Will need to double check for functions that assume
        #       only 3 possible acceleration values instead of
        #       accounting for a continuous range.

    # Support functions for stepping vehicles

    def step_vehicles(self,
                      lateral_deviations: Dict[Vehicle, LateralDeviation] = {}
                      ) -> Iterable[VehicleTransfer]:
        """Execute position step for all vehicles in this lane.

        lateral_movements contains precalculated lateral movements for vehicles
        in this lane. If a lateral movement is not provided for a vehicle, the
        lane will calculate the deviation itself.
        """

        # Track exiting vehicle sections we want to return to the road and
        # fully exited vehicles we need to remove from this lane.
        exiting: List[VehicleTransfer] = []
        to_remove: List[Vehicle] = []

        # Track our progression backwards through the lane.
        last_progress: float = 1.1  # Max (non-None) progress is 1.
        # self.vehicles should be in order of decreasing progress
        for vehicle in self.vehicles:

            new_progress_packet = self.update_vehicle_progress(
                vehicle=vehicle,
                old_progress=self.vehicle_progress[vehicle],
                last_progress=last_progress
            )
            new_vehicle_progress = new_progress_packet[0]
            last_progress = new_progress_packet[1]
            exiting += new_progress_packet[2]

            # Check if this vehicle has fully exited.
            if self.has_vehicle_exited(new_vehicle_progress):
                # If so, mark it for removal from this lane.
                to_remove.append(vehicle)
            else:
                # Otherwise update vehicle_progress with the new values
                self.vehicle_progress[vehicle] = new_vehicle_progress

            # If the center of the vehicle is in this lane, update its pos
            if new_vehicle_progress.center is not None:

                # Check if this is the lane responsible for updating this
                # vehicle's lateral deviation. (A vehicle can be present in
                # multiple lanes during a lane change.)
                if ((vehicle in lateral_deviations) and
                        (self is not lateral_deviations[vehicle].lane)):
                    # If not, skip the update.
                    continue

                lateral: float = 0
                if ((vehicle in lateral_deviations)
                        and (self is lateral_deviations[vehicle].lane)):
                    # We have a precalculated lateral movement for this
                    # vehicle and it's defined as relative to this lane.
                    # This lateral movement is relative to this lane,
                    # so we're ok to include it in our position update.
                    lateral = lateral_deviations[vehicle].d
                else:
                    # We need to infer this vehicle's lateral movement.
                    lateral = self.lateral_deviation_for(
                        vehicle,
                        new_vehicle_progress.center
                    )

                # Update the vehicle's own position Coord.
                self.update_vehicle_position(
                    vehicle,
                    new_vehicle_progress.center,
                    lateral
                )

        # Remove the vehicles marked as exiting from this lane.
        for vehicle in to_remove:
            self.remove_vehicle(vehicle)

        return exiting

    def update_vehicle_progress(self, vehicle: Vehicle,
                                old_progress: VehicleProgress,
                                last_progress: float = 1.1
                                ) -> Tuple[VehicleProgress, float,
                                           List[VehicleTransfer]]:
        """Step a single vehicle forward and track its new VehicleProgress.

        Parameters:
            vehicle: Vehicle
            old_progress: VehicleProgress
                The vehicle's progress at the last timestep.
            last_progress: int = 1.1
                The progress of the preceding vehicle's rear.
                (1.1 if no preceding.)

        Returns a tuple of
            VehicleProgress
                The vehicle's progress at this timestep.
            float
                The proportional progress of the rear section of this vehicle.
            List[VehicleTransfer]
                A list of the exiting sections of this vehicle (may be empty).
        """
        # Track exiting sections.
        exiting: List[VehicleTransfer] = []

        # Convert the VehicleProgress tuple into a List for indexability.
        new_vehicle_progress: List[Optional[float]] = [None]*3

        # Iterate through the 3 sections of the vehicle.
        for i, progress in enumerate(old_progress):

            # Check if we have a progress record for this section.
            if progress is None:
                # This section of the vehicle isn't in this lane. Check if this
                # is a valid case or not. If it is, skip the progress update
                # for this section. If not, error.
                if last_progress > 1:
                    # We're on the downstream end of the lane.
                    if (VehicleSection(i) in {VehicleSection.FRONT,
                                              VehicleSection.CENTER}):
                        # This vehicle front or center section is already in
                        # the downstream object. Nothing to do here.
                        new_vehicle_progress[i] = None
                        continue
                    else:  # this is the rear of the vehicle
                        # If the rear is None, the vehicle should have
                        # fully exited and be out of this lane.
                        raise RuntimeError("Exited vehicle not removed.")
                elif VehicleSection(i) is VehicleSection.FRONT:
                    # last_progress is telling us that we've already looked at
                    # at least one vehicle, but if this next vehicle's front
                    # section is still None that means that it's the first
                    # vehicle in the lane and already halfway into the next
                    # object. This shouldn't be possible.
                    raise RuntimeError("The front section of the second "
                                       "Vehicle should not have started "
                                       "exiting already.")
                elif VehicleSection(i) is VehicleSection.CENTER:
                    if last_progress >= 0:
                        # The front of this vehicle is in the lane but this
                        # center section and the rear section are still in the
                        # upstream intersection. This is ok.
                        new_vehicle_progress[i] = progress
                        last_progress = -1
                        continue
                    else:
                        raise RuntimeError("The center of this vehicle isn't "
                                           "in this lane even though this "
                                           "isn't the first vehicle in the "
                                           "lane.")
                else:  # this is the rear of the vehicle
                    if last_progress >= 0:
                        # The front and center of this vehicle are in this lane
                        # but this rear is still in the upstream intersection.
                        # This is ok, but if anything comes after this vehicle
                        # setting last_progress like this will make sure to
                        # flag it as an issue.
                        last_progress = -1
                    # Otherwise, the center of this vehicle is also in the
                    # upstream intersection in addition to this rear. This is
                    # ok. (If this is not the case, e.g., if there's a vehicle
                    # after this one that's on this lane, this issue will be
                    # caught on the next step.)
                    new_vehicle_progress[i] = progress
                    continue
            elif progress > last_progress:
                warn("Vehicles overlap in-lane. This may be a collision.")

            # Update relative position.
            # TODO: Translate the real distance units vehicle.v*SHARED.timestep
            #       into proportional progress using trajectory. If it exceeds
            #       the length left in the lane, find the distance left to move
            #       d_left.
            new_progress: Optional[float]
            d_left: float = 0
            raise NotImplementedError("TODO")
            new_vehicle_progress[i] = new_progress

            if new_progress is None:
                # Vehicle section has exited. Create a VehicleTransfer object
                # from it and add it to the return list.
                exiting.append(VehicleTransfer(
                    vehicle=vehicle,
                    section=VehicleSection(i),
                    d_left=d_left,
                    pos=self.end_coord
                ))

            # Remember progress of this section for the next section's checks.
            last_progress = progress

        return VehicleProgress(
            front=new_vehicle_progress[0],
            center=new_vehicle_progress[1],
            rear=new_vehicle_progress[2]
        ), last_progress, exiting

    def lateral_deviation_for(self, vehicle: Vehicle,
                              new_progress: float) -> float:
        """Return the lateral movement for a vehicle. 0 unless overridden."""
        return 0

    def update_vehicle_position(self, vehicle: Vehicle, new_p: float,
                                lateral_deviation: float = 0) -> None:
        """Should update the vehicle's position, including lateral movement."""

        # longitudinal update
        pos: Coord = self.trajectory.get_position(new_p)
        heading: float = self.trajectory.get_heading(new_p)

        if lateral_deviation != 0:
            # TODO: (multiple) (stochasticity) use trajectory.heading(new_p) to
            #       calculate the lateral deviation away from pos.
            raise NotImplementedError("TODO")

        # write new values to vehicle
        vehicle.pos = pos
        vehicle.heading = heading

    def has_vehicle_exited(self, progress: VehicleProgress) -> bool:
        """Check if a vehicle has fully exited from the lane."""
        return all((p is None) for p in progress)

    def remove_vehicle(self, vehicle: Vehicle) -> None:
        """Remove an exited vehicle from this lane."""
        self.vehicles.remove(vehicle)
        del self.vehicle_progress[vehicle]

    # Support functions for vehicle transfers

    def enter_vehicle_section(self, transfer: VehicleTransfer) -> None:
        """Adds a section of a transferring vehicle to the lane."""
        vehicle: Vehicle = transfer.vehicle

        if transfer.section is VehicleSection.FRONT:
            # This is the first time this lane has seen this vehicle.
            # Create new entries for this vehicle in the lane's relevant data
            # structures and populate its front section progress.
            self.add_vehicle(vehicle)

        # Update the lane's record of the vehicle's position.
        self.vehicle_progress[vehicle] = self.transfer_to_progress(
            transfer, self.vehicle_progress[vehicle])

        if transfer.section is VehicleSection.CENTER:
            # Update the vehicle's own record of its actual position Coord.
            p = self.vehicle_progress[vehicle].center
            if p is not None:
                self.update_vehicle_position(vehicle, p)
            else:
                raise RuntimeError("Center progress is somehow None.")

    def transfer_to_progress(self, transfer: VehicleTransfer,
                             old_progress: VehicleProgress = VehicleProgress()
                             ) -> VehicleProgress:
        """Update a lane's relative record of a vehicle's progress.

        Parameters:
            transfer: VehicleTransfer
            old_progress: VehicleProgress = VehicleProgress()
                The vehicle's progress in the lane before this transfer
                resolved, e.g. if its front section was already in the lane
                last timestep but now its middle section is transferring.
        """
        vehicle: Vehicle = transfer.vehicle
        new_vehicle_progress: List[Optional[float]] = list(old_progress)

        # Find how far along the lane this vehicle section will be in meters.
        d: float
        if transfer.d_left is None:
            # This is a freshly created vehicle section entering from a spawner
            # so we need to initialize its position.
            if transfer.section is VehicleSection.FRONT:
                # Place the front vehicle section ahead at its full length plus
                # the length of its front and rear buffers.
                d = vehicle.length * (1 + 2*SHARED.length_buffer_factor)
            elif transfer.section is VehicleSection.CENTER:
                # Place the center vehicle section forward at half its length
                # plus its rear buffer.
                d = vehicle.length * (0.5 + SHARED.length_buffer_factor)
            else:  # transfer.section is VehicleSection.REAR
                # Place the rear of the vehicle at the very end of the lane.
                d = 0
        else:
            # This is a transfer between road and intersection or vice versa.
            # Calculate the distance the vehicle section has left to travel
            # in this step.
            d = transfer.d_left

        # Convert the real units distance d into proportional progress along
        # the lane trajectory and update the progress values.
        new_vehicle_progress[transfer.section.value] = d/self.trajectory.length

        return VehicleProgress(
            front=new_vehicle_progress[0],
            center=new_vehicle_progress[1],
            rear=new_vehicle_progress[2]
        )

    def add_vehicle(self, vehicle: Vehicle) -> None:
        """Create entries for this vehicle in lane support structures."""
        self.vehicles.append(vehicle)
        self.vehicle_progress[vehicle] = VehicleProgress()

    # Misc functions

    def __hash__(self) -> int:
        return self.hash


class ScheduledExit(NamedTuple):
    """
    This is when a vehicle is scheduled to either start exiting (if section is
    VehicleSection.FRONT) or finish exiting (if section is VehicleSection.REAR)
    the lane. The former case is used by intersection managers to determine if
    a vehicle can enter the intersection, while the latter is used by road
    lanes to make sure that reservations don't crash into a vehicle immediately
    downstream.

    Parameters
        vehicle: Vehicle
            The vehicle exiting.
        section: VehicleSection
            The vehicle section this exit is in reference to.
        t: int
            The timestep of the exit.
        v: float
            The speed at which the vehicle exits.
    """
    vehicle: Vehicle
    section: VehicleSection
    t: int
    v: float


class RoadLane(Lane):
    """RoadLanes connect intersections, providing them access and egress.

    Like their parent Roads, RoadLanes are divided into three sections: the
    entrance, lane changing, and approach regions.
    """

    def __init__(self,
                 trajectory: Trajectory,
                 width: float,
                 offset: Coord = Coord(0, 0),
                 len_entrance_region: float = SHARED.min_entrance_length,
                 len_approach_region: float = 100,
                 upstream_is_spawner: bool = False,
                 downstream_is_remover: bool = False,
                 speed_limit: int = SHARED.speed_limit) -> None:
        """Create a new road lane."""

        self.start_coord: Coord = Coord(trajectory.start_coord.x + offset.x,
                                        trajectory.start_coord.y + offset.y)
        self.end_coord: Coord = Coord(trajectory.end_coord.x + offset.x,
                                      trajectory.end_coord.y + offset.y)
        self.hash = hash(self.start_coord + self.end_coord)
        # TODO: Copy the provided trajectory, changing the start and end coords
        #       to match the provied offsets.
        raise NotImplementedError("TODO")

        super().__init__(trajectory=trajectory,
                         speed_limit=speed_limit)
        self.upstream_is_spawner: bool = upstream_is_spawner
        self.downstream_is_remover: bool = downstream_is_remover
        self.downstream_intersection: Optional[Intersection] = None

        # TODO: use len_entrance_region and len_approach_region to calculate
        #       the proportional cutoffs for the three regions, for use in the
        #       the speed update and step functions
        # Note that we start at the front of the lane and work back, so
        # proportions decrease as we go on.
        # self.approach_end: float = 1
        self.lcregion_end: float = 0.6
        self.entrance_end: float = 0.3

        # Cache the exit time and speed of the exit of the last vehicle with
        # permission to enter the intersection. This will be used for
        # estimating when the next vehicle looking for permission to enter the
        # intersection will reach the intersection calculations.
        self.latest_scheduled_exit: Optional[ScheduledExit] = None

        raise NotImplementedError("TODO")

        super().__init__(trajectory, width, speed_limit=speed_limit)

    def connect_downstream_intersection(self, downstream: Intersection
                                        ) -> None:
        """Connect a downstream intersection to this RoadLane."""
        if self.downstream_is_remover:
            raise ValueError("Downstream shouldn't be intersection.")
        else:
            self.downstream_intersection = downstream

    # Support functions for speed updates

    def update_speeds(self, to_slow: Set[Vehicle] = set()
                      ) -> Dict[Vehicle, SpeedUpdate]:
        """Return all vehicles on this lane and their speed updates.

        On top of the processes in the parent function, this does an additional
        check to see if a downstream intersection has been registered, if
        necessary.
        """
        if ((not self.downstream_is_remover)
                and (self.downstream_intersection is None)):
            raise RuntimeError("Missing downstream Intersection.")
        return super().update_speeds(to_slow=to_slow)

    def controls_this_speed(self, vehicle: Vehicle) -> Tuple[bool, float,
                                                             VehicleSection]:
        """Return if and what progress and section controls this Vehicle."""

        front = self.vehicle_progress[vehicle].front
        if front is None:
            # The front of the vehicle is not on this road lane.
            if self.downstream_is_remover:
                # There's a remover downstream so this road lane is still in
                # control. Report the rear of the vehicle.
                rear = self.vehicle_progress[vehicle].rear
                if rear is None:
                    raise RuntimeError("Vehicle exited lane already.")
                else:
                    return True, rear, VehicleSection.REAR
            else:
                # The front of the vehicle is in the next intersection, so this
                # road lane is not in control.
                return False, float("inf"), VehicleSection.REAR
        else:
            # The front of this vehicle is on this road.
            if self.upstream_is_spawner:
                # Front of the vehicle is on this road and its rear isn't in an
                # intersection, so this road is in control. Report the front.
                return True, front, VehicleSection.FRONT
            else:
                # The vehicles is still exiting the intersection and therefore
                # not in this road lane's jurisdiction.
                return False, -float("inf"), VehicleSection.FRONT

    def accel_update(self, vehicle: Vehicle, section: VehicleSection, p: float,
                     preceding: Optional[Vehicle]) -> float:
        """Return a vehicle's new acceleration.

        On top of the parent function's operations, adds additional cases to
        determine what to do if the vehicle needs to stop at the intersection
        line, for both there being and not being a preceding vehicle in lane.
        """

        # TODO: (platoon) Insert vehicle chain override here and return it.
        #       Vehicle should look forward to downstream chained vehicles to
        #       match the velocity and accel of all chained vehicles as soon as
        #       possible. Once matched, I think it ends up being just normal
        #       lane following behavior, unless the chain is exiting the
        #       intersection. If a vehicle in the chain is on the road but an
        #       upstream vehicle is still in the intersection, the downstream
        #       vehicle should mirror the speed and acceleration of the vehicle
        #       still in the intersection.

        if ((preceding is None) and
                (not vehicle.permission_to_enter_intersection)):
            # stop at the intersection line
            return self.accel_update_following(vehicle, p)

        a_follow = super().accel_update(vehicle=vehicle, section=section, p=p,
                                        preceding=preceding)

        if vehicle.permission_to_enter_intersection:
            # Follow preceding vehicle into the intersection
            return a_follow
        else:
            # Stop for preceding vehicle AND intersection line
            return min(a_follow, self.accel_update_following(vehicle, p))

    # Support functions for stepping vehicles

    def downstream_stopping_distance(self, vehicle: Vehicle,
                                     section: VehicleSection
                                     ) -> Optional[float]:
        """Check the downstream object's required stopping distance, if any."""
        if self.downstream_intersection is None:
            # There's a remover downstream so there's no possibility of having
            # to follow another vehicle.
            return None
        else:
            return self.downstream_intersection.\
                stopping_distance_to_last_vehicle(self.end_coord)

    def has_vehicle_exited(self, progress: VehicleProgress) -> bool:
        """Check if a vehicle has exited from road the lane.

        This check is a little more lenient if there's a remover downstream
        because the downstream has no way to update a vehicle's position, so
        as soon as the center leaves we delete the vehicle from the lane.
        """
        if self.downstream_is_remover:
            return ((progress.front is None) and (progress.center is None))
        else:
            return super().has_vehicle_exited(progress)

    def remove_vehicle(self, vehicle: Vehicle) -> None:
        """Remove an exited vehicle from this lane.

        On top of the super's parent call, clear the lane's latest scheduled
        exit record if this was actually the latest scheduled exit.
        """
        super().remove_vehicle(vehicle)

        # Clear the latest scheduled exit if this fulfills it.
        if ((self.latest_scheduled_exit is not None)
                and (vehicle is self.latest_scheduled_exit.vehicle)):
            self.latest_scheduled_exit = None

    # Used by spawner and/or intersection manager

    def room_to_enter(self, tight: bool = True) -> float:
        """Return the amount of free space left in the entrance region.

        If tight, check only the space immediately behind the current position
        of the last vehicle. This is used to check if there's room to spawn a
        vehicle.

        If not tight, return the total amount of free space in the entire
        entrance region, including in between vehicles, assuming that all
        vehicles currently in the entrance region have braked at maximum power
        (which will make even more free space available). This is used when
        deciding whether to allow new vehicles into the upstream intersection,
        as they'll need enough room to exit in a worst-case scenario.
        """

        if tight:
            if len(self.vehicles) > 0:
                last = self.vehicles[-1]
                p = self.vehicle_progress[last].rear
                if p is not None:
                    return min(self.entrance_end*self.trajectory.length,
                               p*self.trajectory.length)
                else:
                    return 0
            else:
                return self.entrance_end*self.trajectory.length
        else:
            # TODO: Assume that vehicle at the front of the entrance region
            #       brakes. The vehicles in the entrance region behind it
            #       should already be under lf behavior and be able to come to
            #       a stop.
            #       If it stops outside the entrance region, do the same for
            #       the next vehicle until one of them stops inside the
            #       entrance region.
            #       After the determining how much of the entrance region this
            #       vehicle occupies, subtract this value and the sum total
            #       lengths of the other vehicles in the entrance region from
            #       the total length of the entrance region and return this
            #       value.
            raise NotImplementedError("TODO")
            # TODO: (low) Consider caching this value, resetting it each step.

    def first_without_permission(self, targets: Optional[Set[Coord]] = None,
                                 sequence: bool = False
                                 ) -> Optional[Tuple[int, int]]:
        """Return start and end indices of the first vehicles without perms.

        Parameters
            targets: Optional[Set[Coord]]
                If provided, specifies what exit Coords in the IntersectionLane
                we're looking for.
            sequence: Optional[bool]
                If provided, indicates if we should return a range of indices
                corresponding to consecutive vehicles without permission to
                enter with the same desired movement.
        Note that these two parameters are not intended for use together.

        Returns
            None
                If there are no vehicles in this lane without permission to
                enter the intersection.
            Tuple[int, int]
                The first int is the index of the first vehicle in this lane's
                approach region without permission to enter.
                The second int is the same if sequence is False. If sequence is
                True, it is one plus the index of the last vehicle in the
                sequence of vehicles beginning with the vehicle at the first
                index of vehicles with the same desired movement through the
                intersection.
                This is intended for use as range(int, int).
        """

        if (targets is not None) and (sequence is not None):
            raise ValueError("Both target and sequence is not intended usage.")

        # TODO: (low) Support actually using the feature where a vehicle can
        #       have multiple valid target intersection exit lane.

        first_index: Optional[int] = None
        series_length: int = 0

        for i, vehicle in enumerate(self.vehicles):
            p = self.vehicle_progress[vehicle].front
            if p is None:
                # This is the first vehicle and it's exiting the lane. Ignore.
                continue

            # TODO: (low) Should this check be based on the front or end of
            #       the vehicle?
            if p < self.lcregion_end:
                # This vehicle is outside of the approach area. Done checking.
                break

            if ((first_index is None) and
                    (not vehicle.permission_to_enter_intersection)):
                # We found the first vehicle without permission.
                its_target = vehicle.next_movements(self.end_coord)[0]
                if (targets is not None) and (its_target not in targets):
                    # The first vehicle's movement doesn't match any of the
                    # movements we're targetting, so return None.
                    return None
                first_index = i
                series_length = 1
                if (targets is None) and sequence:
                    # We want to look for futher vehicles with the same move.
                    targets = {its_target}
                else:
                    # We've found a vehicle without permission but with a valid
                    # target movement. Exit loop and prepare to return.
                    break
            elif targets is not None:
                # Check if this vehicle has the same target as the first one.
                if vehicle.next_movements(self.end_coord)[0] in targets:
                    # It does, so the series is longer.
                    series_length += 1
                else:
                    # It doesn't, so there are no more consecutive vehicles
                    # with the same movement. Break out of the intersection.
                    break

        if first_index is None:
            return None
        else:
            return first_index, first_index+series_length

    def soonest_exit(self, vehicle_index: int,
                     last_rear_exit: Optional[ScheduledExit] = None
                     ) -> ScheduledExit:
        """Return the soonest exit this vehicle can make if starting at p.

        Assumes that the invoker has already checked that there is a vehicle
        in self.vehicles to check for a collision against with last_rear_exit.

        Parameters
            vehicle_index: int
                The index of the vehicle in self.vehicles to calculate for.
            last_rear_exit: Optional[ScheduledExit]
                The scheduled exit that we're calculating for this vehicle to
                avoid colliding with while exiting. If None, retrieve the
                lane's latest scheduled exit.
        """

        vehicle = self.vehicles[vehicle_index]
        p = self.vehicle_progress[vehicle].rear

        # Determine what exit parameters we're targetting.
        if (last_rear_exit is None) and (self.latest_scheduled_exit is None):
            # No vehicles to avoid. Just get to the intersection as fast as
            # possible and return the ScheduledExit.
            raise NotImplementedError("TODO")
        elif last_rear_exit is None:
            last_rear_exit = self.latest_scheduled_exit

        # Calculate the fastest exit that doesn't collide with last_rear_exit.
        raise NotImplementedError("TODO")

    def progress_at_exit(self, vehicle_index: int, this_exit: ScheduledExit
                         ) -> Tuple[VehicleProgress, List[VehicleTransfer]]:
        """Given this_exit, return clone's progress and transfers at exit.

        Given the index of the vehicle being calculated for and its scheduled
        exit, clone the vehicle for use in reservation testing and estimate
        its time and speed of arrival into the intersection as well as how far
        it makes it into the intersection on that first timestep.

        Parameters
            vehicle_index: int
                The index of the vehicle in self.vehicles to calculate for.
            this_exit: ScheduledExit
                The scheduled exit of this vehicle, found using soonest_exit().
                The vehicle provided in this_exit is a clone of the original
                vehicle at self.vehicles[vehicle_index].

        Returns a tuple of
            VehicleProgress
                The vehicle's progress in the road lane at exit.
            List[VehicleTransfer]
                Transfers for each section of the vehicle's clone that crosses
                the road-intersection seam given the timing and conditions of
                this_exit. By definition of an exit, this should have at least
                one VehicleTransfer for the vehicle's front section.
        """
        # TODO: Remember to clone the vehicle using vehicle.clone() before
        #       creating and returning the VehicleTransfers.
        raise NotImplementedError("TODO")

    def register_latest_scheduled_exit(self, new_exit: ScheduledExit) -> None:
        """Register a new latest scheduled exit if it's actually later.

        Note that the output of a soonest_exit call should not go directly into
        this function, as soonest_exit calculates the soonest exit of the FRONT
        of a vehicle and the latest scheduled exit requires the REAR of the
        vehicle.
        """
        if new_exit.section is not VehicleSection.REAR:
            raise ValueError("This is not an exit for a vehicle's rear.")
        if ((self.latest_scheduled_exit is None) or
                (self.latest_scheduled_exit.t <= new_exit.t)):
            self.latest_scheduled_exit = new_exit


class IntersectionLane(Lane):
    """
    Unlike RoadLanes, movement on IntersectionLanes may be stochastic.
    Depending on their throttle and tracking score properties, vehicles can be
    faster or slower than instructed and deviate laterally without being
    instructed to by a lane change manager.
    """

    def __init__(self,
                 start_lane: RoadLane,
                 end_lane: RoadLane,
                 speed_limit: int = SHARED.speed_limit):
        """Create a new IntersectionLane.

        Instead of taking an explicit trajectory, the Intersection Lane infers
        its own using the Coords of the start and end road lanes and their
        headings at their start and end to create a BezierTrajectory that
        curves nicely.
        """

        self.upstream = start_lane
        self.downstream = end_lane

        # TODO: Create a BezierTrajectory using start_coord, end_coord, and the
        #       intersection of a line drawn from start_coord at start_heading
        #       with a line drawn from end_coord at end_heading. Consider
        #       precalculating the challenge rating, or having bezier traj auto
        #       calculate a challenge rating based on curvature.
        trajectory: Trajectory
        super().__init__(trajectory, min(start_lane.width, end_lane.width),
                         speed_limit)
        self.hash = hash(
            self.trajectory.start_coord + self.trajectory.end_coord)

        # Calculate the shortest amount of time (in timesteps) that it takes
        # for a vehicle to fully travel across this lane.
        self.min_traversal_time = (trajectory.length/speed_limit *
                                   SHARED.steps_per_second)

        # Track vehicles' lateral deviation from centerline in meters
        self.lateral_deviation: Dict[Vehicle, float] = {}

    # Support functions for speed updates

    def controls_this_speed(self, vehicle: Vehicle) -> Tuple[bool, float,
                                                             VehicleSection]:
        """Return the vehicle's controlling section and progress."""
        front = self.vehicle_progress[vehicle].front
        if front is not None:
            # Front of the vehicle is in the lane.
            return True, front, VehicleSection.FRONT
        else:
            # Surely the rear of the vehicle must be in the lane.
            rear = self.vehicle_progress[vehicle].rear
            if rear is None:
                raise RuntimeError("Vehicle exited lane already.")
            return True, rear, VehicleSection.REAR

    def effective_speed_limit(self, p: float, vehicle: Vehicle) -> float:
        """Return the effective speed limit at the given progression."""
        return super().effective_speed_limit(p, vehicle)

        # TODO: (stochasticity) Consider making the effective speed limit a
        #       function of a vehicle's ability to follow instructions
        #       precisely (to account for it over-accelerating) and its
        #       deviation from the lane's centerline to allow it to hit the
        #       actual speed limit if it cuts the corner.
        #
        #       See the update_speeds todo for more information.

    def accel_update(self, vehicle: Vehicle, section: VehicleSection, p: float,
                     preceding: Optional[Vehicle]) -> float:
        """Return a vehicle's new acceleration.

        Just calls the parent's accel_update() (for now).
        """

        # TODO: (platoon) Insert vehicle chain override here.
        #       May not be necessary if the emergent behavior of breaking
        #       chains before they start results in vehicles simply following
        #       normal reservation behavior with chained accelerations.

        return super().accel_update(vehicle, section, p, preceding)

        # TODO: (stochasticity) Change the return so the speed and acceleration
        #       update is affected by both:
        #           1. A stochastic term dependent on the vehicle's reliability
        #              in following speed updates. They may exceed the actual
        #              speed limit if their acceleration control is poor.
        #           2. What side of the trajectory the vehicle is on and how
        #              far from the centerline the vehicle is laterally.
        #              Vehicles that cut corners at the speed limit will have a
        #              higher effective speed limit in-lane, and vehicles that
        #              turn too loose will have a slower effective speed limit.
        #       This would require tweaking the effective speed limit for the
        #       specific vehicle in addition to this acceleration update, since
        #       in the velocity update the effective speed limit trims the
        #       acceleration to the speed limit if it exceeds the speed limit.

    def downstream_stopping_distance(self, vehicle: Vehicle,
                                     section: VehicleSection
                                     ) -> Optional[float]:
        """Check the downstream road lane's required stopping distance."""
        return self.downstream.stopping_distance_to_last_vehicle()

    # Support functions for step updates

    def remove_vehicle(self, vehicle: Vehicle) -> None:
        """Remove records for this vehicle from this trajectory."""

        # Do the normal stuff
        super().remove_vehicle(vehicle)

        # Delete the vehicle's record from our lateral deviation tracker.
        del self.lateral_deviation[vehicle]

    def lateral_deviation_for(self, vehicle: Vehicle,
                              new_progress: float) -> float:
        """Calculate and return the lateral deviation of this vehicle."""

        # TODO: (stochastic) Retrieve self.lateral_deviation[vehicle] and
        #       calculate the new deviation in this timestep. Add them
        #       together, log it back to self.lateral_deviation[vehicle],
        #       and return it.

        return 0

    # Support functions for vehicle transfers

    def add_vehicle(self, vehicle: Vehicle) -> None:
        """Create entries for this vehicle in lane support structures.

        On top of what the parent add_vehicle() does, creates a new
        lateral_deviation entry for this vehicle.
        """
        super().add_vehicle(vehicle)
        self.lateral_deviation[vehicle] = 0
