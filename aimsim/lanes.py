"""
The `lanes` module handles the progression of vehicles along 1-dimensional
lines we call "lanes", used on roads and within intersections (e.g., as a left
turn trajectory), making them the fundamental building block of aimsim.

Lanes only interact with their Road and Intersection containers, and so must be
written to interact through their parent road or intersection to pass vehicles
between lanes, either as a lane change or onto the next road of intersection.

The geometry of a lane is determined by what type of Trajectory is
provided when initialized. The Trajectory controls the physics of vehicles on
the lane when they move forward, while the Lane is responsible for holding
proportional progress of the vehicles.
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Tuple, Iterable, Optional, List, Dict, Set
from dataclasses import dataclass

import aimsim.shared as SHARED
from .archetypes import Upstream
from .util import (Coord, CollisionError, VehicleSection, SpeedUpdate,
                   TooManyProgressionsError, VehicleTransfer)
from .intersections.reservations import ReservationRequest
from .vehicles import Vehicle
from .trajectories import Trajectory


class LateralDeviation:
    lane: Lane
    d: float


class Lane(ABC):

    @dataclass
    class VehicleProgress:
        """Track the progress of all three sections of a vehicle.

        Note that the index 0, 1, 2 of these sections in this dataclass
        correspond to the values of the section in the enum VehicleSection.
        """
        front: Optional[float] = None
        center: Optional[float] = None
        rear: Optional[float] = None

    @abstractmethod
    def __init__(self,
                 trajectory: Trajectory,
                 speed_limit: float = SHARED.speed_limit) -> None:
        """Create a new lane.

        Lanes inherit the trajectory of the road they're created by (if
        applicable), adding an offset if necessary.

        Keyword arguments:
        trajectory: The trajectory of the road that contains the lane.
        offset: The amount by which to offset the lane by.
        """
        # initialize lane
        self.vehicles: List[Vehicle] = []
        self.vehicle_progress: Dict[Vehicle, Lane.VehicleProgress] = {}
        self.trajectory = trajectory
        self.speed_limit = speed_limit
        self.request_has_changed = False

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
        """Return speed updates for all vehicles on this lane.

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
            vehicle_in_jurisdiction, p = self.controls_this_speed(vehicle)
            if vehicle_in_jurisdiction:  # update its speed
                # A vehicle being in to_slow overrides any acceleration logic
                # defined in accel_update, instead telling the vehicle to start
                # braking no matter what.
                a_new = (vehicle.max_braking if (vehicle in to_slow)
                         else self.accel_update(vehicle, p, preceding))
                new_speed[vehicle] = self.speed_update(vehicle, p, a_new)

            preceding = vehicle

        return new_speed

    @abstractmethod
    def controls_this_speed(self, vehicle: Vehicle) -> Tuple[bool, float]:
        """Should return if lane controls this vehicle and its progress."""
        raise NotImplementedError("Must be implemented in child class.")

    @abstractmethod
    def accel_update(self, vehicle: Vehicle, p: float,
                     preceding: Optional[Vehicle]) -> float:
        """Return a vehicle's acceleration update.

        Note that this does NOT change the vehicle's position.

        `preceding` is the vehicle ahead of the vehicle we're calculating for.
        If there is none, the vehicle is at the head of the queue and must stop
        at the intersection line if it doesn't have a reservation.
        """
        raise NotImplementedError("Must be implemented in child classes.")

    def accel_update_uncontested(self, vehicle: Vehicle, p: float) -> float:
        """Return accel update if there are no vehicles ahead.

        p is the proportional progress associated with this vehicle. Because it
        could be the front or rear of the vehicle depending on the situation,
        it's presented here as an input argument.
        """
        effective_speed_limit = self.effective_speed_limit(p, vehicle)
        if vehicle.v > effective_speed_limit:
            return vehicle.max_braking
        elif vehicle.v == effective_speed_limit:
            return 0
        else:  # vehicle.v < effective_speed_limit
            return vehicle.max_accel

    def accel_update_following(self,
                               vehicle: Vehicle,
                               p: float,
                               pre_p: float = 1,
                               pre_v: float = 0,
                               pre_a: float = 0) -> float:
        """Return speed update to prevent collision with a preceding object.

        p is the proportional progress associated with this vehicle. Because it
        could be the front or rear of the vehicle depending on the situation,
        it's presented here as an input argument.

        The preceding object is either a preceding vehicle with proportional
        progress, velocity, acceleration broken out or, given the defaults, the
        intersection line.
        """
        effective_speed_limit = self.effective_speed_limit(p, vehicle)

        # TODO: when implementing, take into account that time is discrete so
        #       round down when spacing.

        a_maybe = self.accel_update_uncontested(vehicle, p)
        if a_maybe < 0:  # need to brake regardless of closeness
            return a_maybe
        else:
            # TODO: Calculate closeness. Remember to take into account that the
            #       speeds and accelerations are stale by 1 timestep, so
            #       include enough buffer so that this works even if preceding
            #       immediately starts braking.
            raise NotImplementedError("TODO")
            # if (too close based on stopping distances):
            #     return vehicle.max_braking
            # else:
            #     # override a_maybe iff we can accel but that would put the
            #     # vehicles too close together
            #     return 0 if (can't get closer) else a_maybe

    def speed_update(self, vehicle: Vehicle, p: float,
                     accel: float) -> SpeedUpdate:
        """Given an acceleration, update speeds.

        Note acceleration and speed aren't 1-to-1 because of discrete time.

        p is the proportional progress associated with this vehicle. Because it
        could be the front or rear of the vehicle depending on the situation,
        it's presented here as an input argument.
        """
        v_new = vehicle.v + accel*SHARED.TIMESTEP_LENGTH
        if v_new < 0:
            return SpeedUpdate(v=0, a=accel)
        else:
            effective_speed_limit = self.effective_speed_limit(p, vehicle)
            if v_new > effective_speed_limit:
                return SpeedUpdate(v=effective_speed_limit, a=accel)
            else:
                return SpeedUpdate(v=v_new, a=accel)
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

            # TODO: Redo this section to account for VehicleProgress being a
            #       dataclass now instead of this hacky method.
            old_progress: List[Optional[float]] = [
                self.vehicle_progress[vehicle].front,
                self.vehicle_progress[vehicle].center,
                self.vehicle_progress[vehicle].rear
            ]
            new_vehicle_progress: List[Optional[float]] = [None]*3

            # Iterate through the 3 sections of the vehicle.
            for i, progress in enumerate(old_progress):

                # Check if we have a prop progress record for this section.
                if (progress is None):
                    # This section of the vehicle isn't in this lane.
                    if last_progress > 1:
                        # We're on the downstream end of the lane.
                        if ((VehicleSection(i) == VehicleSection.FRONT)
                                or
                                (VehicleSection(i) == VehicleSection.CENTER)):
                            # This vehicle front or center section is in the
                            # downstream object. Nothing to do here.
                            new_vehicle_progress[i] = progress
                            continue
                        else:  # this is the rear of the vehicle
                            # If the rear is None, the vehicle should have
                            # fully exited and be out of this lane.
                            raise RuntimeError("Exited vehicle not removed.")
                    else:
                        # We're either in the middle of or at the upstream
                        # end of the lane.

                        # If this isn't an error, we've now gone past the end
                        # of the lane.
                        if VehicleSection(i) == VehicleSection.FRONT:
                            raise RuntimeError("Vehicle has not entered this "
                                               "lane yet, why is it here?")
                        elif VehicleSection(i) == VehicleSection.CENTER:
                            if last_progress >= 0:
                                # The front of this vehicle is in the lane but
                                # this center and the rear are still in the
                                # upstream intersection. This is ok.
                                new_vehicle_progress[i] = progress
                                last_progress = -1
                                continue
                            else:
                                raise RuntimeError("Front AND center of this "
                                                   "vehicle aren't in this "
                                                   "lane even though they're "
                                                   "not at the front of the "
                                                   "lane? How?")
                        else:  # this is the rear of the vehicle
                            if last_progress >= 0:
                                # The front and center of this vehicle are in
                                # this lane but this rear is still in the
                                # upstream intersection. This is ok.
                                last_progress = -1
                            # Otherwise, the center of this vehicle is also in
                            # the upstream intersection in addition to this
                            # rear. This is ok.
                            # If this is not the case (e.g., if there's a
                            # vehicle after this one that's on this lane),
                            # the issue will be caught on the next loop.
                            new_vehicle_progress[i] = progress
                            continue
                elif progress <= last_progress:
                    raise CollisionError("Vehicle(s) are overlapping in-lane.")

                # Update relative position.
                # TODO: translate vehicle.v*SHARED.timestep into proportional
                #       progress using trajectory
                new_progress: Optional[float]  # the progress after it's done
                t_left: float = 0  # TODO: calculate this if the vehicle exits
                raise NotImplementedError("TODO")
                new_vehicle_progress[i] = new_progress

                if ((VehicleSection(i) == VehicleSection.CENTER)
                        and (progress is not None)):
                    # We're at a vehicle's center section. Update the vehicle's
                    # own record of its position in absolute Coords.

                    lateral: float = 0
                    if ((vehicle in lateral_deviations)
                            and (self is lateral_deviations.lane)):
                        # We have a precalculated lateral movement for this
                        # vehicle and it's defined as relative to this lane.
                        # This lateral movement is relative to this lane,
                        # so we're ok to include it in our position update.
                        self.update_vehicle_position(
                            self,
                            vehicle,
                            new_progress,
                            lateral_deviations.d
                        )
                    elif (vehicle not in lateral_deviations):
                        # We need to infer this vehicle's lateral movement.
                        self.update_vehicle_position(
                            self,
                            vehicle,
                            new_progress,
                            self.lateral_deviation_for(vehicle, new_progress)
                        )
                    # Otherwise, this vehicle is present in multiple lanes due
                    # to a lane change, and its lateral movement is defined as
                    # relative to another lane. Let the other lane update its
                    # true position instead.

                if (new_progress is None):
                    # Vehicle section has exited. Create a VehicleTransfer
                    # object from it and add it to the return list.
                    exiting.append(VehicleTransfer(
                        vehicle=vehicle,
                        section=VehicleSection(i),
                        t_left=t_left,
                        pos=self.end_coord
                    ))

                last_progress = progress

            # Check if this vehicle has fully exited.
            if all((p is None) for p in new_vehicle_progress):
                # If so, mark it for removal from this lane.
                to_remove.append(vehicle)
            else:
                # Update vehicle_progress with the new progress values
                self.vehicle_progress[vehicle] = Lane.VehicleProgress(
                    front=new_vehicle_progress[0],
                    center=new_vehicle_progress[1],
                    rear=new_vehicle_progress[2]
                )

        for vehicle in to_remove:
            # Remove the marked vehicles from this lane.
            self.remove_vehicle(vehicle)

        return exiting

    def lateral_deviation_for(self, vehicle: Vehicle,
                              new_progress: float) -> float:
        """Return the lateral movement for a vehicle. 0 unless overridden."""
        return 0

    def update_vehicle_position(self, vehicle: Vehicle, new_p: float,
                                lateral_deviation: float = 0) -> None:
        """Should update the vehicle's position, including lateral movement."""

        # TODO: only intersectionlane has record of lateral deviation
        #       should lateral_movement be cumulative instead?
        #       could give both a record but would that be weird with multilane
        #       drifting?

        # longitudinal update
        pos: Coord = self.trajectory.get_position(new_p)

        if lateral_deviation != 0:
            # TODO: (multi/stochasticity) use trajectory.heading(new_p) to
            #       calculate the lateral deviation away from pos.
            raise NotImplementedError("TODO")

        # write it to new vehicle
        vehicle.pos = pos

    def remove_vehicle(self, vehicle: Vehicle) -> None:
        """Remove an exited vehicle from this lane."""
        self.vehicles.remove(vehicle)
        del self.vehicle_progress[vehicle]

    # Support functions for vehicle transfers

    def enter_vehicle_section(self, transfer: VehicleTransfer) -> None:
        """Adds a section of a transferring vehicle to the lane."""
        vehicle: Vehicle = transfer.vehicle

        # Find how far along the lane this vehicle section will be in meters.
        d: float
        if transfer.t_left is None:
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
            # This is a transfer between road and intersection.
            # Calculate the distance the vehicle section has left to travel
            # in this step.
            d = transfer.vehicle.v * transfer.t_left

        # Convert the real units distance d into proportional progress along
        # the lane trajectory.
        p: float = d/self.trajectory.length
        if transfer.section is VehicleSection.FRONT:
            # This is the first time this lane has seen this vehicle.
            # Create new entries for this vehicle in the lane's relevant data
            # structures and populate its front section progress.
            self.add_vehicle(vehicle)
            self.vehicle_progress[vehicle].front = p
        elif transfer.section is VehicleSection.CENTER:
            # Update the lane's record of the vehicle center's position and
            # the vehicle's own record of its actual position Coord.
            vehicle.pos = self.trajectory.get_position(p)
            self.vehicle_progress[vehicle].center = p
        else:  # transfer.section is VehicleSection.REAR
            # Update only the lane's record of the vehicle rear's position.
            self.vehicle_progress[vehicle].rear = p

    def add_vehicle(self, vehicle: Vehicle) -> None:
        """Create entries for this vehicle in lane support structures."""
        self.vehicles.append(vehicle)
        self.vehicle_progress[vehicle] = Lane.VehicleProgress()

    # Misc functions

    @abstractmethod
    def __hash__(self):
        raise NotImplementedError("Must be implemented in child classes.")


class RoadLane(Lane):
    """RoadLanes connect intersections, providing them access and egress.

    Like their parent Roads, RoadLanes are divided into three sections: the
    entrance, lane changing, and approach regions.

    They connect directly to the conflict area controlled by `manager`s.
    """

    def __init__(self,
                 trajectory: Trajectory,
                 offset: Coord = Coord(0, 0),
                 len_entrance_region: float = SHARED.min_entrance_length,
                 len_approach_region: float = 100,
                 upstream_is_intersection: bool = False,
                 downstream_is_remover: bool = False,
                 speed_limit: int = SHARED.speed_limit) -> None:

        self.start_coord: Coord = Coord(trajectory.start_coord.x + offset.x,
                                        trajectory.start_coord.y + offset.y)
        self.end_coord: Coord = Coord(trajectory.end_coord.x + offset.x,
                                      trajectory.end_coord.y + offset.y)

        # TODO: Copy the provided trajectory, changing the start and end coords
        #       to match the provied offsets.
        raise NotImplementedError("TODO")

        super().__init__(trajectory=trajectory,
                         speed_limit=speed_limit)
        self.upstream_is_intersection: bool = upstream_is_intersection
        self.downstream_is_remover: bool = downstream_is_remover

        # TODO: use len_entrance_region and len_approach_region to calculate
        #       the proportional cutoffs for the three regions, for use in the
        #       the speed update and step functions
        # Note that we start at the front of the lane and work back, so
        # proportions decrease as we go on.
        # self.approach_end: float = 1
        self.lcregion_end: float = 0.6
        self.entrance_end: float = 0.3

        raise NotImplementedError("TODO")

    # Support functions for speed updates

    def controls_this_speed(self, vehicle: Vehicle) -> Tuple[bool, float]:
        """Return if this lane controls this vehicle, and its progress."""

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
                    return True, rear
            else:
                # The front of the vehicle is in the next intersection, so this
                # road lane is not in control.
                return False, float("inf")
        else:
            # The front of this vehicle front is on this road.
            if self.upstream_is_intersection:
                # The vehicles is still exiting the intersection and therefore
                # not in this road lane's jurisdiction.
                return False, -float("inf")
            else:
                # Front of the vehicle is on this road and its rear isn't in an
                # intersection, so this road is in control. Report the front.
                return True, front

    def accel_update(self, vehicle: Vehicle, p: float,
                     preceding: Optional[Vehicle]) -> float:

        # TODO: (platoon) Insert vehicle chain override here and return it.
        #       Vehicle should look forward to downstream chained vehicles to
        #       match the velocity and accel of all chained vehicles as soon as
        #       possible. Once matched, I think it ends up being just normal
        #       lane following behavior, unless the chain is exiting the
        #       intersection. If a vehicle in the chain is on the road but an
        #       upstream vehicle is still in the intersection, the downstream
        #       vehicle should mirror the speed and acceleration of the vehicle
        #       still in the intersection.

        if preceding is None:
            # don't need to check against the preceding vehicle
            if vehicle.can_enter_intersection:
                # full speed forward
                # TODO: does this run into collision issues for signalized?
                return self.accel_update_uncontested(vehicle, p)
            else:
                # stop at the intersection line
                return self.accel_update_following(vehicle, p)
        else:
            preceding_rear = self.vehicle_progress[preceding].rear
            if preceding_rear is None:
                raise ValueError("Preceding vehicle not in lane.")
            a_follow = self.accel_update_following(vehicle, p,
                                                   pre_p=preceding_rear,
                                                   pre_v=preceding.v,
                                                   pre_a=preceding.a)
            if (preceding.can_enter_intersection
                    and vehicle.can_enter_intersection):
                # follow preceding vehicle into the intersection
                return a_follow
            else:
                # stop for preceding vehicle AND intersection line
                return min(a_follow, self.accel_update_following(vehicle, p))

    # Used by intersection manager via road methods

    def check_entrance_collision(self, steps_forward: int,
                                 entering_vehicle: Vehicle,
                                 entering_v0: float,
                                 entering_accel_profile: Iterable[float]
                                 ) -> bool:
        """Returns true if proposed movement may cause a collision.

        We need to ensure that a vehicle that wants to enter one of this road's
        lanes won't collide with any vehicles currently in the lane. To do so
        we look ahead to when the new vehicle is proposing to enter the lane
        with a simple heuristic.

        Given what velocity and acceleration profile the new vehicle is
        proposing to enter the lane at, we isolate the position and speed of
        the last vehicle in the lane at the current timestep and assume it
        starts and continues braking between the current timestep and the
        target timestep(s). Given this behavior for the vehicle currently in
        the lane, we check if the entering vehicle will collide with the
        in-lane vehicle if it follows its proposed speed and acceleration
        profile. If so, return `True`. If the proposed vehicle won't collide,
        return `False`.

        This function should be called by `VehicleSpawner` to check if a
        vehicle will collide before spawning it, and by `IntersectionManager`
        to check if a reservation will collide as it exits the intersection.

        Parameters:
            steps_forward : int
                how many steps forward does the vehicle start entering
            entering_vehicle : Vehicle
                the vehicle entering
            entering_v0 : float
                at steps_forward, what is its speed
            entering_accel_profile : Iterable[float]
                what this vehicle's acceleration from the time it starts
                entering to the time it finishes entering the lane
        """

        # TODO: Consider caching the worst-case profile of the last vehicle
        #       at each step instead of calculating it fresh each time. Maybe
        #       just calculate the lane position at which the new vehicle would
        #       be considered "colliding" at each timestep as needed, caching
        #       them for future calls. Clear this cache on a new step() call.

        # TODO: I think I'm forgetting an input arg but I forget what.

        # TODO: Maybe add a check that puts the distance to collision at the
        #       min of (projected position of the last vehicle, edge of
        #       entrance region)

        raise NotImplementedError("TODO")

    def next_reservation_candidate(self,
                                   group: bool = False,
                                   all_groups: bool = False,
                                   delay: int = 0,
                                   value: bool = False
                                   ) -> Optional[Iterable[ReservationRequest]]:
        """Return the first vehicle/platoon without a reservation if it exists.

        Parameters:
            group: bool
                Whether to collect vehicles of the same class (human/auto/semi)
                and movement (Intersection end Coord) together into a platoon.
            all_groups: bool
                If grouping, whether to return every combination of groups
                possible. If not, just return the biggest group.
            delay: int
                Return a reservation given that it can't start until this many
                timesteps from now.
        """

        if all_groups and not group:
            raise ValueError("all_groups only valid if group is true.")
        if (delay < 0) or (type(delay) is not int):
            raise ValueError("delay must be non-negative.")

        # TODO: check if the vehicle in front of the current first vehicle
        #       without a reservation is a Platoon. if so, the preceding
        #       vehicle's acceleration capability may change once its front
        #       enters the intersection, altering the calcultion for the
        #       current vehicle's min_time_to_intersection

        # TODO: check a flag to see if it should bother returning a request or
        #       if it'll be the same as the last time this lane was polled

        # look through the approach region for the first Vehicle that wants an
        # intersection reservation and doesn't have one yet. (A human-driven
        # vehicle in an FCFS-Light setup wouldn't want a reservation.) If
        # group, return a sequence of vehicles that fit the criteria.

        # TODO: implement checking for the value flag to sum vot across the
        #       lane andreturn nonzero value the ReservationRequest(s)
        if not group:
            raise NotImplementedError("TODO")
        else:
            # calculate min time based on slowest vehicle in lane
            # make sure to look at every vehicle in the entrance region
            # following the lead vehicle that has the same movement, even if
            # they're not lined up neatly behind the first vehicle.

            if all_groups:
                # returning a bundle of candidate platoons that range from
                # including just the first vehicle to every vehicle so the
                # policy can better sequence auctions
                # TODO: maybe just reprocess the platoon packet in the policy
                #       instead of doing this complicated thing?
                raise NotImplementedError("TODO")

    def free_space(self, tight: bool = True) -> float:
        """Return the amount of free space left in the intersection.

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
        raise NotImplementedError("TODO")

    def min_time_to_intersection(self, vehicle: Vehicle) -> int:
        """Return a vehicle's minimum time to intersection in timesteps."""
        # TODO: abstract away behavior in update_speeds to figure this out
        raise NotImplementedError

    # Misc functions

    def __hash__(self):
        return hash(self.start_coord + self.end_coord)


class IntersectionLane(Lane):
    """
    Like RoadLanes, but without the three-region specification and vehicles
    have more consistent in-lane behavior, at least when identified as
    deterministic. Unlike RoadLanes, IntersectionLanes can potentially be
    uncertain; we aren't sure if the vehicle using it is going to follow
    instructions exactly 100% of the time. Here, instead of being the ground
    truth, the trajectory is just a prior. In the update_*_speeds and step
    functions, vehicles' properties (pos/v/heading) are updated with some
    amount of randomness off of the centerline trajectory.
    """

    def __init__(self,
                 start_coord: Coord,
                 start_heading: float,
                 end_coord: Coord,
                 end_heading: float,
                 width: float,
                 speed_limit: int = SHARED.speed_limit):
        # TODO: Create a BezierTrajectory using start_coord, end_coord, and the
        #       intersection of a line drawn from start_coord at start_heading
        #       with a line drawn from end_coord at end_heading. Consider
        #       precalculating the challenge rating, or having bezier traj auto
        #       calculate a challenge rating based on curvature.
        trajectory: Trajectory
        super().__init__(trajectory, speed_limit)

        # Used to sweep out this lane's area during signal reservations.
        self.width = width

        # Calculate the shortest amount of time (in timesteps) that it takes
        # for a vehicle to fully travel across this lane.
        self.min_traversal_time = (trajectory.length/speed_limit *
                                   SHARED.steps_per_second)

        # Track vehicles' lateral deviation from centerline in meters
        self.lateral_deviation: Dict[Vehicle, float] = {}

        self.temp_speed_limit: float
        self.last_exit: int
        # Reset temp speed limit to avoid crashes out of signal intersections
        self.reset_temp_speed_limit()

    # Support functions for speed updates

    def controls_this_speed(self, vehicle: Vehicle) -> Tuple[bool, float]:
        front = self.vehicle_progress[vehicle].front
        if front is not None:
            # Front of the vehicle is in the lane.
            return True, front
        else:
            # Surely the rear of the vehicle must be in the lane.
            rear = self.vehicle_progress[vehicle].rear
            if rear is None:
                raise RuntimeError("Vehicle exited lane already.")
            return True, rear

    def set_temp_speed_limit(self, vehicle: Vehicle) -> None:
        """Set a temporary speed limit to prevent crashes just downstream.

        In signalized intersections, we don't have full control over vehicles.
        Thus we need to ensure that there's enough room between vehicles just
        outside of the intersection to avoid collisions, even if we can no
        longer see the vehicle just downstream. Solve this by caching the speed
        of recently exited vehicles and requiring that no vehicles on this
        trajectory the same cycle exceeds that.
        """
        self.temp_speed_limit = min(self.temp_speed_limit, vehicle.v)
        self.last_exit = SHARED.t

    def reset_temp_speed_limit(self) -> None:
        """Reset the temporary speed limit."""
        self.temp_speed_limit = self.speed_limit
        self.last_exit = SHARED.t

    def effective_speed_limit(self, p: float, vehicle: Vehicle) -> float:
        """Return the effective speed limit at the given progression.

        Some trajectories may require a lower speed limit, e.g. on sharper
        turns.
        """
        return min(self.temp_speed_limit,
                   super().effective_speed_limit(p, vehicle))

        # TODO: (stochasticity) Consider making the effective speed limit a
        #       function of a vehicle's ability to follow instructions
        #       precisely (to account for it over-accelerating) and its
        #       deviation from the lane's centerline to allow it to hit the
        #       actual speed limit if it cuts the corner.
        #
        #       See the update_speeds todo for more information.

    def accel_update(self, vehicle: Vehicle, p: float,
                     preceding: Optional[Vehicle]) -> float:

        # TODO: (platoon) Insert vehicle chain override here.
        #       May not be necessary if the emergent behavior of breaking
        #       chains before they start results in vehicles simply following
        #       normal reservation behavior with chained accelerations.

        if vehicle.has_reservation or (preceding is None):
            # vehicle has a confirmed reservation that assumed maximum
            # acceleration, so we're free to behave as if it's uncontested
            # OR there is no vehicle ahead and the lane has traffic signal
            # permission, so the temporarily lower speed limit is in effect
            return self.accel_update_uncontested(vehicle, p)
        else:
            # lane has traffic signal permission and there's a vehicle ahead
            # to follow
            preceding_rear = self.vehicle_progress[preceding].rear
            if preceding_rear is None:
                raise RuntimeError("Preceding vehicle not in lane.")
            return self.accel_update_following(vehicle, p,
                                               pre_p=preceding_rear,
                                               pre_v=preceding.v,
                                               pre_a=preceding.a)

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
        #
        #       See the effective speed limit todo for more details.

    # Support functions for step updates

    def step_vehicles(self, lateral_deviations: Dict[Vehicle, LateralDeviation]
                      ) -> Iterable[VehicleTransfer]:
        # Reset speed limit when 30s pass since the last exit.
        if SHARED.t - self.last_exit > self.min_traversal_time:
            self.reset_temp_speed_limit()
        # TODO: mark on vehicle exit
        self.last_exit = SHARED.t

        # Then do the normal stuff
        return super().step_vehicles(lateral_deviations)

    def remove_vehicle(self, vehicle: Vehicle) -> None:
        """Remove records for this vehicle from this trajectory."""

        # Do the normal stuff
        super().remove_vehicle(vehicle)

        # Delete the vehicle's record from our lateral deviation tracker.
        del self.lateral_deviation[vehicle]

        # Set the temporary speed limit
        self.set_temp_speed_limit(vehicle)

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
        super().add_vehicle(vehicle)
        self.lateral_deviation[vehicle] = 0

    # Misc functions

    def __hash__(self):
        return hash(self.trajectory.start_coord + self.trajectory.end_coord)
