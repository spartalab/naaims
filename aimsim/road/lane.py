from __future__ import annotations
from typing import TYPE_CHECKING, Tuple, Optional, List, Dict, Set

from aimsim.lane import Lane, VehicleProgress, ScheduledExit
from aimsim.util import (Coord, VehicleSection, SpeedUpdate, VehicleTransfer,
                         CollisionError, TooManyProgressionsError)

if TYPE_CHECKING:
    from aimsim.trajectories import Trajectory
    from aimsim.intersection import Intersection
    from aimsim.vehicles import Vehicle


class RoadLane(Lane):
    """RoadLanes connect intersections, providing them access and egress.

    Like their parent Roads, RoadLanes are divided into three sections: the
    entrance, lane changing, and approach regions.
    """

    def __init__(self,
                 trajectory: Trajectory,
                 width: float,
                 speed_limit: int,
                 len_entrance_region: float,
                 len_approach_region: float,
                 offset: Coord = Coord(0, 0),
                 upstream_is_spawner: bool = False,
                 downstream_is_remover: bool = False) -> None:
        """Create a new road lane."""

        super().__init__(trajectory=trajectory.clone_with_offset(offset),
                         width=width, speed_limit=speed_limit)
        self.upstream_is_spawner: bool = upstream_is_spawner
        self.downstream_is_remover: bool = downstream_is_remover
        self.downstream_intersection: Optional[Intersection] = None

        # Note that we start at the front of the lane and work back, so
        # proportions decrease as we go on.
        assert len_entrance_region + len_approach_region < trajectory.length
        self.entrance_end: float = len_entrance_region/self.trajectory.length
        self.lcregion_end: float = 1-len_approach_region/self.trajectory.length
        # self.approach_end: float = 1

        # Prepare to cache the exit time and speed of the exit of the last
        # vehicle with permission to enter the intersection.
        #
        # This will be used for estimating when the next vehicle looking for
        # permission to enter the intersection will reach it.
        self.latest_scheduled_exit: Optional[ScheduledExit] = None

    def connect_downstream_intersection(self, downstream: Intersection
                                        ) -> None:
        """Connect a downstream intersection to this RoadLane."""
        if self.downstream_is_remover:
            raise ValueError("Downstream shouldn't be intersection.")
        else:
            self.downstream_intersection = downstream

    # Support functions for speed updates

    def get_new_speeds(self, to_slow: Set[Vehicle] = set()
                       ) -> Dict[Vehicle, SpeedUpdate]:
        """Return all vehicles on this lane and their speed updates.

        On top of the processes in the parent function, this does an additional
        check to see if a downstream intersection has been registered, if
        necessary.
        """
        if ((not self.downstream_is_remover)
                and (self.downstream_intersection is None)):
            raise RuntimeError("Missing downstream Intersection.")
        return super().get_new_speeds(to_slow=to_slow)

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
            rear = self.vehicle_progress[vehicle].rear
            if (rear is None) and (not self.upstream_is_spawner):
                # The vehicles is still exiting the intersection and therefore
                # not in this road lane's jurisdiction.
                return False, -float("inf"), VehicleSection.FRONT
            else:
                # Front of the vehicle is on this road and its rear isn't in an
                # intersection, so this road is in control. Report the front.
                return True, front, VehicleSection.FRONT

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

        if self.downstream_is_remover:
            # No need to check for situations where you stop at the
            # intersection line because there is no intersection
            return super().accel_update(vehicle=vehicle, section=section, p=p,
                                        preceding=preceding)
        else:
            # Need to make sure we stop at the intersection line if necessary
            if ((preceding is None) and
                    (not vehicle.permission_to_enter_intersection)):
                # stop at the intersection line
                return self.accel_update_following(vehicle, p)

            a_follow = super().accel_update(vehicle=vehicle, section=section,
                                            p=p, preceding=preceding)

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
                stopping_distance_to_last_vehicle(self.trajectory.end_coord)

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
                its_target = vehicle.next_movements(
                    self.trajectory.end_coord)[0]
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
                if vehicle.next_movements(
                        self.trajectory.end_coord)[0] in targets:
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
        progress = self.vehicle_progress[vehicle].rear

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
