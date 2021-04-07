from __future__ import annotations
from aimsim.intersection.lane import IntersectionLane
from copy import copy
from math import ceil, sqrt
from typing import TYPE_CHECKING, Tuple, Optional, List, Dict, Set, TypeVar

import aimsim.shared as SHARED
from aimsim.lane import Lane, VehicleProgress, ScheduledExit
from aimsim.util import (Coord, VehicleSection, SpeedUpdate, VehicleTransfer,
                         CollisionError)  # TODO: (low) Check for collisions.

if TYPE_CHECKING:
    from aimsim.trajectories import Trajectory
    from aimsim.intersection import Intersection
    from aimsim.vehicles import Vehicle

L = TypeVar('L', bound='RoadLane')


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
        assert len_entrance_region + len_approach_region <= trajectory.length
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
                The second int is one plus the index of the last vehicle in the
                sequence of vehicles beginning with the vehicle at the first
                index of vehicles with the same desired movement through the
                intersection.
                This is intended for use as range(int, int).
        """

        # TODO: (performance) This ought to be cacheable.

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
            # TODO: Check that this vehicle is fully in lane. If not, ignore it
            #       since we can't calculate a SoonestExit for it.
            if (p < self.lcregion_end) or \
                    (self.vehicle_progress[vehicle].rear is None):
                # This vehicle is outside of the approach area or not fully in
                # the lane.
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
                    # We want to look for further vehicles with the same move.
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

        return None if first_index is None else (first_index,
                                                 first_index + series_length)

    def soonest_exit(self, vehicle_index: int,
                     last_rear_exit: Optional[ScheduledExit] = None
                     ) -> Optional[ScheduledExit]:
        """Return the soonest exit this vehicle can make, given conditions.

        aimsim assumes this behavior of all vehicles:
            1. A vehicle is always accelerating to or at the speed limit,
               unless it needs to brake to prevent a potential collision with a
               preceding vehicle.
            2. If the vehicle is even partially in an intersection, it's
               guaranteed to never need to brake to avoid collision.
        Given these conditions and the exit timing and velocity of the vehicle
        preceding it, we can predict when and how fast a specific vehicle will
        exit a RoadLane into an intersection, with the only control variable
        being when the intersection issues it permission to enter, which
        affects if and when it starts braking before reaching the intersection.
        This function informs the intersection when a vehicle can exit to
        facilitate that decision-making.

        Let x(t, t_b) be the trajectory of the study vehicle at time t given
        how long it spends deviating from its accelerate-to-speed-limit-
        before-intersection behavior t_b, and x_p(t) be the trajectory of the
        preceding vehicle from its time of entrance into the intersection.

        Given that the speed limit v_max, acceleration rate a, and braking rate
        b are the same for all vehicles, we want to find the smallest value of
        t_b such that x(t, t_b) doesn't overtake (collide with) x_p(t).
        Usefully, a collision is only possible before the preceding vehicle
        reaches the speed limit and the trailing vehicle can only overtake the
        preceding vehicle at most once (i.e., they can't keep trading leads),
        so we need only focus on the time-position of both vehicles at the
        moment the preceding vehicle reaches v_max, t_crit. If the trailing
        vehicle meets the preceding one vehicle exactly at that time, or can
        never meet the preceding vehicle because the preceding vehicle is
        moving faster than the trailing one from the start, we know that we've
        found the soonest exit. What side x_p(t_crit) we land on is also
        informative for the purposes of finding the exact minimum t_b.

        This function does three things:
        1. Find the fastest exit x(t, 0) and check if it's the soonest exit,
           returning if x(t_crit, 0) <= x_p(t_crit).
        2. Find the slowest reasonable exit x(t, t_max) (i.e., that doesn't
           require braking to 0 velocity and waiting) and check if it's still
           too fast and causes a collision by checking if x(t_crit) is greater
           than x_p(t_crit). If so, return None.
        3. Binary search x(t_crit, t_b) over t_b with direction indicated by
           x_p(t_crit) - x(t_crit, b). Return the fastest exit found with
           error up to the length of a simulated timestep.

        Terminology note: a VALID exit is one that does not collide with the
        preceding vehicle in the intersection, while an INVALID exit does. But,
        just because an exit is VALID does not make it the SOONEST exit.

        Parameters
            vehicle_index: int
                The index of the vehicle in self.vehicles to calculate for.
            last_rear_exit: Optional[ScheduledExit]
                The scheduled exit that we're calculating for this vehicle to
                avoid colliding with while exiting. If None, retrieve the
                lane's latest scheduled exit if it exists.
        """

        # TODO: (performance) Implement timeout dependent on the estimated
        #       arrival time, e.g., wait (t_current-t_arrival)/2. Maybe don't
        #       do it here since some functions assume that it returns.

        # Fetch the relevant information
        vehicle: Vehicle = self.vehicles[vehicle_index]
        progress: Optional[float] = self.vehicle_progress[vehicle].rear
        if progress is None:
            # Vehicle has yet to fully enter the intersection.
            return None
        if last_rear_exit is None:
            last_rear_exit = self.latest_scheduled_exit

        # Atomize the relevant parameters for readability
        t0: float = SHARED.t
        v0: float = vehicle.velocity
        a: float = vehicle.max_acceleration
        b: float = -vehicle.max_braking
        v_max: float = self.effective_speed_limit(progress, vehicle)
        # TODO: (consistency) Variable speed limits not supported.
        x_to_intersection: float = self._x_to_intersection(progress)

        # 1. Evaluate the free flow case with no preceding exit to find the
        #    ScheduledExit that gets to the intersection as fast as possible.
        t_to_v_max: float = self._t_to_v(v0, a, v_max)
        x_to_v_max: float = self._x_over_constant_a(v0, a, t_to_v_max)
        t_fastest_exit, v_fastest_exit = self._free_flow_exit(
            v0, a, v_max, t_to_v_max, x_to_v_max, x_to_intersection)
        exit: ScheduledExit = ScheduledExit(vehicle, VehicleSection.REAR,
                                            t0 + t_fastest_exit,
                                            v_fastest_exit)

        if (last_rear_exit is None) or ((last_rear_exit.t <= exit.t) and
                                        (last_rear_exit.v >= exit.v)):
            # Case 1: There's nothing to collide with.
            # Case 2: They exit after each other and the preceding vehicle
            #         exited as fast or faster than this one.
            # In either case, a collision is impossible so we can return.
            return exit

        # Find the time when the preceding vehicle reaches the speed limit and
        # how much distance it's covered by then.
        a_p: float = last_rear_exit.vehicle.max_acceleration
        assert a_p == a
        v0_p: float = last_rear_exit.v
        t_p_to_v_max: float = self._t_to_v(v0_p, a_p, v_max)
        x_crit: float = self._x_over_constant_a(v0_p, a_p, t_p_to_v_max)
        # The collision window is between last_rear_exit.t + [0, t_p_to_v_max].
        # If the study vehicle overtakes the preceding vehicle in this time
        # window, it's a crash, so we'll call the furthest the preceding
        # vehicle gets before it reaches the speed limit x_crit.

        # If the proposed exit comes after the last exit chronologically and
        # there's enough separation between the preceding vehicle and the free
        # flow exit, return the latter.
        if (last_rear_exit.t <= exit.t) and self._enough_separation(
                exit, a, v_max, last_rear_exit.t, t_p_to_v_max, x_crit):
            return exit

        # If we reach here, we know that the fastest/free flow exit is invalid.
        # 2. Check if the slowest reasonable exit, braking to a complete stop
        #    just as the study vehicle reaches the intersection, is valid.
        #    (The t_brake values are used to support binary search later.)
        (t_slowest_exit, t_brake_max, t_brake_largest_seen_at_v_max,
         t_brake_smallest_seen_no_v_max) = self._slowest_exit(
             v0, a, b, v_max, x_to_v_max, x_to_intersection, t_to_v_max)
        #    Since the last exit must be moving at at least 0 speed (the speed
        #    of this exit), if it isn't after, no reasonable exit (i.e., an
        #    exit that doesn't involve staying stopped for some time) is valid
        #    so we return None.
        t_slowest_exit_zeroed: int = ceil(t0 + t_slowest_exit)
        if t_slowest_exit_zeroed > last_rear_exit.t:
            return None
        elif t_slowest_exit_zeroed == last_rear_exit.t:
            # The slowest exit is exactly the soonest one.
            return ScheduledExit(vehicle, VehicleSection.REAR,
                                 t_slowest_exit_zeroed, 0)

        # If we reach here, we know that somewhere between 0 brake time and the
        # brake time found in the slowest exit is the fastest possible exit.
        # 3. Do a binary search over the braking space to find the amount of
        #    braking time that causes the vehicle trajectories to meet just as
        #    the preceding vehicle reaches the speed limit, indicating the
        #    fastest possible exit.

        # The critical time is when the preceding vehicle reaches the speed
        # limit, so define it in terms of time from the present timestep.
        t_crit: float = self._t_danger(last_rear_exit.t, t_p_to_v_max, t0)

        # Set up the search to continue until the gap is smaller than the
        # length of a timestep, defaulting to the slowest exit.
        t_left: float = 0
        t_right: float = t_brake_max
        t_exit_guess: float = t_slowest_exit
        v_guess: float = 0

        def search_step(t_left: float, t_right: float,
                        t_brake_largest_seen_at_v_max: Optional[float],
                        t_brake_smallest_seen_no_v_max: Optional[float]
                        ) -> Tuple[float, float, float, float]:
            return self._t_brake_search_step(t_left, t_right,
                                             t_brake_largest_seen_at_v_max,
                                             t_brake_smallest_seen_no_v_max,
                                             v0, a, b, v_max, x_to_v_max,
                                             t_to_v_max, x_to_intersection,
                                             x_crit, t_crit)

        while t_right - t_left > SHARED.SETTINGS.TIMESTEP_LENGTH:
            t_left, t_right, t_exit_guess, v_guess = \
                search_step(t_left, t_right, t_brake_largest_seen_at_v_max,
                            t_brake_smallest_seen_no_v_max)

        return ScheduledExit(vehicle, VehicleSection.FRONT,
                             ceil(t_exit_guess), v_guess)

    def _x_to_intersection(self, progress: float) -> float:
        """Return the distance to the intersection given some progress."""
        return (1-progress)*self.trajectory.length

    @staticmethod
    def _t_to_v(v0: float, a: float, vf: float) -> float:
        """Given v0, acceleration, and vf, find the time to reach vf."""
        return (vf - v0)/a

    @staticmethod
    def _x_over_constant_a(v0: float, a: float, t: float) -> float:
        """Given speed, acceleration, and time, find the distance covered."""
        return v0*t + (a/2)*t**2

    @staticmethod
    def _free_flow_exit(v0: float, a: float, v_max: float, t_to_v_max: float,
                        x_to_v_max: float, x_to_intersection: float
                        ) -> Tuple[int, float]:
        """Return the time and velocity of the free flow exit."""
        if x_to_v_max > x_to_intersection:
            # The vehicle will exit before reaching the speed limit.
            t_fastest_exit = ceil((-v0 + sqrt(v0**2 + 2*a*x_to_intersection)) /
                                  a)
            v_fastest_exit = v0 + a*t_fastest_exit
        else:
            # The vehicle will exit after reaching the speed limit.
            t_fastest_exit = ceil(t_to_v_max +
                                  (x_to_intersection - x_to_v_max)/v_max)
            v_fastest_exit = v_max
        return t_fastest_exit, v_fastest_exit

    @staticmethod
    def _t_danger(t_last_exit: float, t_last_exit_to_v_max: float,
                  t_study: float) -> float:
        """Find the amount of possible collision time."""
        return t_last_exit + t_last_exit_to_v_max - t_study

    @staticmethod
    def _enough_separation(exit: ScheduledExit, a: float, v_max: float,
                           t_last_exit: float, t_last_exit_to_v_max: float,
                           x_crit: float) -> bool:
        """Check separation between the last exit and this free flow exit.

        Find the amount of possible collision time the study vehicle spends in
        the intersection. If it doesn't spend any, this free flow exit is good;
        if not, we need to check the distances both exits cover in the
        intersection to determine if they'll collide or not.
        """
        t_danger: float = RoadLane._t_danger(t_last_exit, t_last_exit_to_v_max,
                                             exit.t)
        if t_danger <= 0:
            return True
        else:
            # Find the distance covered by the trailing vehicle in the time
            # it takes for the preceding vehicle to reach the speed limit.
            t_i_to_speed_limit: float = RoadLane._t_to_v(exit.v, a, v_max)
            x_i: float
            if t_i_to_speed_limit > t_danger:
                # Won't reach the speed limit before the collision window ends.
                x_i = RoadLane._x_over_constant_a(exit.v, a, t_danger)
            else:
                x_i = RoadLane._x_over_constant_a(exit.v, a,
                                                  t_i_to_speed_limit) \
                    + RoadLane._x_over_constant_a(v_max, 0, t_danger -
                                                  t_i_to_speed_limit)
            if x_i <= x_crit:
                # This vehicle can never overtake the last exit.
                return True
        return False

    @staticmethod
    def _slowest_exit(v0: float, a: float, b: float, v_max: float,
                      x_to_v_max: float, x_to_intersection: float,
                      t_to_v_max: float) -> Tuple[
                          float, float, Optional[float], Optional[float]]:
        """Find the slowest possible exit that doesn't stay at 0 velocity.

        Returns a few support variables that will facilitate binary search if
        this exit ends up being valid.
        """
        t_brake_from_v_max: float = RoadLane._t_to_v(v_max, -b, 0)
        x_brake_from_v_max: float = RoadLane._x_over_constant_a(
            v_max, -b, t_brake_from_v_max)

        t_slowest_exit: float
        t_brake_max: float
        # (Tracking variables for use in binary search)
        t_brake_largest_seen_at_v_max: Optional[float] = None
        t_brake_smallest_seen_no_v_max: Optional[float] = None
        if x_to_v_max + x_brake_from_v_max > x_to_intersection:
            # Need to brake before reaching v_max.
            t_slowest_exit = (-b*v0 + (b**2*v0**2 + a*b*v0**2 - 2*a *
                                       b*x_to_intersection*(a + b))**(.5)
                              ) / (a*b)
            t_brake_max = t_slowest_exit - (b*t_slowest_exit - v0) / (a+b)
            # See dissertation appendix for derivation.
            t_brake_smallest_seen_no_v_max = t_brake_max
        else:
            # Spend some time at the speed limit before braking.
            x_at_v_max: float = RoadLane._x_at_v_max(
                x_to_intersection, x_to_v_max, x_brake_from_v_max)
            t_slowest_exit = RoadLane._t_of_v_max_exit(
                t_to_v_max, t_brake_from_v_max, x_at_v_max, v_max)
            t_brake_max = t_brake_from_v_max
            t_brake_largest_seen_at_v_max = t_brake_max
        return (t_slowest_exit, t_brake_max, t_brake_largest_seen_at_v_max,
                t_brake_smallest_seen_no_v_max)

    @staticmethod
    def _x_at_v_max(x_to_intersection: float, x_to_v_max: float,
                    x_brake_from_v_max: float) -> float:
        """Find the distance spent at v_max."""
        return x_to_intersection - x_to_v_max - x_brake_from_v_max

    @staticmethod
    def _t_of_v_max_exit(t_to_v_max: float, t_brake_from_v_max: float,
                         x_at_v_max: float, v_max: float) -> float:
        """Find the time an exit that stays at v_max for a bit happens."""
        return t_to_v_max + x_at_v_max / v_max + t_brake_from_v_max

    @staticmethod
    def _t_brake_search_step(t_left: float, t_right: float,
                             t_brake_largest_seen_at_v_max: Optional[float],
                             t_brake_smallest_seen_no_v_max: Optional[float],
                             v0: float, a: float, b: float, v_max: float,
                             x_to_v_max: float, t_to_v_max: float,
                             x_to_intersection: float, x_crit: float,
                             t_crit: float
                             ) -> Tuple[float, float, float, float]:
        """Execute one step of the binary search."""

        t_brake_guess = t_left + (t_right - t_left)/2
        reaches_v_max_before_intersection: bool = False

        # Check if our guessed brake time is in a region known to reach the
        # speed limit or not before reaching the intersection. If it's in
        # an unknown region, manually check if it reaches the speed limit
        # and update our known region.
        (reaches_v_max_before_intersection, t_brake_largest_seen_at_v_max,
         t_brake_smallest_seen_no_v_max) = \
            RoadLane._reaches_v_max_before_intersection(
            t_brake_guess, t_brake_largest_seen_at_v_max,
            t_brake_smallest_seen_no_v_max, b, v_max, x_to_v_max,
            x_to_intersection)

        # Find the exit time and velocity, and the distance covered in the
        # intersection.
        t_exit_guess, v_guess = RoadLane._parameterized_exit(
            t_brake_guess, reaches_v_max_before_intersection, v0, a, b, v_max,
            x_to_intersection, x_to_v_max, t_to_v_max)
        x_guess = RoadLane._x_in_intersection(v_guess, v_max, a, t_crit,
                                              t_exit_guess)

        # Check if we've found the optimal t_brake.
        if x_guess > x_crit:  # need to brake more to not overtake
            t_left = t_brake_guess
        elif x_guess < x_crit:  # need to brake less to get closer
            t_right = t_brake_guess
        else:  # braking just enough, so we're done
            t_left = t_right = t_brake_guess

        return t_left, t_right, t_exit_guess, v_guess

    @staticmethod
    def _reaches_v_max_before_intersection(
        t_brake_guess: float, t_brake_largest_seen_at_v_max: Optional[float],
        t_brake_smallest_seen_no_v_max: Optional[float], b: float,
        v_max: float, x_to_v_max: float, x_to_intersection: float
    ) -> Tuple[bool, Optional[float], Optional[float]]:
        """Find if the brake time reaches the speed limit before exiting.

        Also updates the known region of speed updates.
        """
        if (t_brake_largest_seen_at_v_max is not None) and \
                (t_brake_guess <= t_brake_largest_seen_at_v_max):
            # The smaller t_brake is, the more likely the vehicle will
            # reach the speed limit.
            reaches_v_max_before_intersection = True
        elif (t_brake_smallest_seen_no_v_max is not None) and \
                (t_brake_guess >= t_brake_smallest_seen_no_v_max):
            # The larger t_brake is, the more likely the vehicle won't
            # reach the speed limit.
            reaches_v_max_before_intersection = False
        else:
            # This braking time may hit the speed limit. Check here.
            x_brake_guess_from_v_max = RoadLane._x_over_constant_a(
                v_max, -b, t_brake_guess)

            if x_to_v_max + x_brake_guess_from_v_max > x_to_intersection:
                # Braking must start before reaching v_max.
                reaches_v_max_before_intersection = False
                t_brake_smallest_seen_no_v_max = t_brake_guess
            else:
                # Vehicle reaches v_max before this braking time kicks in.
                reaches_v_max_before_intersection = True
                t_brake_largest_seen_at_v_max = t_brake_guess
        return (reaches_v_max_before_intersection,
                t_brake_largest_seen_at_v_max, t_brake_smallest_seen_no_v_max)

    @staticmethod
    def _parameterized_exit(t_brake_guess: float,
                            reaches_v_max_before_intersection: bool, v0: float,
                            a: float, b: float, v_max: float,
                            x_to_intersection: float, x_to_v_max: float,
                            t_to_v_max: float) -> Tuple[float, float]:
        """Find the time and velocity of exit given t_brake_guess."""
        if reaches_v_max_before_intersection:
            x_brake_guess_from_v_max = RoadLane._x_over_constant_a(
                v_max, -b, t_brake_guess)
            x_guess_at_v_max = RoadLane._x_at_v_max(
                x_to_intersection, x_to_v_max, x_brake_guess_from_v_max)
            t_exit_guess = RoadLane._t_of_v_max_exit(
                t_to_v_max, t_brake_guess, x_guess_at_v_max, v_max)
            v_guess = v_max - t_brake_guess*b
        else:
            # This braking time will not hit the speed limit.
            t_exit_guess = (-v0 + (v0**2 + 2*a**2*t_brake_guess**2 +
                                   2*a*b*t_brake_guess**2 +
                                   4*a*x_to_intersection)**(.5)) / a
            v_guess = v0 + a*t_exit_guess - (a+b)*t_exit_guess
            # See dissertation appendix for derivation.
        return t_exit_guess, v_guess

    @staticmethod
    def _x_in_intersection(v_guess: float, v_max: float, a: float,
                           t_crit: float, t_exit_guess: float) -> float:
        """Find how far into the intersection this vehicle gets at t_crit."""
        t_i_guess = t_crit - t_exit_guess
        if v_guess + a*t_i_guess > v_max:
            # Vehicle reaches the speed limit before t_crit.
            t_i_guess_to_v_max = RoadLane._t_to_v(v_guess, a, v_max)
            x_i_guess_to_v_max = RoadLane._x_over_constant_a(
                v_guess, a, t_i_guess_to_v_max)
            t_i_guess_at_v_max = t_i_guess - t_i_guess_to_v_max
            x_i_guess_at_v_max = RoadLane._x_over_constant_a(
                v_max, 0, t_i_guess_at_v_max)
            return x_i_guess_to_v_max + x_i_guess_at_v_max
        else:
            # Vehicle does not reach the speed limit before t_crit.
            return RoadLane._x_over_constant_a(v_guess, a, t_i_guess)

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
        # TODO: (low) This is intended to be used by intersection.tiling but
        #       isn't, as it currently uses a heuristic where its starts
        #       testing with the assumption that at this_exit, the vehicle's
        #       front section has just barely crossed the intersection line
        #       instead of the high-fidelity estimate this function is meant
        #       to implement. The estimate is correct to within one timestep,
        #       more or less, so it should suffice, but we could implement this
        #       function to increase marginal accuracy.
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

    # Misc functions

    def clone(self: L) -> L:
        """Return a copy of the lane with vehicles and downstream removed."""
        # clone = super().clone()
        clone = copy(self)
        clone.vehicles = []
        clone.vehicle_progress = {}
        clone.downstream_is_remover = True
        clone.downstream_intersection = None
        clone.latest_scheduled_exit = None
        # TODO: (clarity) this implementation short circuits intended behavior
        #       by claiming that the cloned RoadLane doesn't end at an
        #       intersection regardless of if it actually does, so that it
        #       doesn't try calling the intersection it was originally
        #       connected to and mixing the cloned and original environments.
        #       This is helpful for check_request usage but not intended
        #       behavior; consider adding overrides to the constructor for this
        #       use case.
        return clone
