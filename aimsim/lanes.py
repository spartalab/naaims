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

import heapq
from abc import ABC, abstractmethod
from typing import Tuple, Iterable, Optional, List, NamedTuple, Dict

import aimsim.settings as SETTINGS
from .archetypes import Upstream
from .util import (Coord, CollisionError, VehicleSection,
                   TooManyProgressionsError, VehicleTransfer)
from .intersections.reservations import ReservationRequest
from .vehicles import Vehicle
from .trajectories import Trajectory


class Lane(ABC):

    def __init__(self,
                 trajectory: Trajectory,
                 offset: Coord = Coord(0, 0),
                 speed_limit: int = SETTINGS.speed_limit) -> None:
        """Create a new lane.

        Lanes inherit the trajetory of the road they're created by (if
        applicable), adding an offset if necessary.

        Keyword arguments:
        trajectory: The trajectory of the road that contains the lane.
        offset: The amount by which to offset the lane by.
        """
        # initialize lane
        self.vehicles: List[Vehicle] = []
        self.vehicle_progress: Dict[VehicleSection, float] = {}
        self.trajectory = trajectory
        self.speed_limit = speed_limit
        self.request_has_changed = False

        # TODO: intersection lanes are not allowed be be offset, maybe update
        #       the generic lane to reflect that?
        self.start_coord: Coord = Coord(trajectory.start_coord.x + offset.x,
                                        trajectory.start_coord.y + offset.y)
        self.end_coord: Coord = Coord(trajectory.end_coord.x + offset.x,
                                      trajectory.end_coord.y + offset.y)

        raise NotImplementedError("TODO")

    def enter_vehicle_section(self, vehicle_transfer: VehicleTransfer) -> None:
        """Adds a section of a transferring vehicle to the lane."""
        # TODO: add a vehicle to this lane and progress it forward using the
        #       time in the timestep it has left
        raise NotImplementedError("TODO")

    def lane_following_update(self, vehicle: Vehicle,
                              preceding: Optional[Vehicle]):
        """Update vehicle's speed and acceleration per lane following behavior.

        Note that this does NOT change the vehicle's position.

        `preceding` is a pointer to the vehicle ahead of it. If it doesn't
        exist, the vehicle is at the head of the queue and must stop at the
        intersection line if it doesn't have a reservation.
        """
        raise NotImplementedError("TODO")


class RoadLane(Lane):
    """RoadLanes connect intersections, providing them access and egress.

    Like their parent Roads, RoadLanes are divided into three sections: the
    entrance, lane changing, and approach regions.

    They connect directly to the conflict area controlled by `manager`s.
    """

    def step_approach(self) -> Optional[VehicleTransfer]:
        """Step vehicles in the approach region forward."""

        # save a pointer to the preceding vehicle before operating on the next
        # one so you can make sure you don't crash with it in the worst case
        preceding: Optional[Vehicle] = None
        # TODO: how to handle the first vehicle in the lane? write a special
        #       case or make a dummy vehicle that has v and a=0 at the
        #       intersection line?
        #       the first vehicle already has a lot of special checks so maybe
        #       having it not in the for loop is fine. but then we'd have to do
        #       something like
        #           itercars = iter(cars)
        #           next(itercars)
        #           for car in itercars:

        # TODO: other than the first vehicle, the logic in step_approach,
        #       step_entrance, and reservation request calculations maybe are
        #       very similar. consider creating a shared private function to
        #       avoid repeating code.

        # TODO: set flag for if the ReservationRequest has changed since the
        #       last setup. this only applies when we cache requests instead of
        #       responding in real time (e.g. for non-FCFS systems), but should
        #       be implemented for use in cached. the reservation changes iff a
        #       new vehicle arrives right behind the current group of vehicles
        #       that are waiting to turn with the same movement AND the manager
        #       has set a delay until the next accepted request (batching).

        # TODO: A sequence of consecutive vehicles behaves like this: all
        #       vehicles accelerate
        #       as much as possible (brake as slowly as possible), until one of
        #       them breaks the intersection line, at which point they slow
        #       down to the acceleration/braking rate of the worst vehicle, in
        #       order to maintain a constant length. The RoadLane needs to
        #       handle this change in acceleration by replacing the vehicles in
        #       a Sequence with the Sequence Vehicle-like in-place as soon as
        #       the first vehicle in the Sequence breaks the intersection line.

        leaving: Optional[VehicleTransfer] = None

        for vehicle in self.vehicles:

            to_check: Iterable[VehicleSection] = list(VehicleSection)

            if self.vehicle_progress[VehicleSection.FRONT] is None:
                # This vehicle has entered the intersection and the
                # responsibility of updating its reference position
                # (vehicle.pos) now belongs to the intersection.

                # However, because roads and intersections can be updated
                # asynchronously, we need to infer its new position in the
                # context of this lane.

                # By our model, since the vehicle has entered the intersection,
                # it has a reservation and is accelerating as much as possible.

                # update the vehicle's properties (pos, speed, accel) according
                # to its kinematics as far as possible without hitting the back
                # of `preceding`.
                #
                # If a vehicle has a reservation, move it as if it doesn't have
                # to stop at the intersection line. otherwise, move it as if it
                # does have to stop.
                to_check = [VehicleSection.CENTER, VehicleSection.REAR]
            else:
                # Calculate the movement of the vehicle based on its front
                # position and propogate it to the vehicle's properties.
                pass

            for section in to_check:
                # update the lane's record of the vehicle's progress according
                # to its properties. (This allows for a vehicle to be
                # controlled by its front, if it's in a different lane, and
                # have its movement propogated back to its back sections)
                raise NotImplementedError("TODO")

            preceding = vehicle

            # break out of the loop once you exit the approach region
            # if something:
            # break

        return leaving

        # TODO: old code. adapt or remove.
        # timestep = 5.0
        # for x in range(len(self.vehicles)):
        #     # x_vehicle is the current vehicle
        #     # b_vehicle is the vehicle that is ahead of the x_vehicle
        #     # timestep is the time step
        #     i_vehicle = self.vehicles[x]
        #     h_vehicle = self.vehicles[x-1] if x != 0 else Vehicle(
        #         self, None, 0, 0, 4.5, 3, 0)

        #     # --- this section taken from another function(?)
        #     i_vehicle.a = (
        #         i_vehicle.a_max if i_vehicle.v < self.speed_limit else 0)
        #     i_vehicle.v = min(i_vehicle.v + i_vehicle.a *
        #                       timestep, self.speed_limit)

        #     i_vehicle.p += (i_vehicle.v * timestep) / self.length
        #     # TODO: keep track of the (x,y) coordinates
        #     if x == 0 and i_vehicle.p >= 1:
        #         i_vehicle.vehicles.popleft()
        #     # --- end external(?) section

        #     # If we can enter the intersection and the proportion completed is 1
        #     if x == 0 and i_vehicle.enter_intersection == True and i_vehicle.p >= 1:
        #         i_vehicle.a = i_vehicle.a_max if i_vehicle.v < self.speed_limit else 0
        #         i_vehicle.v = min(i_vehicle.v + i_vehicle.a *
        #                           timestep, self.speed_limit)

        #         i_vehicle.p = ((i_vehicle.p + (i_vehicle.v*timestep) /
        #                         self.length) - 1) * (self.length / self.trajectory.length)
        #         # TODO: keep track of the (x,y) coordinates
        #         self.trajectory.vehicles.append(self.vehicles.popleft())
        #         x = 0
        #         continue

        #     # Following the equations per the spec
        #     prog_h = (h_vehicle.l / self.length) + ((timestep *
        #                                              (h_vehicle.v - h_vehicle.b_max * timestep) / self.length))
        #     prog_curr = (timestep * (i_vehicle.v -
        #                              i_vehicle.b_max * timestep)) / self.length
        #     if ((h_vehicle.p - prog_h) >= (i_vehicle.p + prog_curr)):
        #         if (i_vehicle.v < self.speed_limit):
        #             i_vehicle.a = i_vehicle.a_max
        #         elif (i_vehicle.v == self.speed_limit):
        #             i_vehicle.a = 0
        #     else:
        #         i_vehicle.a = i_vehicle.b

        #     i_vehicle.v = min(i_vehicle.v + i_vehicle.a *
        #                       timestep, self.speed_limit)
        #     i_vehicle.p += (timestep * (i_vehicle.v +
        #                                 i_vehicle.a * timestep)) / self.length

        #     # TODO: keep track of the (x,y) coordinates

    def step_entrance(self) -> None:
        """Step vehicles in the entrance region forward."""
        preceding = None
        for vehicle in self.vehicles:
            raise NotImplementedError("TODO")
            preceding = vehicle

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


class IntersectionLane(Lane):
    """


    Unlike RoadLanes, IntersectionLanes can potentially be uncertain.

    An IntersectionLane for when you're not sure if the vehicle using it is
    going to follow instructions exactly 100% of the time. Here, instead of
    being the ground truth, the trajectory is just a suggestion and in step()
    vehicles' properties (pos/v/a/heading) are updated with some amount of
    randomness off of the centerline trajectory.
    """

    def __init__(self, trajectory, length, enteringIntersection=False):
        super().__init__(trajectory, length, enteringIntersection=False)

        # TODO: do we check for conflicts here? or just leave it?
        #       maybe we need to be able to rapidly duplicate these since the
        #       intersection may create a new lane based on a blueprint for
        #       new reservation, unlike RoadLanes which are static after
        #       instantiation.
        raise NotImplementedError("TODO")

    def step(self) -> Optional[VehicleTransfer]:
        """

        Uses  equations defined in the spec to progress the completion of each
        vehicle. The general logic of the code is to iterate through all the
        vehicles and then progress each car. We then check to see if there is a
        car that can enter the intersecton and then pop it off from the queue
        and return it to the parent road or intersection so they can figure out
        what to do with it.

        Alternatively, for each vehicle at completion percentage, see if we can
        move it forward. If the head vehicle can get a reservation given an
        approximate arrival time then we should remove the vehicle from the
        dictionary

        Returns
            A VehicleTransfer object if a vehicle leaves the lane.
        """

        # TODO: implement stochasticity in IntersectionLane
        #       If vehicles deviate from centerline, it will be because their
        #       IntersectionLane told them to. IntersectionLane has flag for if
        #       a vehicle will stochastically deviate from acceleration
        #       instructions and by how much, but otherwise progression is like
        #       RoadLane, except IntersectionLane doesn't need to calculate how
        #       to avoid collisions since the reservations system takes care of
        #       that already (unless we have stochastic movement and get really
        #       bad RNG).

        # TODO: when the vehicle exits the intersection lane, remember to set
        #       its has_reservation property to false

        raise NotImplementedError("TODO")
