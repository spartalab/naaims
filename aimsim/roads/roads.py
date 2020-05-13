"""
A road is a collection of lanes moving in one direction that connects
intersections to each other.

Roads are divided into up to 3 sections:
    1. The entrance region, the section immediately following an
       intersection. This section is as long as the distance it would take the
       longest vehicle the lane may need to handle to come to a full and
       complete stop in order to ensure that the vehicles can always safely
       exit the intersection. Vehicles are not allowed to change lanes in this
       section, though this section need not always be clear.
    2. The lane changing region, which follows the entrance region. This
       section is the only area in which vehicles are allowed to change lanes.
       The length of this section is variable, with no maximum and a minimum
       length determined by the RoadManager implemented.
    3. The approach region, which follows the lane changing region and precedes
       the next intersection. This area has a minimum length of 0, which may be
       because there is no room left or because there is no next intersection
       (i.e, if this road terminates at the end of the simulation area into a
       vehicle collector). Like the entrance region, lane changes are barred
       in this section.
TODO: Not all sections need to be implemented in case the road is attached to
      a spawner or remover.

The vehicle transmission and entrance interfaces provide a consistent way for
roads and intersections to communicate with each other when scheduling the
passage of a vehicle from one to the other and avoid collisions.
"""


from abc import abstractmethod
from typing import (TYPE_CHECKING, Type, Optional, Iterable, Union, Dict, Any,
                    Tuple, NamedTuple, Set)
from __future__ import annotations

import aimsim.settings as SETTINGS
from ..vehicles import Vehicle
from ..trajectories import Trajectory
from ..lanes import RoadLane
from ..util import (Coord, CollisionError, DownstreamError, VehicleTransfer,
                    TooManyProgressionsError)
from ..archetypes import Configurable, Facility, Upstream, Downstream
from ..intersections.reservations import ReservationRequest
from .managers import LaneChangeManager, DummyManager

if TYPE_CHECKING:
    from ..intersections import Intersection
    from ..endpoints import VehicleRemover


class Road(Configurable, Facility, Upstream, Downstream):
    """
    ...

    A road's trajectory object describes the centerline of the road, assuming
    that all roads are symmetrical left to right, and that where one lane ends
    and the other begins, i.e., that there are no medians. (If there is a
    median, create a new road.)

    A road contains one or more lanes that hold vehicles.

    These lanes share the same trajectory object as the road at large,
    clarified plus a unique offset (including lane spacing and angle)
    for each lane to account for the spacing between lanes.

    """

    def __init__(self, trajectory: Trajectory,
                 num_lanes: int = 1,
                 lane_width: float = 4,  # meters
                 lane_offset_angle: Optional[float] = None,  # degrees
                 len_entrance_region: float = SETTINGS.min_entrance_length,
                 len_approach_region: float = 100,  # meters
                 speed_limit: int = settings.speed_limit,
                 manager: Optional[Type[LaneChangeManager]] = None,
                 manager_config: Optional[Dict[str, Any]] = None) -> None:
        """Create a new road.

        Parameters:
            trajectory: Trajectory
                The trajectory of the centerline of the road.
            num_lanes: int
                Number of lanes the road has
            lane_offset_angle: float
                Angle by which to offset each lane
            len_entrance_region: float
                How long the entrance region is
            len_approach_region: float
                How long the approach region is
            speed_limit: int
                Speed limit on this road in km/h
            manager: Optional[Type[LaneChangeManager]]
                What type of LaneChangeManager to use to handle lane changes.
                If a LaneChangeManager is not provided, lane changes are
                prohibited, there is no lane changing region, and the entrance
                and approach regions are combined.
            manager_config: Dict[str, Any]
                What settings to give to the manager constructor

        TODO: change len_entrance_region, len_approach_region defaults
        """

        # error checking
        if num_lanes < 1:
            raise ValueError('Must have more than one lane.')
        elif (num_lanes > 1) and (lane_offset_angle is None):
            raise ValueError(
                'Need lane_offset if there\'s more than one lane.')
        elif ((lane_offset_angle is not None) and
              (lane_offset_angle < 0 or lane_offset_angle >= 360)):
            raise ValueError('lane_offset_angle must be between [0,360) deg')
        elif (
            (manager is None) and (trajectory.length < max(
                len_entrance_region, len_approach_region
            ))
        ) or (
            trajectory.length < len_entrance_region+len_approach_region
        ):
            raise ValueError(
                'Road is not long enough for the region lengths specified.'
            )
            # TODO: the lane change region, if it exists, also has to be some
            #       min length to allow for a vehicle to accelerate from 0 and
            #       pass
        elif lane_width <= 0:
            # TODO: maybe more stringent lane width check
            raise ValueError('Need positive lane width.')
        if len_entrance_region + len_approach_region > trajectory.length:
            raise ValueError('Sum of regions longer than trajectory.')

        self.trajectory = trajectory
        # may not be necessary to store the following explicitly
        self.num_lanes = num_lanes
        self.lane_width = lane_width
        self.lane_offset_angle = lane_offset_angle
        self.speed_limit = speed_limit

        # initialize data structures
        self.lanes: Tuple[RoadLane] = tuple([RoadLane(
            trajectory=trajectory,
            offset=Coord(0, 0)  # TODO: calculate this correctly
        ) for i in range(num_lanes)])
        if manager is not None:
            self.manager = manager(self.lanes, manager_config)
        else:
            self.manager = None

        self.lanes_by_downstream_coord: Dict[Coord, RoadLane] = {
            l.trajectory.start_coord: l for l in self.lanes
        }

        raise NotImplementedError("TODO")

        # init buffer for incoming vehicles
        Upstream.__init__(self)
        Downstream.__init__(self)

    # TODO: finish stitching
    def unconnected_upstreams(self) -> Optional[Iterable[Coord]]:
        """Return unconnected upstream coordinates, if any."""
        return (None if (self.upstream is not None) else None)
        # TODO: fix

    def connect_upstreams(self, upstreams: Iterable[Downstream]) -> None:
        """Finalize connecting upstream object(s)."""
        # if upstream don't match the lanes we expect, raise value error
        # else link it up
        raise NotImplementedError("Must be implemented in child class.")

    # TODO delete
    def finish_setup(self, incoming: Upstream, outgoing: Downstream) -> None:
        """Connect up and downstream objects to this road after creation."""
        self.upstream = incoming
        self.downstream = outgoing
        raise NotImplementedError("TODO")

    def update_speeds(self) -> None:
        """Update speed and acceleration for all vehicles on this road.

        This road is responsible for updating the speed and acceleration of all
        vehicles on this road that aren't partially in an intersection.
        """

        # error checking
        if self.downstream is None:
            raise DownstreamError("No downstream object.")
        elif ((self.downstream is not Intersection)
              and (self.downstream is not VehicleRemover)):
            raise DownstreamError(
                "Downstream object is not Intersection or VehicleRemover.")

        if self.manager is not None:
            # three separate road regions. process accordingly.
            raise NotImplementedError("TODO")
        else:
            # no lane change region. approach and entrance regions are combined
            # so there's only one continuous region.
            for lane in self.lanes:
                lane.update_speeds()

    def step(self) -> None:
        """Update the positions of all vehicles, passing them on if exiting."""

        # move vehicles in the approach region forward, passing vehicles if
        # necessary
        for lane in self.lanes:
            leaving = lane.step_approach()
            if leaving is not None:
                self.downstream.transfer_vehicle(leaving)

        # # next, have the LaneChangeManager move the vehicles in the middle
        # punt TODO: skipping designing the API for the LaneChangeManager
        #            until the basics are complete.

        # finally, move vehicles in the entrance region forward
        for lane in self.lanes:
            lane.step_entrance()

    def process_transfers(self) -> None:
        """Incorporate new vehicles onto this road."""
        # TODO: verify this implementation
        while len(self.entering_vehicle_buffer) > 0:
            transfer = self.entering_vehicle_buffer.pop()
            # TODO: consider checking if a lane gets more than one vehicle
            # added in a cycle. if so, raise TooManyProgressionsError
            if transfer.pos not in self.lanes_by_downstream_coord:
                raise ValueError('Lane not in this road.')
            self.lanes_by_downstream_coord[transfer.pos].enter_vehicle_section(
                transfer)
        super().process_transfers()  # just makes sure the list is empty after

    def handle_logic(self) -> None:
        # # tell RoadManager to schedule the next set of lane changes
        # punt TODO: skipping designing the API for the LaneChangeManager
        #            until the basics are complete.
        pass

    def room_to_enter(self, lane_coord: Coord) -> float:
        """If the vehicles in the entrance region stop, how much room is there?

        Used to ensure that entering vehicles don't collide with vehicles just
        outside of the intersection.
        """
        raise NotImplementedError("TODO")

    def check_entrance_collision(self, lane: RoadLane, steps_forward: int,
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
            lane : RoadLane
                which lane does it enter at
            entering_vehicle : Vehicle
                the vehicle entering
            entering_v0 : float
                at steps_forward, what is its speed
            entering_accel_profile : Iterable[float]
                what this vehicle's acceleration from the time it starts
                entering to the time it finishes entering the lane
        """

        if lane in self.lanes:
            lane.check_entrance_collision(steps_forward,
                                          entering_vehicle,
                                          entering_v0,
                                          entering_accel_profile)
        else:
            raise ValueError('Lane not in this road.')

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

    def get_reservation_candidates(self,
                                   skip: Optional[Set[Coord]] = None,
                                   group: bool = False,
                                   all_groups: bool = False,
                                   delay: int = 0
                                   ) -> Iterable[ReservationRequest]:
        """Return reservation requests from each lane, if there are any.

        (Note that the ReservationRe)

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
        # TODO: check
        return [l.next_reservation_candidate() for l in self.lanes]

    def __hash__(self):
        return hash((self.trajectory.start_coord, self.trajectory.end_coord))
