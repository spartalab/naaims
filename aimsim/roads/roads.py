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

The vehicle transmission and entrance interfaces provide a consistent way for
roads and intersections to communicate with each other when scheduling the
passage of a vehicle from one to the other and avoid collisions.
"""


from __future__ import annotations
from abc import abstractmethod
from typing import (TYPE_CHECKING, Type, Optional, Iterable, Union, Dict, Any,
                    Tuple, NamedTuple, Set, List)

import aimsim.shared as SHARED
from ..vehicles import Vehicle
from ..trajectories import Trajectory, BezierTrajectory
from ..lanes import RoadLane
from ..util import (Coord, CollisionError, LinkError, VehicleTransfer,
                    TooManyProgressionsError, SpeedUpdate)
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

    def __init__(self,
                 trajectory: Trajectory,
                 manager_type: Type[LaneChangeManager],
                 manager_spec: Dict[str, Any],
                 num_lanes: int = 1,
                 lane_width: float = 4,  # meters
                 lane_offset_angle: Optional[float] = None,  # degrees
                 len_entrance_region: float = SHARED.min_entrance_length,
                 len_approach_region: float = 100,  # meters
                 v_max: int = SHARED.speed_limit) -> None:
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
            v_max: int
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
        if (num_lanes > 1) and (lane_offset_angle is None):
            raise ValueError(
                'Need lane_offset if there\'s more than one lane.')
        if ((lane_offset_angle is not None) and
                (lane_offset_angle < 0 or lane_offset_angle >= 360)):
            raise ValueError('lane_offset_angle must be between [0,360) deg')
        if (
            (manager_type is DummyManager) and (trajectory.length < max(
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
        if lane_width <= 0:
            # TODO: maybe more stringent lane width check
            raise ValueError('Need positive lane width.')
        if len_entrance_region + len_approach_region > trajectory.length:
            raise ValueError('Sum of regions longer than trajectory.')

        self.trajectory = trajectory
        self.num_lanes = num_lanes
        self.lane_width = lane_width
        self.lane_offset_angle = lane_offset_angle
        self.v_max = v_max

        # Create support structures
        self.lanes: Tuple[RoadLane, ...] = tuple([RoadLane(
            trajectory=trajectory,
            offset=Coord(0, 0)  # TODO: calculate this correctly
        ) for i in range(num_lanes)])
        self.manager: LaneChangeManager = manager_type.from_spec(manager_spec)

        # Organize lanes
        self.lanes_by_start: Dict[Coord, RoadLane] = {
            l.trajectory.start_coord: l for l in self.lanes
        }
        self.lanes_by_end: Dict[Coord, RoadLane] = {
            l.trajectory.end_coord: l for l in self.lanes
        }

        # Init buffer for incoming vehicles
        Downstream.__init__(self)

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a road spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: interpret the string into the spec dict
        raise NotImplementedError("TODO")

        # TODO: enforce provision of separate trajectory_type and
        #       trajectory_config fields in road spec string

        if trajectory_type.lower() in {'', 'bezier', 'beziertrajectory'}:
            spec['trajectory'] = BezierTrajectory.from_spec(
                BezierTrajectory.spec_from_str(trajectory_config)
            )
        else:
            raise ValueError("Unsupported Trajectory type.")

        # TODO: enforce provision of separate manager_type and manager_config
        #       fields in road spec string

        # Based on the spec, identify the correct manager type
        if manager_type.lower() in {'', 'dummy', 'dummymanager'}:
            spec['manager_type'] = DummyManager
        else:
            raise ValueError("Unsupported LaneChangeManager type.")

        return spec

    @classmethod
    def from_spec(cls, spec: Dict[str, Any]) -> Road:
        """Create a new Road from the output of spec_from_str.

        Roads are constructed first so no post-processing necessary between
        spec_from_str and this function, unlike other Configurables.
        """
        return cls(
            trajectory=spec['trajectory'],
            manager_type=spec['manager_type'],
            manager_spec=spec['manager_spec'],
            num_lanes=spec['num_lanes'],
            lane_width=spec['lane_width'],
            lane_offset_angle=spec['lane_offset_angle'],
            len_entrance_region=spec['len_entrance_region'],
            len_approach_region=spec['len_approach_region'],
            v_max=spec['v_max']
        )

    def connect_upstream(self, upstream: Upstream) -> None:
        """Finalize connecting upstream object."""
        # TODO: check that the upstream thing matches what we expect
        self.upstream = upstream

    def connect_downstream(self, downstream: Downstream) -> None:
        """Finalize connecting downstream object."""
        # TODO: check that the downstream thing matches what we expect
        self.downstream = downstream

    # Begin simulation cycle methods

    def update_speeds(self) -> Dict[Vehicle, SpeedUpdate]:
        """Return speed and acceleration update for all vehicles on this road.

        This road is responsible for updating the speed and acceleration of all
        vehicles on this road that aren't partially in an intersection.
        """

        # Check that Upstream and Downstream objects have been connected.
        # This only needs to be checked the first time but hopefully this
        # runtime is trivial.
        try:
            self.upstream
        except NameError:
            raise LinkError("No upstream object.")
        try:
            self.downstream
        except NameError:
            raise LinkError("No downstream object.")

        new_speeds: List[Dict[Vehicle, SpeedUpdate]] = []

        # Update speed and acceleration for vehicles lane-by-lane. For each
        # lane, poll the LaneChangeManager for a Set of vehicles that the LCM
        # is instructing to slow down to allow for another vehicle to merge in.
        for lane in self.lanes:
            new_speeds.append(
                lane.update_speeds(
                    to_slow=self.manager.vehicles_to_slow(lane)
                )
            )

        # Merge the SpeedUpdates from every lane and section into one dict
        finalized_speed: Dict[Vehicle, SpeedUpdate] = {}
        for new_speed_dict in new_speeds:
            for vehicle, new_speed in new_speed_dict.items():
                if vehicle in finalized_speed:
                    # One vehicle is present in two lanes because it's in the
                    # middle of a lane change. Take the slower speed update of
                    # the two lanes. (Technically speed_updates are tuples so
                    # comparing them is a bit odd but since if the v of the
                    # update is lower the a is as well and vice versa, so this
                    # works.)
                    finalized_speed[vehicle] = min(finalized_speed[vehicle],
                                                   new_speed)
                else:
                    finalized_speed[vehicle] = new_speed
        return finalized_speed

    def step_vehicles(self) -> None:
        """Update all vehicles' positions and transfer them if they exit."""

        # Update the true and lane-relative positions of vehicles along the
        # centerline of their lane. If a vehicle portion exits the lane, pass
        # it back to the road for transferring.
        for lane in self.lanes:
            transfers: Iterable[VehicleTransfer] = lane.step_vehicles(
                lateral_deviations=self.manager.lateral_movements(lane)
            )
            for transfer in transfers:
                self.downstream.transfer_vehicle(transfer)

    def process_transfers(self) -> None:
        """Incorporate new vehicles onto this road."""
        while len(self.entering_vehicle_buffer) > 0:
            transfer = self.entering_vehicle_buffer.pop()
            if transfer.pos not in self.lanes_by_start:
                raise RuntimeError('Lane not in this road.')
            self.lanes_by_start[transfer.pos].enter_vehicle_section(transfer)
        super().process_transfers()  # just makes sure the list is empty after

    def handle_logic(self) -> None:
        """Tell LaneChangeManager to schedule the next set of lane changes."""

        # TODO: (platoon) Make or break vehicle chains here.

        self.manager.handle_logic()

    # Functions used by intersection managers.

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
