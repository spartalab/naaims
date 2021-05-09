"""
A road is a collection of lanes moving in one direction that connects
intersections to one another as well as vehicle spawn and removal points.

Roads are divided into up to 3 sections:
    1. The entrance region, the section immediately following an
       intersection. This section needs to be long enough to accommodate the
       longest vehicle that will be spawned in the AIM simulation. Vehicles are
       not allowed to change lanes in this section, though this section need
       not always be clear.
    2. The lane changing region, which follows the entrance region. This
       section is the only area in which vehicles are allowed to change lanes.
       The length of this section is variable; its length can be 0 but if it
       isn't the RoadManager implemented will dictate a minimum length.
    3. The approach region, which follows the lane changing region and precedes
       the next intersection. This area must be at least as long as the longest
       possible vehicle. Like the entrance region, lane changes are barred in
       this section.
"""


from __future__ import annotations
from typing import Optional, TYPE_CHECKING, Type, Iterable, Dict, Any, Tuple, List
from math import pi, sin, cos

from aimsim.vehicles import Vehicle
from aimsim.trajectories import Trajectory, BezierTrajectory
from aimsim.road.lane import RoadLane
from aimsim.util import (Coord, MissingConnectionError, VehicleTransfer,
                         SpeedUpdate)
from aimsim.archetypes import Configurable, Facility, Upstream, Downstream
from aimsim.road.managers import LaneChangeManager, DummyManager
from aimsim.intersection import Intersection

if TYPE_CHECKING:
    from aimsim.endpoints import VehicleSpawner, VehicleRemover


class Road(Configurable, Facility, Upstream, Downstream):
    """
    A road contains one or more lanes that hold vehicles and several support
    functions that allow Downstream intersections and vehicle removers to
    interface with vehicles exiting the road and intersections to schedule new
    arrivals and prevent collisions.

    A road's trajectory object describes the centerline of the road.
    """

    def __init__(self,
                 trajectory: Trajectory,
                 len_entrance_region: float,
                 speed_limit: int,
                 upstream_is_spawner: bool,
                 downstream_is_remover: bool,
                 manager_type: Type[LaneChangeManager] = DummyManager,
                 manager_spec: Dict[str, Any] = {},
                 num_lanes: int = 1,
                 lane_width: float = 4,  # meters
                 lane_offset_angle: float = 0,  # radians
                 len_approach_region: float = 100  # meters
                 ) -> None:
        """Create a new road.

        Parameters:
            trajectory: Trajectory
                The trajectory of the centerline of the road, assuming that all
                roads are symmetrical left to right. Lanes share the same
                trajectory as their parent road, plus a unique offset vector
                for each lane found using num_lanes and lane_offset_angle.
            len_entrance_region: float
                How long the entrance region is in meters
            speed_limit: int
                Speed limit on this road in km/h
            manager_type: Type[LaneChangeManager]
                What type of LaneChangeManager to use to handle lane changes
            manager_spec: Dict[str, Any]
                What specifications to give to the manager constructor
            upstream_is_spawner: bool
                Whether the upstream object is a spawner or intersection
            downstream_is_remover: bool
                Whether the downstream object is a remover or intersection
            num_lanes: int
                Number of lanes the road has
            lane_width: float
                The width of each lane (assumed equal for every lane) in meters
            lane_offset_angle: float
                Angle by which to offset each lane in radians. Measured
                relative to the lane_width, i.e., perpendicular to the
                direction of the road. If your road vector, points to the
                right, 0 lane_offset_angle points up. pi/2 lane_off_set angle
                will see your lanes offset from bottom left to top right.
            len_approach_region: float
                How long the approach region is in meters

        TODO: Finalize len_entrance_region and len_approach_region usage as
              as well as their defaults.
        """

        # error checking
        if num_lanes < 1:
            raise ValueError('Must have more than one lane.')
        if (num_lanes > 1) and (lane_offset_angle is None):
            raise ValueError(
                'Need lane_offset if there\'s more than one lane.')
        if ((lane_offset_angle is not None) and
                (lane_offset_angle <= -pi/2 or lane_offset_angle >= pi/2)):
            raise ValueError('lane_offset_angle must be between +/- pi/2.')
        if (
            (manager_type is DummyManager) and (trajectory.length < max(
                len_entrance_region, len_approach_region
            ))
        ) or (trajectory.length < len_entrance_region+len_approach_region):
            raise ValueError(
                'Road is not long enough for the region lengths specified.'
            )
        if lane_width <= 0:
            # TODO: (low) Maybe add a more stringent lane width check.
            raise ValueError('Need positive lane width.')
        if len_entrance_region + len_approach_region > trajectory.length:
            raise ValueError('Sum of regions longer than trajectory.')

        self.trajectory = trajectory
        self.upstream_is_spawner = upstream_is_spawner
        self.downstream_is_remover = downstream_is_remover
        self.num_lanes = num_lanes
        self.lane_width = lane_width
        self.lane_offset_angle = lane_offset_angle
        self.speed_limit = speed_limit

        # Calculate lane offsets
        spacing = Coord(lane_width*sin(lane_offset_angle),
                        lane_width*cos(lane_offset_angle))
        offset_count = (num_lanes-1)/2
        starting_offset: Coord = Coord(-offset_count*spacing.x,
                                       -offset_count*spacing.y)
        offsets = [Coord(starting_offset.x + i*spacing.x,
                         starting_offset.y + i*spacing.y)
                   for i in range(num_lanes)]

        # Create support structures
        self.lanes: Tuple[RoadLane, ...] = tuple([RoadLane(
            trajectory=trajectory,
            width=lane_width,
            speed_limit=speed_limit,
            len_entrance_region=len_entrance_region,
            len_approach_region=len_approach_region,
            offset=offset,
            upstream_is_spawner=upstream_is_spawner,
            downstream_is_remover=downstream_is_remover
        ) for offset in offsets])
        self.manager: LaneChangeManager = manager_type.from_spec(manager_spec)

        # Organize lanes
        self.lanes_by_start: Dict[Coord, RoadLane] = {
            lane.trajectory.start_coord: lane for lane in self.lanes
        }
        self.lanes_by_end: Dict[Coord, RoadLane] = {
            lane.trajectory.end_coord: lane for lane in self.lanes
        }

        # Init buffer for incoming vehicles
        Downstream.__init__(self)

        # Setup upstream and downstream items
        self.__upstream: Optional[Upstream] = None
        self.__downstream: Optional[Downstream] = None

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a road spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: (spec) Interpret the string into the spec dict.
        raise NotImplementedError("TODO")

        # TODO: (spec) Enforce provision of separate trajectory_type and
        #       trajectory_config fields in road spec string.

        trajectory_type: str
        trajectory_config: Dict[str, Any]
        # Based on the spec, identify the correct trajectory type
        if trajectory_type.lower() in {'', 'bezier', 'beziertrajectory'}:
            spec['trajectory'] = BezierTrajectory.from_spec(
                BezierTrajectory.spec_from_str(trajectory_config)
            )
        else:
            raise ValueError("Unsupported Trajectory type.")

        # TODO: (spec) Enforce provision of separate manager_type and
        #       manager_config fields in road spec string.

        manager_type: str
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
            num_lanes=spec['num_lanes'],
            lane_width=spec['lane_width'],
            upstream_is_spawner=spec['upstream_is_spawner'],
            downstream_is_remover=spec['downstream_is_remover'],
            manager_type=spec.get('manager_type', DummyManager),
            manager_spec=spec.get('manager_spec', {}),
            lane_offset_angle=spec['lane_offset_angle'],
            len_entrance_region=spec['len_entrance_region'],
            len_approach_region=spec['len_approach_region'],
            speed_limit=spec['speed_limit']
        )

    def connect_upstream(self, upstream: Upstream) -> None:
        """Finalize connecting upstream object."""
        if ((self.upstream_is_spawner and (type(upstream) is Intersection)) or
                ((not self.upstream_is_spawner)
                 and (type(upstream) is VehicleSpawner))):
            raise ValueError("Incorrect Upstream type.")
        self.__upstream = upstream

    def connect_downstream(self, downstream: Downstream) -> None:
        """Finalize connecting downstream object."""
        if ((self.downstream_is_remover and (type(downstream) is Intersection))
            or ((not self.downstream_is_remover)
                and (type(downstream) is VehicleRemover))):
            raise ValueError("Incorrect Downstream type.")
        self.__downstream = downstream
        if type(downstream) is Intersection:
            for lane in self.lanes:
                lane.connect_downstream_intersection(
                    downstream)  # type: ignore

    @property
    def upstream(self) -> Upstream:
        if self.__upstream is None:
            raise MissingConnectionError("No upstream object.")
        return self.__upstream

    @property
    def downstream(self) -> Downstream:
        if self.__downstream is None:
            raise MissingConnectionError("No downstream object.")
        return self.__downstream

    # Begin simulation cycle methods

    def get_new_speeds(self) -> Dict[Vehicle, SpeedUpdate]:
        """Return speed and acceleration update for all vehicles on this road.

        This road is responsible for updating the speed and acceleration of all
        vehicles on this road that aren't partially in an intersection.
        """

        new_speeds: List[Dict[Vehicle, SpeedUpdate]] = []

        # Update speed and acceleration for vehicles lane-by-lane. For each
        # lane, poll the LaneChangeManager for a Set of vehicles that the LCM
        # is instructing to slow down to allow for another vehicle to merge in.
        for lane in self.lanes:
            new_speeds.append(
                lane.get_new_speeds(
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
            transfer = self.entering_vehicle_buffer.pop(0)
            if transfer.pos not in self.lanes_by_start:
                raise RuntimeError('Lane not in this road.')
            self.lanes_by_start[transfer.pos].enter_vehicle_section(transfer)
        super().process_transfers()  # just makes sure the list is empty after

    def update_schedule(self) -> None:
        """Tell LaneChangeManager to schedule the next set of lane changes."""

        # TODO: (platoon) Make or break vehicle chains here.

        self.manager.update_schedule()

    # Misc functions

    def __hash__(self) -> int:
        return hash((self.trajectory.start_coord, self.trajectory.end_coord))
