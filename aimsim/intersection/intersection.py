"""
This module implements a road intersection with a specific priority control
policy (the manager) and method of registering reservations (the tiling).

The intersection includes incoming roads up to some distance specified in the
constructor. Vehicles on these approach roads are barred from changing lanes
as a necessary prerequisite for the AIM scheduling method.

The intersection handles the creation of all its moving parts and the transfer
of vehicles from incoming roads and to outgoing roads, as well as instructing
the lanes and manager to start their processes.

The intersection lanes handle vehicles' acceleration, speed, and position
updates.

The manager handles how vehicles progress along the lanes and if vehicles get
to come in or not.

The tiling handles how the manager checks for upcoming conflicts.

The Intersection owns the IntersectionManager, which owns the Tiling, which
owns the Tiles.
"""

from __future__ import annotations
from typing import TYPE_CHECKING, Set, Type, Dict, Any, List, Tuple, Optional

from aimsim.archetypes import Configurable, Facility, Upstream, Downstream
from aimsim.util import Coord, VehicleTransfer, SpeedUpdate, VehicleSection
from aimsim.intersection.lane import IntersectionLane
from aimsim.intersection.managers import IntersectionManager, FCFSManager

if TYPE_CHECKING:
    from aimsim.vehicles import Vehicle
    from aimsim.road import Road, RoadLane


class Intersection(Configurable, Facility, Upstream, Downstream):

    def __init__(self,
                 incoming_roads: List[Road],
                 outgoing_roads: List[Road],
                 connectivity: List[Tuple[Road, Road, bool]],
                 manager_type: Type[IntersectionManager],
                 manager_spec: Dict[str, Any],
                 speed_limit: int
                 ) -> None:
        """Create a new intersection.

        Parameters
            incoming_roads: Iterable[Road]
                The roads from which vehicles enter this intersection.
            outgoing_roads: Iterable[Road]
                The roads onto which vehicles exit this intersection.
            connectivity: Iterable[Tuple[Road, Road, bool]]
                Describes which roads connect to each other, used to create
                IntersectionLanes.
                The pattern is a list of 3-tuples which correspond to:
                    0. The incoming Road
                    1. The outgoing Road
                    2. A bool denoting if the two roads are fully connected,
                       e.g., as in a normal through connection.
                If the number of lanes don't match, defer to the incoming Road,
                attaching them to a corresponding lane in the outgoing Road
                that results in the shortest distance and ensures that no
                IntersectionLane created starts or ends at the same node.
                If false, only one Intersection will be created between
                the closest pair of farthest left or right lanes on both
                trajectories. (This approximates a restricted turning
                movement.)
            manager_type: Type[IntersectionManager]
                The type of IntersectionManager to init.
            manager_spec: Dict[str, Any]
                Specifications to create the manager with.
            speed_limit: int = SHARED.speed_limit
                The speed limit.
        """

        # Index the upstream and downstream roads by their lanes' Coords
        self.incoming_road_lane_by_coord: Dict[Coord, RoadLane] = {}
        self.outgoing_road_lane_by_coord: Dict[Coord, RoadLane] = {}
        self.outgoing_road_by_lane_coord: Dict[Coord, Road] = {}
        for r in incoming_roads:
            for coord, lane in r.lanes_by_end.items():
                self.incoming_road_lane_by_coord[coord] = lane
        for r in outgoing_roads:
            for coord, lane in r.lanes_by_start.items():
                self.outgoing_road_lane_by_coord[coord] = lane
                self.outgoing_road_by_lane_coord[coord] = r

        # Given the upstream and downstream roads and connectivity matrix,
        # connect incoming and outgoing RoadLanes with IntersectionLanes.
        lanes: List[IntersectionLane] = []
        for incoming_road, outgoing_road, fully_connected in connectivity:

            # Find the distance between each incoming and outgoing lane.
            io_distances: Dict[RoadLane, Dict[RoadLane, float]] = {}
            for incoming_lane in incoming_road.lanes:
                for outgoing_lane in outgoing_road.lanes:
                    dist = (
                        (incoming_lane.trajectory.end_coord.x -
                         outgoing_lane.trajectory.start_coord.x)**2 +
                        (incoming_lane.trajectory.end_coord.y -
                         outgoing_lane.trajectory.start_coord.y)**2
                    )
                    if incoming_lane not in io_distances:
                        io_distances[incoming_lane] = {}
                    io_distances[incoming_lane][outgoing_lane] = dist

            unmatched_incoming_lanes: Set[RoadLane] = set(incoming_road.lanes)
            unmatched_outgoing_lanes: Set[RoadLane] = set(outgoing_road.lanes)
            while len(unmatched_incoming_lanes) > 0:
                # Search all IO pairs for the shortest distance pair.
                shortest_dist: float = float('inf')
                shortest_pair: Tuple[RoadLane, RoadLane] = (
                    incoming_road.lanes[0], outgoing_road.lanes[0])

                # Avoid assigning multiple incoming lanes to one outgoing lane
                # unless there aren't enough outgoing lanes left.
                enforce_one_to_one = len(unmatched_outgoing_lanes) < \
                    len(unmatched_incoming_lanes)

                for incoming_lane in unmatched_incoming_lanes:
                    search_outgoing = unmatched_outgoing_lanes \
                        if enforce_one_to_one else set(outgoing_road.lanes)
                    for outgoing_lane in search_outgoing:
                        dist = io_distances[incoming_lane
                                            ][outgoing_lane]
                        if dist < shortest_dist:
                            shortest_pair = (incoming_lane, outgoing_lane)
                            shortest_dist = dist

                lanes.append(IntersectionLane(shortest_pair[0],
                                              shortest_pair[1],
                                              speed_limit))
                if fully_connected:
                    # Remove connected lanes from the eligible pool, with a
                    # more lenient check for the outgoing lane because it could
                    # have been a multiple-to-one connection if there weren't
                    # enough outgoing lanes.
                    unmatched_incoming_lanes.remove(shortest_pair[0])
                    unmatched_outgoing_lanes.discard(shortest_pair[1])
                else:
                    # Finding the shortest connection is enough.
                    break

        self.lanes: Tuple[IntersectionLane, ...] = tuple(lanes)

        # Index the IntersectionLanes by their Coord
        self.lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                      IntersectionLane] = {
            (lane.trajectory.start_coord,
             lane.trajectory.end_coord): lane for lane in self.lanes
        }
        self.lanes_by_start: Dict[Coord, List[IntersectionLane]] = {}
        for lane in self.lanes:
            coord = lane.trajectory.start_coord
            if coord not in self.lanes_by_start:
                self.lanes_by_start[coord] = [lane]
            else:
                self.lanes_by_start[coord].append(lane)

        # Finish the spec for the manager by providing the roads and lanes
        manager_spec['incoming_road_lane_by_coord'
                     ] = self.incoming_road_lane_by_coord
        manager_spec['outgoing_road_lane_by_coord'
                     ] = self.outgoing_road_lane_by_coord
        manager_spec['lanes'] = self.lanes
        manager_spec['lanes_by_endpoints'] = self.lanes_by_endpoints

        # Create the manager
        self.manager: IntersectionManager = manager_type.from_spec(
            manager_spec)

        # Init buffer for incoming vehicles
        Downstream.__init__(self)

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a intersection spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: (spec) Interpret the string into the spec dict.
        raise NotImplementedError("TODO")

        # TODO: (spec) Enforce provision of separate manager_type and
        #       manager_config fields in intersection spec string.

        manager_type: str
        # Based on the spec, identify the correct manager type
        if manager_type.lower() in {'fcfs', 'fcfsmanager'}:
            spec['manager_type'] = FCFSManager
        else:
            raise ValueError("Unsupported IntersectionManager type.")

        return spec

    @classmethod
    def from_spec(cls, spec: Dict[str, Any]) -> Intersection:
        """Create a new Intersection from a processed spec.

        Note that what spec_from_str returns needs additional processing before
        it can be handled by this method.
        """
        return cls(
            incoming_roads=spec['incoming_roads'],
            outgoing_roads=spec['outgoing_roads'],
            connectivity=spec['connectivity'],
            manager_type=spec['manager_type'],
            manager_spec=spec['manager_spec'],
            speed_limit=spec['speed_limit']
        )

    # Begin simulation cycle methods

    def get_new_speeds(self) -> Dict[Vehicle, SpeedUpdate]:

        new_speeds: List[Dict[Vehicle, SpeedUpdate]] = []

        for lane in self.lanes:
            new_speeds.append(lane.get_new_speeds())

        return dict(update for lane_update in new_speeds
                    for update in lane_update.items())

    def step_vehicles(self) -> None:
        """Progress vehicles currently in intersection.

        Have lanes progress their vehicles forward. If a vehicle in a lane
        exits the intersection in this step, pass it to the downstream road's
        vehicle transfer object and tell the tiling that this vehicle's
        reservation is complete and can be dropped.
        """

        for lane in self.lanes:
            transfers: List[VehicleTransfer] = lane.step_vehicles()
            for transfer in transfers:
                self.outgoing_road_by_lane_coord[transfer.pos
                                                 ].transfer_vehicle(transfer)
                if transfer.section is VehicleSection.REAR:
                    self.manager.finish_exiting(transfer.vehicle)

    def process_transfers(self) -> None:
        """Incorporate new vehicles onto this intersection."""
        while len(self.entering_vehicle_buffer) > 0:
            transfer = self.entering_vehicle_buffer.pop(0)
            lane: IntersectionLane
            if transfer.vehicle.has_reservation:
                if transfer.section == VehicleSection.FRONT:
                    # Tell the manager this reservation has started and get the
                    # lane it should be on.
                    lane = self.manager.start_reservation(transfer.vehicle)
                else:
                    # Reservation already started, so just get the lane it
                    # should be on.
                    lane = self.manager.get_lane(transfer.vehicle)
            elif transfer.vehicle.permission_to_enter_intersection:
                # Retrieve the vehicle's desired IntersectionLane by using
                # its next movement.
                # TODO: (low) Consider checking that this lane is open for
                #       unscheduled movements at this timestep. This
                #       shouldn't be necessary since this should have been
                #       cleared when the vehicle was given permission to
                #       enter.
                # TODO: (speed) cache value of front section for use by center
                #       and rear.
                lane = self.lanes_by_endpoints[(
                    transfer.pos,
                    transfer.vehicle.next_movements(transfer.pos)[0]
                )]
            else:
                raise RuntimeError("Received a vehicle that does not have "
                                   "permission to enter the intersection.")
            lane.enter_vehicle_section(transfer)
        super().process_transfers()  # just makes sure the list is empty after

    def update_schedule(self) -> None:
        """Have the manager update reservations and poll for new requests."""
        self.manager.update_schedule()

    # Begin helper methods

    def stopping_distance_to_last_vehicle(self, start_coord: Coord
                                          ) -> Optional[float]:
        """Return the closest vehicle on a lane that starts at this coord."""
        sds: List[float] = []
        for lane in self.lanes_by_start[start_coord]:
            sd = lane.stopping_distance_to_last_vehicle()
            if sd is not None:
                sds.append(sd)

        # Return the smallest stopping distance among all the IntersectionLanes
        # starting at this Coord. If none of them have a stopping distance
        # (i.e., none of them have vehicles on them), return None.
        return min(sds) if len(sds) > 0 else None
