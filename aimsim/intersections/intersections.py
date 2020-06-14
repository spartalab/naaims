"""
`intersection` implements a road intersection with a specific priority
control policy, creating all of its associated incoming and outgoing lanes.

The intersection includes incoming roads up to some distance specified in the
constructor. Vehicles on these approach roads are barred from changing lanes
as a necessary prerequisite for the AIM scheduling method.


The intersection handles creation and management of the intersection lanes and
vehicle physical I/O.

The manager handles how vehicles progress along the lanes and if vehicles get
to come in or not.

The tiling handles how the manager checks for upcoming conflicts.
"""

from __future__ import annotations
import itertools
from typing import Iterable, Type, Dict, Any, List, Tuple, Optional

from pandas import DataFrame

import aimsim.shared as SHARED
from ..archetypes import Configurable, Facility, Upstream, Downstream
from ..util import Coord, VehicleTransfer, SpeedUpdate, VehicleSection
from ..trajectories import Trajectory, BezierTrajectory
from ..lanes import IntersectionLane, RoadLane
from ..roads import Road
from ..endpoints import VehicleRemover
from ..vehicles import Vehicle
from .managers import IntersectionManager, FCFSManager


class Intersection(Configurable, Facility, Upstream, Downstream):

    def __init__(self,
                 upstream_roads: Iterable[Road],
                 downstream_roads: Iterable[Road],
                 connectivity: Iterable[Tuple[Road, Road, bool]],
                 manager_type: IntersectionManager,
                 manager_spec: Dict[str, Any],
                 v_max: int = SHARED.speed_limit
                 ) -> None:
        """Create a new intersection.

        Keyword arguments
            connectivity: Iterable[Tuple[Road, Road, bool]]
                Describes which lanes connect to each other, used to create
                IntersectionLanes.
                The pattern is a list of 3-tuples which correspond to:
                    0. The incoming Road
                    1. The outgoing Road
                    2. If true, the two roads will be fully connected as best
                       as possible. If the number of lanes don't match, defer
                       to the Road with fewer lanes, attaching them to the
                       corresponding lane in the other Road that results in the
                       shortest distance and ensures that no IntersectionLane
                       created starts or ends at the same node.
                       If false, only one Intersection will be created between
                       the closest pair of farthest left or right lanes on both
                       trajectories. (This approximates a restricted turning
                       movement.)
            manager_spec: Dict[str, Any]
                Specifications to create the manager with.
            v_max: int = SHARED.speed_limit
                The speed limit.
        """

        # Index the upstream and downstream roads by their lanes' Coords
        self.upstream_road_lane_by_coord: Dict[Coord, RoadLane] = {}
        self.downstream_road_lane_by_coord: Dict[Coord, RoadLane] = {}
        self.downstream_road_by_coord: Dict[Coord, Road] = {}
        for r in upstream_roads:
            for coord, lane in r.lanes_by_end.items():
                self.upstream_road_lane_by_coord[coord] = lane
        for r in downstream_roads:
            for coord, lane in r.lanes_by_start.items():
                self.downstream_road_lane_by_coord[coord] = lane
                self.downstream_road_by_coord[coord] = r

        # Given the upstream and downstream roads and connectivity matrix,
        # connect the road endpoints together by creating new lanes, curving
        # if necessary based on the difference in heading between the endpoints
        self.lanes: List[IntersectionLane]
        raise NotImplementedError("TODO")

        # Index the IntersectionLanes by their Coord
        self.lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                      IntersectionLane] = {
            (l.start_coord, l.end_coord): l for l in self.lanes
        }
        self.lanes_by_start: Dict[Coord, List[IntersectionLane]] = {}
        for lane in self.lanes:
            coord = lane.trajectory.start_coord
            if coord not in self.lanes_by_start:
                self.lanes_by_start[coord] = [lane]
            else:
                self.lanes_by_start[coord].append(lane)

        # Finish the spec for the manager by providing the roads and lanes
        manager_spec['upstream_road_lane_by_coord'
                     ] = self.upstream_road_lane_by_coord
        manager_spec['downstream_road_lane_by_coord'
                     ] = self.downstream_road_lane_by_coord
        manager_spec['lanes'] = self.lanes
        manager_spec['lanes_by_endpoints'] = self.lanes_by_endpoints

        # Create the manager
        self.manager: IntersectionManager = manager_type.from_spec(
            manager_spec)

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a intersection spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: interpret the string into the spec dict
        raise NotImplementedError("TODO")

        # TODO: enforce provision of separate manager_type and manager_config
        #       fields in intersection spec string

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
            upstream_roads=spec['upstream_roads'],
            downstream_roads=spec['downstream_roads'],
            connectivity=spec['connectivity'],
            manager_type=spec['manager_type'],
            manager_spec=spec['manager_spec'],
            v_max=spec['v_max']
        )

    # Begin simulation cycle methods

    def update_speeds(self) -> Dict[Vehicle, SpeedUpdate]:

        new_speeds: List[Dict[Vehicle, SpeedUpdate]] = []

        for lane in self.lanes:
            new_speeds.append(lane.update_speeds())

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
            transfers: Iterable[VehicleTransfer] = lane.step_vehicles()
            for transfer in transfers:
                self.downstream_road_by_coord[transfer.pos].transfer_vehicle(
                    transfer)
                if transfer.section is VehicleSection.REAR:
                    self.manager.finish_exiting(transfer.vehicle)

    def process_transfers(self) -> None:
        """Incorporate new vehicles onto this intersection."""
        while len(self.entering_vehicle_buffer) > 0:
            transfer = self.entering_vehicle_buffer.pop()
            lane: IntersectionLane
            if transfer.vehicle.has_reservation:
                # Tell the manager this reservation has started and get the
                # lane it should be on.
                lane = self.manager.start_reservation(transfer.vehicle)
            elif transfer.vehicle.permission_to_enter_intersection:
                # Retrieve the vehicle's desired IntersectionLane by using its
                # next movement.
                # TODO: (low) Consider checking that this lane is open for
                #       unscheduled movements at this timestep. This shouldn't
                #       be necessary since this should have been cleared when
                #       the vehicle was given permission to enter.
                lane = self.lanes_by_endpoints[(
                    transfer.pos,
                    transfer.vehicle.next_movements(transfer.pos)[0]
                )]
            else:
                raise RuntimeError("Received a vehicle that does not have "
                                   "permission to enter the intersection.")
            lane.enter_vehicle_section(transfer)
        super().process_transfers()  # just makes sure the list is empty after

    def handle_logic(self) -> None:
        """Have the manager update reservations and poll for new requests."""
        self.manager.handle_logic()

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
