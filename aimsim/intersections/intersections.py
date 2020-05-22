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
from typing import Iterable, Type, Dict, Union, Set, Any, List

from pandas import DataFrame

import aimsim.shared as SHARED
from ..archetypes import Configurable, Facility, Upstream, Downstream
from ..util import Coord, VehicleTransfer, SpeedUpdate
from ..trajectories import Trajectory, BezierTrajectory
from ..lanes import IntersectionLane
from ..roads import Road
from ..endpoints import VehicleRemover
from ..vehicles import Vehicle
from .managers import IntersectionManager, FCFSManager


class Intersection(Configurable, Facility, Upstream, Downstream):

    def __init__(self,
                 upstream_roads: Iterable[Road],
                 downstream_roads: Iterable[Road],
                 connectivity: Iterable[Iterable],
                 manager_type: IntersectionManager,
                 manager_spec: Dict[str, Any],
                 v_max: int = SHARED.speed_limit
                 ) -> None:
        """Create a new intersection.

        Keyword arguments
            connectivity: Iterable[Iterable] # TODO: finalize structure
                Describes which lanes connect to each other.
            manager_spec: Dict[str, Any]
                Specifications to create the manager with.
            v_max: int = SHARED.speed_limit
                The speed limit.
        """

        # Index the upstream and downstream roads by their lanes' Coords
        self.road_by_coord: Dict[Coord, Road] = {}
        self.upstream_road_by_coord: Dict[Coord, Road] = {}
        self.downstream_road_by_coord: Dict[Coord, Road] = {}
        for r in upstream_roads:
            for coord in r.lanes_by_end:
                self.road_by_coord[coord] = r
                self.upstream_road_by_coord[coord] = r
        for r in downstream_roads:
            for coord in r.lanes_by_start:
                self.road_by_coord[coord] = r
                self.downstream_road_by_coord[coord] = r

        # Given the upstream and downstream roads and connectivity matrix,
        # connect the road endpoints together by creating new lanes, curving
        # if necessary based on the difference in heading between the endpoints
        # self.lanes = blah
        raise NotImplementedError("TODO")

        # Index the IntersectionLanes by their Coord
        self.lane_coords: Set[Coord] = {**{
            l.start_coord: l for l in self.lanes
        }, **{
            l.end_coord: l for l in self.lanes
        }}

        # Finish the spec for the manager by providing the roads and lanes
        manager_spec['upstream_road_by_coord'] = self.upstream_road_by_coord
        manager_spec['downstream_road_by_coord'
                     ] = self.downstream_road_by_coord
        manager_spec['lanes'] = self.lanes  # TODO: should this be lane_coords?

        # Create the manager
        self.manager: IntersectionManager = manager_type.from_spec(
            manager_spec)

        # TODO: old code. rewrite or replace.
        # for _, row in lanes_df.iterrows():
        #     traj = trajectory(
        #         row['TAIL_X'],
        #         row['TAIL_Y'],
        #         row['MID_X'],
        #         row['MID_Y'],
        #         row['HEAD_X'],
        #         row['HEAD_Y'])
        #     if row['IO'] == 'I':
        #         incoming_lanes[row['ID']] = RoadLane(traj, 0)
        #     elif row['IO'] == 'O':
        #         outgoing_lanes[row['ID']] = RoadLane(traj, 0)
        #     else:
        #         raise ValueError("Unexpected lane type.")

        # for _, row in intersection_traj_df.iterrows():
        #     tail_traj = incoming_lanes[row['TAIL_ID']].trajectory
        #     head_traj = outgoing_lanes[row['HEAD_ID']].trajectory
        #     traj = Bezier(
        #         tail_traj.p2[0],
        #         tail_traj.p2[1],
        #         row['MID_X'],
        #         row['MID_Y'],
        #         head_traj.p0[0],
        #         head_traj.p0[1])
        #     # TODO: determine the proper length of the lane
        #     intersection_lane = IntersectionLane(traj, 0)

        #     self.intersection_lanes.add(intersection_lane)
        #     self.add_incoming_lane(
        #         incoming_lanes[row['TAIL_ID']], intersection_lane)
        #     self.add_outgoing_lane(
        #         outgoing_lanes[row['HEAD_ID']], intersection_lane)

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a intersection spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: interpret the string into the spec dict
        raise NotImplementedError("TODO")

        # TODO: enforce provision of separate manager_type and manager_config
        #       fields in intersection spec string

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
        # TODO: make sure that road objects are provided
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

    def step(self) -> None:
        """Progress vehicles currently in intersection."""

        # TODO: error check that setup has finished
        # # error checking
        # for road_coord in self.upstream_road_coords:
        #     if road_coord not in self.upstream:
        #        raise ValueError("Missing promised upstream road in connected"
        #                          "roads. Check if the intersection and road"
        #                          "coords in the spec file match."
        #                         )

        # Have lanes progress their vehicles forward.
        # If a vehicle in a lane exits the intersection in this step, pass it
        # to the downstream road's vehicle transfer object and tell the tiling
        # that this vehicle's reservation is complete and can be dropped.

        # Manager does NOTHING in this cycle step.

        for lane in self.lanes:
            leaving = lane.step()
            if leaving is not None:
                self.downstream.transfer_vehicle(leaving)
                self.tiling.reservation_complete(leaving.vehicle)

    def process_transfers(self) -> None:
        """Incorporate new vehicles"""
        # TODO: write logic to process transfers using self.enter_vehicle()
        while len(self.entering_vehicle_buffer) > 0:
            transfer = self.entering_vehicle_buffer.pop()
            # TODO: consider checking if a lane gets more than one vehicle
            # added in a cycle. if so, raise TooManyProgressionsError
            # enter_vehicle(transfer)
            # TODO: tell tiling that this reservation is now active
            raise NotImplementedError("TODO")
        super().process_transfers()  # just makes sure the list is empty after

    def handle_logic(self) -> None:
        """Have the manager update reservations and poll for new requests."""
        self.manager.handle_logic()
