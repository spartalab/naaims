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

import itertools
from typing import Iterable, Type, Dict, Union, Set

from pandas import DataFrame

from ..archetypes import Configurable, Facility, Upstream, Downstream
from ..util import Coord, VehicleTransfer
from ..trajectories import Trajectory, Bezier
from ..lanes import IntersectionLane
from ..roads import Road
from ..endpoints import VehicleRemover
from .managers import IntersectionManager, FCFSManager
from .tilings import Tiling, ArcTiling


class Intersection(Configurable, Facility, Upstream, Downstream):

    def __init__(self,
                 intersection_traj_df: DataFrame,
                 lanes_df: DataFrame,
                 trajectory: Type[Trajectory] = Bezier,
                 manager: Type[IntersectionManager] = FCFSManager,
                 tiling: Type[Tiling] = ArcTiling,
                 v_max: int = 70  # speed limit, in m/s
                 ) -> None:
        """Create a new intersection.

        Keyword arguments:
        TODO
        """

        # TODO: roads are set up as a trajectory with offsets for lanes, but
        #       intersections need lanes to be given explicitly. how to handle?

        # given init, build all intersection lanes
        # self.lanes : Iterable[IntersectionLane] = {some logic}
        raise NotImplementedError("TODO")

        # collect IntersectionLane start coords into a set for stitching later
        self.upstream_road_coords: Set[Coord] = {
            l.start_coord for l in self.lanes
        }

        # do the same for the downstream coords
        self.downstream_road_coords: Set[Coord] = {
            l.end_coord for l in self.lanes
        }

        # initialize set of outgoing roads
        self.upstream: Dict[Coord, Road] = {}
        self.downstream: Dict[Coord, Road] = {}

        # pass the entire setup to the Tiling since it needs to set up with it
        self.tiling = tiling(self.lanes)

        # (wait on initializing the manager until we have the incoming roads)
        self.manager_type = manager

        # TODO: old code. rewrite or replace.
        for _, row in lanes_df.iterrows():
            traj = trajectory(
                row['TAIL_X'],
                row['TAIL_Y'],
                row['MID_X'],
                row['MID_Y'],
                row['HEAD_X'],
                row['HEAD_Y'])
            if row['IO'] == 'I':
                incoming_lanes[row['ID']] = RoadLane(traj, 0)
            elif row['IO'] == 'O':
                outgoing_lanes[row['ID']] = RoadLane(traj, 0)
            else:
                raise ValueError("Unexpected lane type.")

        for _, row in intersection_traj_df.iterrows():
            tail_traj = incoming_lanes[row['TAIL_ID']].trajectory
            head_traj = outgoing_lanes[row['HEAD_ID']].trajectory
            traj = Bezier(
                tail_traj.p2[0],
                tail_traj.p2[1],
                row['MID_X'],
                row['MID_Y'],
                head_traj.p0[0],
                head_traj.p0[1])
            # TODO: determine the proper length of the lane
            intersection_lane = IntersectionLane(traj, 0)

            self.intersection_lanes.add(intersection_lane)
            self.add_incoming_lane(
                incoming_lanes[row['TAIL_ID']], intersection_lane)
            self.add_outgoing_lane(
                outgoing_lanes[row['HEAD_ID']], intersection_lane)

    def finish_setup(self, incoming_roads: Iterable[Road],
                     outgoing_roads: Iterable[Road]) -> None:
        """Given the """
        # TODO: how will setup finish with the intersection? once all the Road
        #       objects are finalized, we need to tie them to the coords and
        #       then pass them to the manager.

        # tie incoming and outgoing roads to this object by coords
        # for road in
        # self.upstream[] =
        # self.downstream[] =

        # finish by instantiating the manager
        self.manager = self.manager_type(tiling=self.tiling,
                                         incoming_roads=incoming_roads)

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

        # consider checking for collisions here if we have stochasic movement
        # otherwise collisions _should_ not occur if implemented correctly
        # vehicles = []
        # for lane in self.lanes:
        #     vehicles += [vp.vehicles for vp in lane.vehicles]
        # # draw every vehicle to check for collisions?
        raise NotImplementedError("TODO")

        # have manager and tiling compare the current positions and velocities
        # of all vehicles in intersection. use the difference to update
        # reservations to reflect resolved stochasticities (probably a TODO
        # future feature, this will likely only be used by a stochastic
        # manager)
        self.tiling.update_active_reservations()

        # Have tiling peel off the layer of reservations that correspond to
        # this passing timestep (i.e., zero the tiling at this step)
        self.tiling.step()

        # have manager look for new requests and at those in the queue to see
        # which ones to accept
        self.manager.handle_logic()
