"""
`simulator` implements the class that runs a new instance of the AIM simulator,
including visualizations if enabled.
"""

from abc import ABC, abstractmethod
from typing import Iterable, Tuple, Dict, Optional, Set, Any
from __future__ import annotations

from pandas import DataFrame
from matplotlib import pyplot as plt

import aimsim.settings as SETTINGS
from ..util import Coord
from ..archetypes import Configurable, Facility, Upstream, Downstream
from ..intersections import Intersection
from ..roads import Road
from ..endpoints import VehicleSpawner, VehicleRemover
from ..vehicles import Vehicle


class Simulator(ABC):
    """
    The Simulator class gives fully comprehensive access to every tool
    available in this AIM implementation.

    As intended, subclasses of Simulator should only serve to streamline the
    call to the Simulator constructor by reading text files and converting them
    into datatypes the parent Simulator init can understand, as well as by
    limiting users' options when creating new AIM simulations (e.g., limiting
    the Simulator to only a single FCFS intersection).

    To this end, inits of subclasses should convert text files into the
    proprietary datatypes Simulator needs to construct the network.
    """

    @abstractmethod
    def __init__(self,
                 road_strs: Iterable[str],
                 intersection_strs: Iterable[str],
                 spawner_strs: Iterable[str],
                 remover_strs: Iterable[str],
                 config_filename: str = './config.ini',
                 display: bool = False) -> None:
        """Should create all roads, intersections, and suppport structures.

        This abstract init that all Simulators should call serves only to
        initialize data structures for lookup, calls, and visualization.
        """

        # 1. Read in the config file.
        SETTINGS.read(config_filename)

        # 2. Process the specs for every Upstream and Downstream object
        road_specs: Iterable[Dict[str, Any]] = [
            Road.spec_from_str(r) for r in road_strs]
        intersection_specs: Iterable[Dict[str, Any]] = [
            Intersection.spec_from_str(i) for i in intersection_strs]
        spawner_specs: Iterable[Dict[str, Any]] = [
            VehicleSpawner.spec_from_str(s) for s in spawner_strs]
        remover_specs: Iterable[Dict[str, Any]] = [
            VehicleRemover.spec_from_str(r) for r in remover_strs]

        # 3. Create the Upstream and Downstream objects
        #   a. Create the roads and organize by intersection and road to be
        #      created.
        self.roads: Dict[int, Road] = {}
        for_intersection_upstream: Dict[int, Set[int]] = {}
        for_intersection_downstream: Dict[int, Set[int]] = {}
        for_spawner: Dict[int, int] = {}
        for_remover: Dict[int, int] = {}
        for spec in road_specs:
            self.roads[spec['id']] = Road.from_spec(spec)
            # TODO: finish this pseudocode, figure out how info is stored
            #       and if spec['upstream/downstream'] needs processing
            # if spec['upstream'] is an intersection:
            #     for_intersection_upstream.setdefault([spec['upstream'], []
            #       ]) = self.roads[spec['id']]
            # else: upstream better be a spawner, throw if not
            #       TODO: also throw if it has a spawner that already exists
            # if spec['downstream'] is an intersection:
            #     for_intersection_downstreamsetdefault([spec['downstream'], []
            #       ]) = self.roads[spec['id']]
            raise NotImplementedError("TODO")

        #   b. Create intersections, spawners, and removers given the
        #      roads. Connect the intersection, spawner, and removers to the
        #      relevant roads.
        #   TODO: make sure actual spec fields match what's expected here
        self.intersections: Dict[int, Intersection] = {}
        for spec in intersection_specs:

            # cache intersection ID
            iid = spec['id']

            # Check that there are roads that lead to this intersection from
            # both upstream and downstream
            if (iid not in for_intersection_upstream
                    or iid not in for_intersection_downstream):
                raise ValueError(
                    f"Spec has intersection {iid} but no road does.")

            # Put the upstream and downstream roads in this spec
            # TODO: error check that every road the spec expects exists
            spec['upstream_roads'] = {}
            for rid in spec['upstream_road_ids']:
                spec['upstream_roads'][rid] = self.roads[rid]
            spec['downstream_roads'] = {}
            for rid in spec['downstream_road_ids']:
                spec['downstream_roads'][rid] = self.roads[rid]

            # Create the intersection
            self.intersections[iid] = Intersection.from_spec(spec)

            # Attach the intersection to the up and downstream roads
            while len(for_intersection_downstream[iid]) > 0:
                rid = for_intersection_downstream[iid].pop()
                self.roads[rid].connect_downstream(self.intersections[iid])
            while len(for_intersection_upstream[iid]) > 0:
                rid = for_intersection_upstream[iid].pop()
                self.roads[rid].connect_upstream(self.intersections[iid])

            # On the road side, track which intersections we've confirmed exist
            del for_intersection_upstream[iid]
            del for_intersection_downstream[iid]

        # Check if every road's intersection has been created
        if (bool(for_intersection_upstream)
                or bool(for_intersection_downstream)):
            raise ValueError(
                f"Roads have at least one intersection that the spec doesn't.")

        self.spawners: Dict[int, VehicleSpawner] = {}
        for spec in spawner_specs:

            # Cache spawner ID
            sid = spec['id']

            # Check that a road leaves this spawner
            if sid not in for_spawner:
                raise ValueError(f"Spec has spawner {sid} but no road does.")

            # Check that the road the spawner thinks is connected to agrees
            # that the spawner is connected to it
            rid = spec['road_id']
            try:
                assert rid == for_spawner[sid]
            except AssertionError:
                raise ValueError(f"Spawner {sid} thinks it's connected to road"
                                 f" {rid} but road {for_spawner[sid]} thinks"
                                 " it's the one.")

            # Put the road in the spec
            spec['road'] = self.roads[rid]

            # Create the spawner
            self.spawners[sid] = VehicleSpawner.from_spec(spec)

            # Attach the spawner to the road
            self.roads[rid].connect_upstream(self.spawners[sid])

            # On the road side, track that we've created this spawner
            del for_spawner[rid]

        # Check if every road's spawner (if it has one) has been created
        if bool(for_spawner):
            raise ValueError(
                f"Roads have at least one spawner that the spec doesn't.")

        self.removers: Dict[int, VehicleRemover] = {}
        for spec in remover_specs:

            # Cache remover ID
            vid = spec['id']

            # Check that a road enters this remover
            if vid not in for_remover:
                raise ValueError(f"Spec has remover {vid} but no road does.")

            # Check that the road the remover thinks is connected to agrees
            # that the remover is connected to it
            rid = spec['road_id']
            try:
                assert rid == for_remover[vid]
            except AssertionError:
                raise ValueError(f"Remover {vid} thinks it's connected to road"
                                 f" {rid} but road {for_remover[vid]} thinks"
                                 " it's the one.")

            # Put the road in the spec
            spec['road'] = self.roads[rid]

            # Create the remover
            self.removers[vid] = VehicleRemover.from_spec(spec)

            # Attach the remover to the road
            self.roads[rid].connect_downstream(self.removers[vid])

            # On the road side, track that we've created this remover
            del for_remover[rid]

        # Check if every road's remover (if it has one) has been created
        if bool(for_remover):
            raise ValueError(
                f"Roads have at least one remover that the spec doesn't.")

        # 4. Group them into common sets so it's neater to loop through
        self.facilities: Tuple[Dict[int, Facility], ...] = (
            self.intersections,
            self.roads
        )
        self.upstreams: Tuple[Dict[int, Upstream], ...] = (
            self.spawners,
            self.intersections,
            self.roads
        )
        self.downstreams: Tuple[Dict[int, Downstream], ...] = (
            self.intersections,
            self.roads,
            self.removers
        )

        # 5. Initialize an empty list of vehicles
        # self.vehicles: Dict[int, Vehicle] = {}
        self.vehicles: Set[Vehicle] = set()

        # 6. If the visualize flag is enabled, draw the basemap image of
        #    roads and intersections for use later.
        if display:
            # 6a. loop through all roads and visualize their length, width,
            #     and lane markings, etc. using their trajectory.
            for road in self.roads:
                raise NotImplementedError("TODO")
                # road.display()

            # loop through all intersections and visualize their area
            for intersection in self.intersections:
                raise NotImplementedError("TODO")
                # intersection.display()

            # Save the result of the road+intersection vis into a property
            # self.vis_basemap = whatever

            # TODO: old code, revise
            # TODO: revamp to draw lanes better and to include vehicles
            # lanes = list(self.intersection.incomingLanes.keys())
            # for ls in self.intersection.outgoingLanes.values():
            #     for l in ls:
            #         lanes.append(l)
            # print(lanes)
            # plt.figure(figsize=(15, 16))
            # ax = plt.gca()
            # plt.xlim(-plot_lim, plot_lim)
            # plt.ylim(-plot_lim, plot_lim)
            # for i, r in enumerate(lanes):
            #     if r not in self.intersection.intersection_lanes:
            #         plt.arrow(r.trajectory.p0[0],
            #                   r.trajectory.p0[1],
            #                   r.trajectory.p2[0] - r.trajectory.p0[0],
            #                   r.trajectory.p2[1] - r.trajectory.p0[1],
            #                   head_width=1,
            #                   overhang=.5)

            # for i, r in enumerate(self.intersection.outgoingLanes.keys()):
            #     if r not in self.intersection.intersection_lanes:
            #         plt.arrow(r.trajectory.p0[0],
            #                   r.trajectory.p0[1],
            #                   r.trajectory.p2[0] - r.trajectory.p0[0],
            #                   r.trajectory.p2[1] - r.trajectory.p0[1],
            #                   head_width=1,
            #                   overhang=1,
            #                   color='r')

            # for traj in self.intersection.intersection_lanes:
            #     traj.trajectory.get_curve().plot(256, ax=ax)

            # # self.intersection.find_conflicts()
            # # print(len(self.intersection.conflicts))
            # x = [cp.point[0] for cp in self.intersection.conflicts]
            # y = [cp.point[1] for cp in self.intersection.conflicts]
            # ax.scatter(x, y)
            # if figname:
            #     plt.savefig(figname)
            # plt.show()

    def step(self) -> None:
        """Execute one simulation step."""

        # TODO: The double for loops in each step are parallelizable. Each pass
        #       can be implemented as a parallel worker. It'd probably be
        #       tricky, but the algorithm is designed such that at most only
        #       one pass of the double for loop changes any one data structure.

        # 1. Have every facility (roads, intersections) calculate the speed and
        #    acceleration of its responsible vehicles. If a vehicle is in any
        #    part inside an intersection, the intersection calculates it,
        #    otherwise, the road does.
        #    Currently, calculate_speeds() updates new_speed and
        #    new_acceleration properties inside each vehicle, but a parallel
        #    implementation might have each worker save and return changes for
        #    each worker to update serially between steps.
        for fs in self.facilities:
            for f in fs:
                f.update_speeds()
        # could possibly update speeds outside the loop here.

        # 2. Have upstream objects (vehicle spawners, roads, and intersections)
        #    update vehicle absolute (Coord) and relative positions (in-lane
        #    progression) based on speeds calculated during last step. If the
        #    front, center, or rear of a vehicle exits a lane, place it in the
        #    buffer of the downstream object. For spawners, this decides
        #    whether to create a new vehicle and if so places it in the
        #    downstream object.
        for us in self.upstreams:
            for u in us:
                new_vehicle = u.step()
                if new_vehicle is not None:
                    self.vehicles.add(new_vehicle)

        # 3. Have every downstream object (roads, intersections, and vehicle
        #    removers) resolve all vehicle transfers. This finishes updates to
        #    absolute and relative positions using speeds calculed in 1.
        for dss in self.downstreams:
            for ds in dss:
                exiting_vehicles: Optional[Iterable[Vehicle]
                                           ] = ds.process_transfers()
                # only vehicle_removers return vehicles, iff any get removed
                # in this cycle
                if exiting_vehicles is not None:
                    for vehicle in exiting_vehicles:
                        # log the vehicle
                        raise NotImplementedError("TODO")
                        # remove it from self.vehicles
                        self.vehicles.remove(vehicle)

        # 4. Have facility managers handle their special internal logic (e.g.,
        #    lane changes and reservations).
        for fs in self.facilities:
            for f in fs:
                f.handle_logic()

    def display(self) -> None:
        """Display the current position of all vehicles on the grid."""

        # TODO: consider having this return an image instead

        # since vehicles are the only things that change position on each step
        # loop through all vehicles and draw their position on the base image
        # at self.vis_basemap
        # for vehicle in self.Vehicles.values():
        raise NotImplementedError("TODO")


class SingleIntersectionSimulator(Simulator):

    def __init__(self,
                 intersection_traj_file: DataFrame,
                 lanes_file: DataFrame,
                 config_filename: str = './config.ini') -> None:
        """Create a new simulator for a single intersection."""

        # interpret the data files and convert them into classes
        # that the parent Simulator can understand

        # self.intersection = Intersection(
        #     intersection_traj_file, lanes_file)

        # call super's init first to initialize data structures
        super().__init__(config_filename)

        # fill those data structures by ingesting road geometry


class LargeSimulator(Simulator):
    # TODO: implement a simulator for multiple intersections
    pass
