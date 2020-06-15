"""
The simulators module contains the classes that create and run a new instance
of the AIM simulator, including visualizations if enabled.
"""

from __future__ import annotations
from typing import Iterable, Tuple, Dict, Optional, Set, Any, List

from pandas import DataFrame
from matplotlib import pyplot as plt

import aimsim.shared as SHARED
from ..util import Coord
from ..pathfinder import Pathfinder
from ..archetypes import Configurable, Facility, Upstream, Downstream
from ..intersections import Intersection
from ..roads import Road
from ..endpoints import VehicleSpawner, VehicleRemover
from ..vehicles import Vehicle


class Simulator:
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

    def __init__(self,
                 road_specs: Iterable[Dict[str, Any]],
                 intersection_specs: Iterable[Dict[str, Any]],
                 spawner_specs: Iterable[Dict[str, Any]],
                 remover_specs: Iterable[Dict[str, Any]],
                 lane_destination_pairs: Optional[Dict[Tuple[Coord, int],
                                                       List[Coord]]] = None,
                 config_filename: str = './config.ini',
                 display: bool = False) -> None:
        """Create all roads, intersections, and suppport structures.

        Parameters
            road_specs: Iterable[Dict[str, Any]]
            intersection_specs: Iterable[Dict[str, Any]]
            spawner_specs: Iterable[Dict[str, Any]]
            remover_specs: Iterable[Dict[str, Any]]
                These dicts, which can contain nested dicts, provide the
                initialization parameters for all customizable objects in the
                AIM simulation.
            lane_destination_pairs:
                Optional[Dict[Tuple[Coord, int], List[Coord]]] = None
                If provided, overrides default Pathfinder behavior by
                hardcoding paths through the intersection network, obviating
                the need for shortest path calculations.
            config_filename: str = './config.ini'
                The location of the config file.
            display: bool = False
                Whether to visualize the progress of the simulation.
        """

        # 0. Read in the config file.
        SHARED.read(config_filename)

        # 1. Create the Upstream and Downstream objects
        #   a. Create the roads and organize by intersection and road to be
        #      created.
        self.roads: Dict[int, Road] = {}
        for_intersection_upstream: Dict[int, Set[int]] = {}
        for_intersection_downstream: Dict[int, Set[int]] = {}
        for_spawner: Dict[int, int] = {}
        for_remover: Dict[int, int] = {}
        for spec in road_specs:
            rid: int = spec['id']
            self.roads[rid] = Road.from_spec(spec)

            # Record the road's expected upstream object to check for
            # correctness later.
            upstream_id: int = spec['upstream_id']
            if spec['upstream_is_spawner']:
                for_spawner[upstream_id] = rid
            else:
                if upstream_id not in for_intersection_upstream:
                    for_intersection_upstream[upstream_id] = {rid}
                else:
                    for_intersection_upstream[upstream_id].add(rid)

            # Prepare to attach to Downstream object
            downstream_id: int = spec['downstream_id']
            if spec['downstream_is_remover']:
                for_remover[downstream_id] = rid
            else:
                if downstream_id not in for_intersection_downstream:
                    for_intersection_downstream[downstream_id] = {rid}
                else:
                    for_intersection_downstream[downstream_id].add(rid)

        #   b. Create intersections, spawners, and removers given the roads.
        #      Connect roads to their intersections, spawners, and removers.
        self.intersections: Dict[int, Intersection] = {}
        for spec in intersection_specs:

            # Cache intersection ID
            iid: int = spec['id']

            # Check that there are roads that lead to this intersection from
            # both upstream and downstream
            if (iid not in for_intersection_upstream
                    or iid not in for_intersection_downstream):
                raise ValueError(
                    f"Spec has intersection {iid} but no road does.")

            # Put the upstream and downstream roads in this spec
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

            # On road's side, track which intersections we've confirmed exist
            del for_intersection_upstream[iid]
            del for_intersection_downstream[iid]

        # Check if every road's intersection has been created
        if ((len(for_intersection_upstream) > 0) or
                (len(for_intersection_downstream) > 0)):
            raise ValueError(
                f"Roads have at least one intersection that the spec doesn't.")

        self.spawners: Dict[int, VehicleSpawner] = {}
        for spec in spawner_specs:

            # Cache spawner ID
            sid: int = spec['id']

            # Check that a road leaves this spawner
            if sid not in for_spawner:
                raise ValueError(f"Spec has spawner {sid} but no road does.")

            # Check that the road the spawner thinks is connected to agrees
            # that the spawner is connected to it
            rid = spec['road_id']
            if rid != for_spawner[sid]:
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
        if len(for_spawner) > 0:
            raise ValueError(
                f"Roads have at least one spawner that the spec doesn't.")

        self.removers: Dict[int, VehicleRemover] = {}
        for spec in remover_specs:

            # Cache remover ID
            vid: int = spec['id']

            # Check that a road enters this remover
            if vid not in for_remover:
                raise ValueError(f"Spec has remover {vid} but no road does.")

            # Check that the road the remover thinks is connected to agrees
            # that the remover is connected to it
            rid = spec['road_id']
            if rid != for_remover[vid]:
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
        if len(for_remover) > 0:
            raise ValueError(
                f"Roads have at least one remover that the spec doesn't.")

        # 2. Generate a Pathfinder from the specs and share it across modules
        SHARED.pathfinder = Pathfinder(self.roads.values(),
                                       self.intersections.values(),
                                       lane_destination_pairs)

        # 3. Group them into common sets so it's neater to loop through
        self.facilities: Iterable[Facility] = list(
            self.intersections.values()
        ) + list(self.roads.values())

        self.upstreams: Iterable[Upstream] = list(
            self.spawners.values()
        ) + list(self.intersections.values()) + list(self.roads.values())

        self.downstreams: Iterable[Downstream] = list(
            self.intersections.values()
        ) + list(self.roads.values()) + list(self.removers.values())

        # 4. Initialize an empty set of vehicles
        self.vehicles: Set[Vehicle] = set()

        # 5. If the visualize flag is enabled, draw the basemap image of
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

            # TODO: (visualize) Old code, revise. If revise, revamp to draw
            #       lanes better and to include vehicles.
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

        # TODO: (parallel) The for loops in each step can be parallelized. Each
        #       pass can be implemented as a parallel worker. It'd probably be
        #       tricky, but the algorithm is designed such that at most only
        #       one pass of the double for loop changes any one data structure.

        # 1. Have every facility (roads, intersections) calculate the speed and
        #    acceleration of its responsible vehicles. If a vehicle is in any
        #    part inside an intersection, the intersection calculates it,
        #    otherwise, the road does. Can be done in parallel. Once
        #    calculated, update vehicle speed and acceleration serially.
        new_speeds = []
        for f in self.facilities:
            new_speeds.append(f.update_speeds())
        new_speed = dict(update for f_update in new_speeds
                         for update in f_update.items())
        for vehicle in self.vehicles:
            update = new_speed[vehicle]
            vehicle.v = update.v
            vehicle.a = update.a

        # 2. Have upstream objects (vehicle spawners, roads, and intersections)
        #    update vehicle absolute (Coord) and relative positions (in-lane
        #    progression) based on speeds calculated during last step. If the
        #    front, center, or rear of a vehicle exits a lane, place it in the
        #    buffer of the downstream object. For spawners, this decides
        #    whether to create a new vehicle and if so places it in the
        #    downstream object.
        for u in self.upstreams:
            new_vehicle = u.step_vehicles()
            if new_vehicle is not None:
                self.vehicles.add(new_vehicle)

        # 3. Have every downstream object (roads, intersections, and vehicle
        #    removers) resolve all vehicle transfers. This finishes updates to
        #    absolute and relative positions using speeds calculated in 1.
        for d in self.downstreams:
            exiting_vehicles: Optional[Iterable[Vehicle]
                                       ] = d.process_transfers()
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
        for f in self.facilities:
            f.handle_logic()

        # 5. Update shared time step and (TODO: (low)) shortest path values
        SHARED.t += 1
        SHARED.pathfinder.update(None)

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
        raise NotImplementedError("TODO")

        # fill those data structures by ingesting road geometry


class LargeSimulator(Simulator):

    def __init__(self,
                 road_strs: Iterable[str],
                 intersection_strs: Iterable[str],
                 spawner_strs: Iterable[str],
                 remover_strs: Iterable[str],
                 config_filename: str = './config.ini',
                 display: bool = False) -> None:
        """Create a Simulator using strings for every road object."""

        # Process the specs for every Upstream and Downstream object
        road_specs: Iterable[Dict[str, Any]] = [
            Road.spec_from_str(r) for r in road_strs]
        intersection_specs: Iterable[Dict[str, Any]] = [
            Intersection.spec_from_str(i) for i in intersection_strs]
        spawner_specs: Iterable[Dict[str, Any]] = [
            VehicleSpawner.spec_from_str(s) for s in spawner_strs]
        remover_specs: Iterable[Dict[str, Any]] = [
            VehicleRemover.spec_from_str(r) for r in remover_strs]

        # Hand it off to the main Simulator init
        super().__init__(
            road_specs=road_specs,
            intersection_specs=intersection_specs,
            spawner_specs=spawner_specs,
            remover_specs=remover_specs,
            config_filename=config_filename,
            display=display
        )
