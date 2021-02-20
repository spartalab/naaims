"""
The simulators module contains the classes that create and run a new instance
of the AIM simulator, including visualizations if enabled.
"""

from __future__ import annotations
from typing import List, Tuple, Dict, Optional, Set, Any
from matplotlib import pyplot as plt

import aimsim.shared as SHARED
from aimsim.util import Coord
from aimsim.pathfinder import Pathfinder
from aimsim.archetypes import Configurable, Facility, Upstream, Downstream
from aimsim.intersection import Intersection
from aimsim.road import Road
from aimsim.endpoints import VehicleSpawner, VehicleRemover
from aimsim.vehicles import Vehicle


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
                 road_specs: List[Dict[str, Any]],
                 intersection_specs: List[Dict[str, Any]],
                 spawner_specs: List[Dict[str, Any]],
                 remover_specs: List[Dict[str, Any]],
                 lane_destination_pairs: Optional[Dict[Tuple[Coord, int],
                                                       List[Coord]]] = None,
                 config_filename: str = './config.ini',
                 display: bool = False) -> None:
        """Create all roads, intersections, and suppport structures.

        Parameters
            road_specs: List[Dict[str, Any]]
            intersection_specs: List[Dict[str, Any]]
            spawner_specs: List[Dict[str, Any]]
            remover_specs: List[Dict[str, Any]]
                These lists, which can contain nested dicts, provide the
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
        SHARED.SETTINGS.read(config_filename)

        # 1. Create the Upstream and Downstream objects
        #   a. Create the roads and organize by intersection and road to be
        #      created.
        self.roads: Dict[int, Road] = {}
        upstream_intersection_rids: Dict[int, Set[int]] = {}
        downstream_intersection_rids: Dict[int, Set[int]] = {}
        spawner_rids: Dict[int, int] = {}
        remover_rids: Dict[int, int] = {}
        for spec in road_specs:
            rid: int = spec['id']
            self.roads[rid] = Road.from_spec(spec)

            # Record the road's expected upstream object to check for
            # correctness later.
            upstream_id: int = spec['upstream_id']
            if spec['upstream_is_spawner']:
                spawner_rids[upstream_id] = rid
            else:
                if upstream_id not in upstream_intersection_rids:
                    upstream_intersection_rids[upstream_id] = {rid}
                else:
                    upstream_intersection_rids[upstream_id].add(rid)

            # Prepare to attach to Downstream object
            downstream_id: int = spec['downstream_id']
            if spec['downstream_is_remover']:
                remover_rids[downstream_id] = rid
            else:
                if downstream_id not in downstream_intersection_rids:
                    downstream_intersection_rids[downstream_id] = {rid}
                else:
                    downstream_intersection_rids[downstream_id].add(rid)

        #   b. Create intersections, spawners, and removers given the roads.
        #      Connect roads to their intersections, spawners, and removers.
        self.intersections: Dict[int, Intersection] = {}
        for spec in intersection_specs:

            # Cache intersection ID
            iid: int = spec['id']

            # Check that there are roads that lead to this intersection from
            # both upstream and downstream
            if (iid not in upstream_intersection_rids
                    or iid not in downstream_intersection_rids):
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
            while len(downstream_intersection_rids[iid]) > 0:
                rid = downstream_intersection_rids[iid].pop()
                self.roads[rid].connect_downstream(self.intersections[iid])
            while len(upstream_intersection_rids[iid]) > 0:
                rid = upstream_intersection_rids[iid].pop()
                self.roads[rid].connect_upstream(self.intersections[iid])

            # On road's side, track which intersections we've confirmed exist
            del upstream_intersection_rids[iid]
            del downstream_intersection_rids[iid]

        # Check if every road's intersection has been created
        if ((len(upstream_intersection_rids) > 0) or
                (len(downstream_intersection_rids) > 0)):
            raise ValueError(
                f"Roads have at least one intersection that the spec doesn't.")

        self.spawners: Dict[int, VehicleSpawner] = {}
        for spec in spawner_specs:

            # Cache spawner ID
            sid: int = spec['id']

            # Check that a road leaves this spawner
            if sid not in spawner_rids:
                raise ValueError(f"Spec has spawner {sid} but no road does.")

            # Check that the road the spawner thinks is connected to agrees
            # that the spawner is connected to it
            rid = spec['road_id']
            if rid != spawner_rids[sid]:
                raise ValueError(f"Spawner {sid} thinks it's connected to road"
                                 f" {rid} but road {spawner_rids[sid]} thinks"
                                 " it's the one.")

            # Put the road in the spawner's spec as its downstream Facility
            spec['downstream'] = self.roads[rid]

            # Create the spawner
            self.spawners[sid] = VehicleSpawner.from_spec(spec)

            # Attach the spawner to the road
            self.roads[rid].connect_upstream(self.spawners[sid])

            # On the road side, track that we've created this spawner
            del spawner_rids[rid]

        # Check if every road's spawner (if it has one) has been created
        if len(spawner_rids) > 0:
            raise ValueError(
                f"Roads have at least one spawner that the spec doesn't.")

        self.removers: Dict[int, VehicleRemover] = {}
        for spec in remover_specs:

            # Cache remover ID
            vid: int = spec['id']

            # Check that a road enters this remover
            if vid not in remover_rids:
                raise ValueError(f"Spec has remover {vid} but no road does.")

            # Check that the road the remover thinks is connected to agrees
            # that the remover is connected to it
            rid = spec['road_id']
            if rid != remover_rids[vid]:
                raise ValueError(f"Remover {vid} thinks it's connected to road"
                                 f" {rid} but road {remover_rids[vid]} thinks"
                                 " it's the one.")

            # Put the road in remover's the spec as its upstream Facility
            spec['upstream'] = self.roads[rid]

            # Create the remover
            self.removers[vid] = VehicleRemover.from_spec(spec)

            # Attach the remover to the road
            self.roads[rid].connect_downstream(self.removers[vid])

            # On the road side, track that we've created this remover
            del remover_rids[rid]

        # Check if every road's remover (if it has one) has been created
        if len(remover_rids) > 0:
            raise ValueError(
                f"Roads have at least one remover that the spec doesn't.")

        # 2. Generate a Pathfinder from the specs and share it across modules
        SHARED.SETTINGS.pathfinder = Pathfinder(self.roads.values(),
                                                self.intersections.values(),
                                                lane_destination_pairs)

        # 3. Group them into common sets so it's neater to loop through
        self.facilities: List[Facility] = [*self.intersections.values(),
                                           *self.roads.values()]

        self.upstreams: List[Upstream] = [*self.spawners.values(),
                                          *self.intersections.values(),
                                          *self.roads.values()]

        self.downstreams: List[Downstream] = [*self.intersections.values(),
                                              *self.roads.values(),
                                              *self.removers.values()]

        # 4. Initialize vehicle loggers to track vehicles in the simulator as
        #    well as global entry, exit, and routing success for performance
        #    evaluation. See fetch_log for more information.
        self.vehicles_in_scope: Set[Vehicle] = set()
        self.vehicle_entry_times: Dict[Vehicle, int] = {}
        self.entry_exit_log: List[Tuple[int, Optional[int],
                                        Optional[bool]]] = []

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
            new_speeds.append(f.get_new_speeds())
        new_speed = dict(update for f_update in new_speeds
                         for update in f_update.items())
        for vehicle in self.vehicles_in_scope:
            update = new_speed[vehicle]
            vehicle.velocity = update.velocity
            vehicle.acceleration = update.acceleration

        # 2. Have upstream objects (vehicle spawners, roads, and intersections)
        #    update vehicle absolute (Coord) and relative positions (in-lane
        #    progression) based on speeds calculated during last step. If the
        #    front, center, or rear of a vehicle exits a lane, place it in the
        #    buffer of the downstream object. For spawners, this decides
        #    whether to create a new vehicle and if so places it in the
        #    downstream object.
        for u in self.upstreams:
            incoming_packet = u.step_vehicles()
            if incoming_packet is not None:
                spawned_vehicle, entering_vehicles = incoming_packet
                if spawned_vehicle is not None:
                    self.vehicle_entry_times[spawned_vehicle] = SHARED.t
                self.vehicles_in_scope.update(entering_vehicles)

        # 3. Have every downstream object (roads, intersections, and vehicle
        #    removers) resolve all vehicle transfers. This finishes updates to
        #    absolute and relative positions using speeds calculated in step 1.
        for d in self.downstreams:
            exiting_vehicles: Optional[List[Vehicle]
                                       ] = d.process_transfers()
            # only vehicle_removers return vehicles, iff any get removed
            # in this cycle
            if exiting_vehicles is not None:
                for vehicle in exiting_vehicles:
                    # log the vehicle
                    self.entry_exit_log.append(
                        (self.vehicle_entry_times[vehicle], SHARED.t, True))
                    # TODO: (multiple) determine if a vehicle successfully
                    #       reached its destination.

                    # remove it from our tracker
                    del self.vehicle_entry_times[vehicle]
                    self.vehicles_in_scope.remove(vehicle)

        # 4. Have facility managers handle their special internal logic (e.g.,
        #    lane changes and reservations).
        for f in self.facilities:
            f.update_schedule()

        # 5. Update shared time step and (TODO: (low)) shortest path values
        SHARED.t += 1
        SHARED.SETTINGS.pathfinder.update(None)

    def fetch_log(self) -> List[Tuple[int, Optional[int], Optional[bool]]]:
        """Returns log of times that vehicles spawned and exited the sim.

        This is a list of (int, int, bool) tuples denoting
            1. The vehicle's entry/spawning timestep
            2. The vehicle's exit/removal timestep
            3. Whether or not the vehicle successfully reached its intended
               destination


        # TODO: (logging) Collect more detailed information on vehicles in log.
        """
        log_image: List[Tuple[int, Optional[int],
                              Optional[bool]]] = self.entry_exit_log.copy()
        for vehicle in self.vehicles_in_scope:
            log_image.append((self.vehicle_entry_times[vehicle], None, None))
        return log_image

    def display(self) -> None:
        """Display the current position of all vehicles on the grid."""

        # TODO: consider having this return an image instead

        # since vehicles are the only things that change position on each step
        # loop through all vehicles and draw their position on the base image
        # at self.vis_basemap
        # for vehicle in self.Vehicles.values():
        raise NotImplementedError("TODO")
