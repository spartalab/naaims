"""
`simulator` implements the class that runs a new instance of the AIM simulator,
including visualizations if enabled.
"""

from abc import ABC, abstractmethod
from typing import Iterable, Tuple, Dict, Optional, Set
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
                 intersections: Iterable[Intersection],
                 roads: Iterable[Road],
                 vehicle_spawners: Iterable[VehicleSpawner],
                 vehicle_removers: Iterable[VehicleRemover],
                 config_filename: str = './config.ini',
                 display: bool = False) -> None:
        """Should create all roads, intersections, and suppport structures.

        This abstract init that all Simulators should call serves only to
        initialize data structures for lookup, calls, and visualization.
        """

        # First, read in the config file.
        SETTINGS.read(config_filename)

        # Next, save every collection of first level objects. That is,
        # intersections, roads, vehicle_spawners, and vehicle_removers.
        self.intersections: Iterable[Intersection] = intersections
        self.roads: Iterable[Road] = roads
        self.vehicle_spawners: Iterable[VehicleSpawner] = vehicle_spawners
        self.vehicle_removers: Iterable[VehicleRemover] = vehicle_removers
        # TODO note: road entrance region length should be determined by (but
        #            likely won't be for sake of convenience) the vehicle with
        #            the longest length+(speed_limit)**2/(2*braking)

        # Initialize an empty list of vehicles
        # self.vehicles: Dict[int, Vehicle] = {}
        self.vehicles: Set[Vehicle] = set()

        # Group them into common sets so it's quicker to loop through them all
        self.facilities: Tuple[Iterable[Facility], ...] = (
            self.intersections,
            self.roads
        )
        self.upstreams: Tuple[Iterable[Upstream], ...] = (
            self.vehicle_spawners,
            self.intersections,
            self.roads
        )
        self.downstreams: Tuple[Iterable[Downstream], ...] = (
            self.intersections,
            self.roads,
            self.vehicle_removers
        )

        # Given these, verify that the intersections and roads are compatible
        # (i.e., that the endpoints all match), and stitch them together.
        # TODO: implement this, making sure to create lookup dicts
        self.upstream_lookup: Dict[Coord, Upstream] = {}
        self.downstream_lookup: Dict[Coord, Downstream] = {}
        # for i in self.intersections + self.roads:
        #     to_merge = i.upstream_connection_coords
        #     # merge lookup with self.upstream_lookup
        #     to_merge = i.downstream_connection_coords
        #     # merge lookup with self.downstream_lookup
        # for s in self.vehicle_spawners:
        #     to_merge = s.downstream_connection_coords
        # for r in self.vehicle_removers:
        #     to_merge = r.upstream_connection_coords

        # Stitch the roads together
        for spawner in vehicle_spawners:
            spawner.finish_setup()
        for road in roads:
            road.finish_setup()
        for intersection in intersections:
            intersection.finish_setup()
        for remover in vehicle_removers:
            vehicle_removers.finish_setup()

        # Finally, if the visualize flag is enabled, draw the basemap image of
        # roads+intersection and save for later
        if display:
            # loop through all roads and visualize their length, width, lane
            # markings, etc. using their trajectory
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
        """Execute one simulation step.

        1. update_speeds (Facilities)
        2. step (Upstreams)
        3. process_transfers (Downstreams)
        4. handle_logic (Facilities)
        """

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
