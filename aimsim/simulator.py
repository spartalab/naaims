"""
The simulators module contains the classes that create and run a new instance
of the AIM simulator, including visualizations if enabled.
"""

from __future__ import annotations
from typing import Generator, List, Tuple, Dict, Optional, Set, Any
from math import sin, cos

from matplotlib.pyplot import subplots
from matplotlib.patches import Rectangle, Polygon
from matplotlib.animation import FuncAnimation

import aimsim.shared as SHARED
from aimsim.util import Coord, SpeedUpdate
from aimsim.pathfinder import Pathfinder
from aimsim.archetypes import Facility, Upstream, Downstream
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
                 visualize: bool = False) -> None:
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
            visualize: bool = False
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
            spec['incoming_roads'] = []
            for rid in spec['incoming_road_ids']:
                # spec['incoming_roads'][rid] = self.roads[rid]
                spec['incoming_roads'].append(self.roads[rid])
            spec['outgoing_roads'] = []
            for rid in spec['outgoing_road_ids']:
                # spec['outgoing_roads'][rid] = self.roads[rid]
                spec['outgoing_roads'].append(self.roads[rid])

            # Convert the road IDs in the connectivity matrix into Roads
            connectivity: List[Tuple[int, int, bool]] = spec['connectivity']
            connectivity_converted: List[Tuple[Road, Road, bool]] = []
            for road_in_id, road_out_id, fully_connected in connectivity:
                connectivity_converted.append((self.roads[road_in_id],
                                               self.roads[road_out_id],
                                               fully_connected))
            spec['connectivity'] = connectivity_converted

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
            del spawner_rids[sid]

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
            del remover_rids[vid]

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
        self.visualize = visualize
        if self.visualize:
            # Some configurable values
            figsize = (10, 10)
            background_color = '#2d5138'  # '#bf5700'  # 'black'
            lane_dash_color = '#d6d2c4'  # '.8'  # '.8'
            road_sep_color = '#ffd600'  # 'y'
            road_color = '#333f48'  # '.5'
            self.vehicle_color = '.9'  # '#ffffff'  # 'g'

            self.fig, self.ax = subplots(figsize=figsize)
            # TODO: Variable figsize.
            self.fig.subplots_adjust(left=0, bottom=0, right=1, top=1,
                                     wspace=None, hspace=None)
            self.fig.patch.set_facecolor(background_color)
            self.fig.patch.set_alpha(1)
            self.ax.margins(x=0, y=0)
            self.ax.axis('off')
            self.ax.patch.set_alpha(0)
            self.ax.set_aspect(1)

            min_x = max_x = self.roads[0].trajectory.start_coord.x
            min_y = max_y = self.roads[0].trajectory.start_coord.x

            # 6a. Loop through all roads and visualize their length, width,
            #     and lane markings, etc. using their trajectory.
            for road in self.roads.values():
                offset_angle = road.lane_offset_angle + \
                    road.trajectory.get_heading(1)
                spacing = Coord(road.lane_width*sin(offset_angle)/2,
                                road.lane_width*cos(offset_angle)/2)

                # Plot lane markings
                for lane in road.lanes[:-1]:
                    coords: List[Coord] = [lane.trajectory.start_coord,
                                           lane.trajectory.end_coord]
                    self.ax.plot([c.x+spacing.x for c in coords],
                                 [c.y+spacing.y for c in coords],
                                 '--', c=lane_dash_color)

                # Plot road edge markings
                lane_first = road.lanes[0]
                coords = [lane_first.trajectory.start_coord,
                          lane_first.trajectory.end_coord]
                top_xs = [c.x-spacing.x for c in coords]
                top_ys = [c.y-spacing.y for c in coords]
                self.ax.plot(top_xs, top_ys, c=road_sep_color)
                lane_last = road.lanes[-1]
                coords = [lane_last.trajectory.start_coord,
                          lane_last.trajectory.end_coord]
                bot_xs = [c.x+spacing.x for c in coords]
                bot_ys = [c.y+spacing.y for c in coords]
                self.ax.plot(bot_xs, bot_ys, c=road_sep_color)

                # Fill in road
                self.ax.add_patch(Polygon([[top_xs[0], top_ys[0]],
                                           [top_xs[1], top_ys[1]],
                                           [bot_xs[1], bot_ys[1]],
                                           [bot_xs[0], bot_ys[0]]],
                                          facecolor=road_color))

                # Update all-plot bounds
                coords = [road.trajectory.start_coord,
                          road.trajectory.end_coord]
                xs = [c.x for c in coords]
                ys = [c.y for c in coords]
                for x in xs:
                    if min_x > x:
                        min_x = x
                    if max_x < x:
                        max_x = x
                for y in ys:
                    if min_y > y:
                        min_y = y
                    if max_y < y:
                        max_y = y

            # 6b. Loop through all intersections and visualize their area.
            for intersection in self.intersections.values():
                int_min_x = int_max_x = \
                    intersection.lanes[0].trajectory.start_coord.x
                int_min_y = int_max_y = \
                    intersection.lanes[0].trajectory.start_coord.y
                coords = []
                for i_lane in intersection.lanes:
                    coords.append(i_lane.trajectory.start_coord)
                    coords.append(i_lane.trajectory.end_coord)
                xs = [c.x for c in coords]
                ys = [c.y for c in coords]
                for x in xs:
                    if int_min_x > x:
                        int_min_x = x
                    if int_max_x < x:
                        int_max_x = x
                for y in ys:
                    if int_min_y > y:
                        int_min_y = y
                    if int_max_y < y:
                        int_max_y = y
                self.ax.add_patch(Rectangle((int_min_x, int_min_y),
                                            int_max_x-int_min_x,
                                            int_max_y-int_min_y,
                                            facecolor=road_color))

            # 6c. Ready a list for all vehicles being plotted.
            self.vehicle_patches: List[Polygon] = []

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
        new_speeds: Dict[Vehicle, SpeedUpdate] = {}
        for f in self.facilities:
            new_speeds.update(f.get_new_speeds())
        for vehicle in self.vehicles_in_scope:
            update = new_speeds[vehicle]
            vehicle.acceleration = update.acceleration
            vehicle.velocity = update.velocity

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

    def animate(self, frame_ratio: int = 1, max_timestep: int = 10*60
                ) -> FuncAnimation:
        """Animate the simulation run.

        Parameters:
            frame_ratio: int = 1
                How often to save frames per sim timestep, in timesteps per
                frame. Defaults saves one frame per timestep.
        """

        t0 = SHARED.t

        def draw(vehicles: Set[Vehicle]) -> List[Polygon]:
            changed = self.vehicle_patches.copy()
            for patch in self.vehicle_patches:
                patch.remove()
            self.vehicle_patches = []
            for vehicle in vehicles:
                vehicle_patch = Polygon(vehicle.get_outline(),
                                        facecolor=self.vehicle_color, alpha=1,
                                        edgecolor=None, zorder=5)
                self.ax.add_patch(vehicle_patch)
                self.vehicle_patches.append(vehicle_patch)
                changed.append(vehicle_patch)
            return changed

        def get_next_frame() -> Generator[Set[Vehicle], None, None]:
            while (SHARED.t - t0) < max_timestep:
                self.step()
                if len(self.vehicles_in_scope) > 0:
                    assert len(self.vehicles_in_scope) > 0
                if (SHARED.t - t0) % frame_ratio == 0:
                    yield self.vehicles_in_scope

        return FuncAnimation(self.fig, draw, frames=get_next_frame,
                             interval=frame_ratio *
                             SHARED.SETTINGS.TIMESTEP_LENGTH * 1000, save_count=max_timestep*frame_ratio, blit=True)
