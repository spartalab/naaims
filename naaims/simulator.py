"""
The simulators module contains the classes that create and run a new instance
of the AIM simulator, including visualizations if enabled.
"""

from __future__ import annotations
from typing import Generator, List, Tuple, Dict, Optional, Set, Any, Union
from math import sin, cos

from matplotlib.pyplot import subplots
from matplotlib.patches import Rectangle, Polygon
from matplotlib.animation import FuncAnimation

import naaims.shared as SHARED
from naaims.util import Coord, SpeedUpdate
from naaims.pathfinder import Pathfinder
from naaims.archetypes import Facility, Upstream, Downstream
from naaims.intersection import Intersection
from naaims.road import Road
from naaims.endpoints import VehicleSpawner, VehicleRemover
from naaims.vehicles import Vehicle, HumanGuidedVehicle


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
                 steps_per_second: int = 60,
                 speed_limit: int = 15,
                 min_braking: float = -2.6,
                 min_acceleration: float = 3,
                 max_vehicle_length: float = 5.5,
                 length_buffer_factor: float = 0.1,
                 acceptable_crash_mev: float = 0.,
                 visualize: bool = False,
                 visualize_tiles: bool = False) -> None:
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
            steps_per_second: int = 60
                The number of steps the simulation calculates per second of
                simulated time.
            speed_limit: int = 15
                The default speed limit in meters per second (can be overridden
                by specific roads or intersections in their specifications).
            min_braking: float = -2.6
                Maximum comfortable deceleration (braking) for all vehicles in
                m/s^2. Because this is based on human comfort, this should be
                also be lower than the maximum deceleration possible by every
                vehicle in the simulation, to the extent that we don't need to
                specify a separate deceleration for every vehicle in the
                simulation. Although this is called min_braking, this should be
                the braking rate expressed in negative values, corresponding to
                negative acceleration.
            min_acceleration: float = 3
                Like min_braking, for a vehicle's (positive) acceleration rate.
            max_vehicle_length: float = 5.5
            length_buffer_factor: float = 0.1
                Amount of clearance to maintain before and after each vehicle
                (separately), as a function of the vehicle's length. The
                default option, 0.1, makes a vehicle 10% longer in front and
                10% longer behind than it actually is.
            acceptable_crash_mev: float = 0.
                The rate of crashes the user is willing to tolerate per million
                entering vehicles (MEV), per intersection. Activates stochastic
                movement and reservation features if greater than 0.

                Internally, this is applied to the total vehicles per minute
                across all spawners to convert it into a probability of crash
                incidence per timestep for use in movement calculations.
            visualize: bool = False
                Whether to visualize the progress of the simulation.
            visualize_tiles: bool = False
                Show reservation status of tiles in intersections.
        """

        if acceptable_crash_mev < 0:
            raise ValueError("acceptable_crash_mev must be non-negative.")

        # 0. Load shared settings.
        SHARED.SETTINGS.load(steps_per_second=steps_per_second,
                             speed_limit=speed_limit,
                             min_braking=min_braking,
                             min_acceleration=min_acceleration,
                             max_vehicle_length=max_vehicle_length,
                             length_buffer_factor=length_buffer_factor)

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

        vpm_total: float = 0
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

            # Calculate the total vehicle arrival (spawn) rate
            vpm_total += spec['vpm']

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

        # Calculate acceptable crash probability per timestep per intersection.
        n_intersections = len(intersection_specs)
        crash_probability_tolerance = Simulator.crash_probability_tolerance(
            vpm_total, acceptable_crash_mev, SHARED.SETTINGS.steps_per_second
        )/n_intersections if n_intersections > 0 else 0

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
                spec['incoming_roads'].append(self.roads[rid])  # type: ignore
            spec['outgoing_roads'] = []
            for rid in spec['outgoing_road_ids']:
                # spec['outgoing_roads'][rid] = self.roads[rid]
                spec['outgoing_roads'].append(self.roads[rid])  # type: ignore

            # Convert the road IDs in the connectivity matrix into Roads
            connectivity: List[Tuple[int, int, bool]] = spec['connectivity']
            connectivity_converted: List[Tuple[Road, Road, bool]] = []
            for road_in_id, road_out_id, fully_connected in connectivity:
                connectivity_converted.append((self.roads[road_in_id],
                                               self.roads[road_out_id],
                                               fully_connected))
            spec['connectivity'] = connectivity_converted

            # Add the crash probability tolerance to the spec
            spec['manager_spec']['tiling_spec']['crash_probability_tolerance'
                                                ] = crash_probability_tolerance

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
        self.vehicle_log: Dict[int, Dict[str, Union[int, float, str]]] = {}

        # 5. If the visualize flag is enabled, draw the basemap image of
        #    roads and intersections for use later.
        self.visualize = visualize
        self.visualize_tiles = visualize_tiles
        if self.visualize:
            # Some configurable values
            figsize = (10, 10)
            background_color = '#2d5138'  # '#bf5700'  # 'black'
            lane_dash_color = '#d6d2c4'  # '.8'  # '.8'
            road_sep_color = '#ffd600'  # 'y'
            road_color = '#333f48'  # '.5'
            self.vehicle_color = '.9'  # '#ffffff'  # 'g'
            self.human_vehicle_outline_color = 'yellow'
            self.reserved_color = '#FF00FF'
            self.permitted_color = '#EFFD5F'
            self.tile_color = {
                1: '#FCF483',
                2: '#FF9D5C',
                3: '#F3ACAC'
            }

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

            # 6c. Ready a list for all changing objects being plotted.
            self.patches: List[Polygon] = []

            # 6d. Place time on vis.
            self.time_text = self.ax.text(min_x, min_y,  # type: ignore
                                          self.strf_t(), fontsize=16)

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
                spawned_vehicles, entering_vehicles = incoming_packet
                for spawn in spawned_vehicles:
                    self.vehicle_log[spawn.vin] = {}
                    self.vehicle_log[spawn.vin]['t_spawn'] = SHARED.t
                    self.vehicle_log[spawn.vin][
                        'destination_target'] = spawn.destination
                    self.vehicle_log[spawn.vin]['width'] = spawn.width
                    self.vehicle_log[spawn.vin]['length'] = spawn.length
                    self.vehicle_log[spawn.vin][
                        'throttle_mn'] = spawn.throttle_mn
                    self.vehicle_log[spawn.vin][
                        'throttle_sd'] = spawn.throttle_sd
                    self.vehicle_log[spawn.vin][
                        'tracking_mn'] = spawn.tracking_mn
                    self.vehicle_log[spawn.vin][
                        'tracking_sd'] = spawn.tracking_sd
                    self.vehicle_log[spawn.vin]['vot'] = spawn.vot
                    self.vehicle_log[spawn.vin]['type'] = type(spawn).__name__
                for entering in entering_vehicles:
                    self.vehicle_log[entering.vin]['t_entry'] = SHARED.t
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
                for exiting in exiting_vehicles:
                    self.vehicle_log[exiting.vin]['t_exit'] = SHARED.t
                    # TODO: (multiple) determine if a vehicle successfully
                    #       reached its destination. Will require adding an ID
                    #       property to VehicleRemovers.

                    # remove it from our tracker
                    self.vehicles_in_scope.remove(exiting)

        # 4. Have facility managers handle their special internal logic (e.g.,
        #    lane changes and reservations).
        for f in self.facilities:
            f.update_schedule(self.visualize_tiles)

        # 5. Update shared time step and (TODO: (low)) shortest path values
        SHARED.t += 1
        SHARED.SETTINGS.pathfinder.update(None)

    def save_log(self, filename: str) -> None:
        """Save log of vehicles spawned in the simulation to file in CSV form.

        The row ID is each vehicle's VIN. In order, each row is a vehicle's
             1. spawn time (seconds)
             2. time it enters the simulation space (seconds) if available
                (this will be the same as the spawn time unless the spawn lane
                queue backs up past the length of the lane) and -1 if it has
                yet to enter
             3. simulation space exit time (seconds) if available
             4. intended destination
             5. actual destination, if it's exited (-1 if unavailable)
             6. width in meters
             7. length in meters
             8. throttle mean
             9. throttle sd
            10. tracking mean
            11. tracking sd
            12. value of time (VOT) in units per second
            13. vehicle type/class
        """
        with open(filename, 'w') as f:
            f.write("t_spawn,t_entry,t_exit,destination_target,"
                    "destination_actual,width,length,throttle_mn,throttle_sd,"
                    "tracking_mn,tracking_sd,vot,type")
            for _, vehicle_log in sorted(self.vehicle_log.items()):
                f.write(f'{vehicle_log["t_spawn"]},'
                        f'{vehicle_log.get("t_entry", -1)},'
                        f'{vehicle_log.get("t_exit", -1)},'
                        f'{vehicle_log["destination_target"]},'
                        f'{vehicle_log.get("destination_actual", -1)},'
                        f'{vehicle_log["width"]},'
                        f'{vehicle_log["length"]},'
                        f'{vehicle_log["throttle_mn"]},'
                        f'{vehicle_log["throttle_sd"]},'
                        f'{vehicle_log["tracking_mn"]},'
                        f'{vehicle_log["tracking_sd"]},'
                        f'{vehicle_log["vot"]},'
                        f'{vehicle_log["type"]}\n')

    def animate(self, frame_ratio: int = 1, max_timestep: int = 10*60
                ) -> FuncAnimation:  # type: ignore
        """Animate the simulation run.

        Parameters:
            frame_ratio: int = 1
                How often to save frames per sim timestep, in timesteps per
                frame. Defaults saves one frame per timestep.
        """

        t0 = SHARED.t

        def draw(packet: Tuple[Set[Vehicle], Tuple[Intersection, ...]]
                 ) -> List[Polygon]:

            vehicles: Set[Vehicle] = packet[0]
            intersections: Tuple[Intersection, ...] = packet[1]

            # Prepare to remove last timestep's patches.
            changed = self.patches.copy()
            for patch in self.patches:
                patch.remove()
            self.patches = []

            # Prepare new vehicle patches
            for vehicle in vehicles:
                vehicle_color: str
                if vehicle.has_reservation:
                    vehicle_color = self.reserved_color
                elif vehicle.permission_to_enter_intersection:
                    vehicle_color = self.permitted_color
                else:
                    vehicle_color = self.vehicle_color
                patch = Polygon(
                    vehicle.get_outline(), facecolor=vehicle_color, alpha=1,
                    edgecolor=self.human_vehicle_outline_color if
                    (type(vehicle) is HumanGuidedVehicle) else None,
                    zorder=5)
                self.ax.add_patch(patch)
                self.patches.append(patch)
                changed.append(patch)

            # Prepare new tile patches
            for intersection in intersections:
                if intersection.tiles_to_visualize is not None:
                    for outline, p, n in intersection.tiles_to_visualize:
                        n = 3 if (n > 3) else n
                        patch = Polygon(
                            outline, facecolor=self.tile_color[n],
                            alpha=(p**.2)/2, edgecolor=None, zorder=4)
                        self.ax.add_patch(patch)
                        self.patches.append(patch)
                        changed.append(patch)

            # Update time
            self.time_text.set_text(self.strf_t())
            changed.append(self.time_text)  # type: ignore
            return changed

        def get_next_frame() -> Generator[
                Tuple[Set[Vehicle], Tuple[Intersection, ...]], None, None]:
            while (SHARED.t - t0) < max_timestep:
                self.step()
                if len(self.vehicles_in_scope) > 0:
                    assert len(self.vehicles_in_scope) > 0
                if (SHARED.t - t0) % frame_ratio == 0:
                    yield self.vehicles_in_scope, tuple(
                        self.intersections.values())

        return FuncAnimation(self.fig, draw,  # type: ignore
                             frames=get_next_frame, interval=frame_ratio *
                             SHARED.SETTINGS.TIMESTEP_LENGTH * 1000,
                             save_count=max_timestep*frame_ratio, blit=True)

    def strf_t(self) -> str:
        """Return the current timestep as a nicely formatted time string."""
        time = SHARED.t * SHARED.SETTINGS.TIMESTEP_LENGTH
        min = time//60
        sec = time % 60
        return f'{min:02.0f}:{sec:06.3f}'

    @staticmethod
    def crash_probability_tolerance(vpm: float, crash_mev: float,
                                    timesteps_per_second: float) -> float:
        """Find the acceptable crash probability per intersection.

        This is
          crashes per million vehicles
              divided by
          1 million, for crashes per vehicle
              multiplied by
          vehicles per minute, for crashes per minute
              divided by
          60 seconds per minute, for crashes per second
              divided by
          steps per second, for crashes per timestep.
        This will need to be broken up further for each stochastic tile.
        """
        return crash_mev / 1e6 * vpm / 60 / timesteps_per_second
