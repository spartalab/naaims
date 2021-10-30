from __future__ import annotations
from typing import Any, TYPE_CHECKING, Optional, Set, Dict, Tuple, List, Type
from math import ceil, floor, isclose
from warnings import warn

import naaims.shared as SHARED
from naaims.util import Coord, CollisionError
from naaims.intersection.tilings.tiling import Tiling
from naaims.intersection.tilings.tiles import DeterministicTile

if TYPE_CHECKING:
    from naaims.road import RoadLane
    from naaims.intersection.tilings.tiles import Tile, DeterministicTile
    from naaims.vehicles import Vehicle
    from naaims.intersection import IntersectionLane
    from naaims.intersection.reservation import Reservation


class SquareTiling(Tiling):
    def __init__(self,
                 incoming_road_lane_by_coord: Dict[Coord, RoadLane],
                 outgoing_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Tuple[IntersectionLane, ...],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 crash_probability_tolerance: float,
                 cycle: Optional[List[
                     Tuple[Set[IntersectionLane], int]
                 ]] = None,
                 timeout: bool = False,
                 tile_type: Type[Tile] = DeterministicTile,
                 misc_spec: Dict[str, Any] = {}
                 ) -> None:
        """Creates a new square (pixelated) tiling."""

        super().__init__(
            incoming_road_lane_by_coord=incoming_road_lane_by_coord,
            outgoing_road_lane_by_coord=outgoing_road_lane_by_coord,
            lanes=lanes,
            crash_probability_tolerance=crash_probability_tolerance,
            lanes_by_endpoints=lanes_by_endpoints,
            cycle=cycle,
            timeout=timeout
        )

        tile_width = misc_spec.get('tile_width')  # in meters
        if tile_width is None:
            raise ValueError("tile_width must be provided.")
        self.tile_width = tile_width

        # Identify bounding box by looking for min and max x and y coordinates
        # across all IntersectionLanes.
        self.min_x: float = float('inf')
        self.max_x: float = -float('inf')
        self.min_y: float = float('inf')
        self.max_y: float = -float('inf')
        for start, end in lanes_by_endpoints:
            # TODO: (medium) Account for lane width.
            for xy in (start, end):
                if xy.x < self.min_x:
                    self.min_x = xy.x
                if xy.x > self.max_x:
                    self.max_x = xy.x
                if xy.y < self.min_y:
                    self.min_y = xy.y
                if xy.y > self.max_y:
                    self.max_y = xy.y

        x_tile_count = (self.max_x - self.min_x)/self.tile_width
        y_tile_count = (self.max_y - self.min_y)/self.tile_width
        self.x_tile_count = ceil(x_tile_count)
        self.y_tile_count = ceil(y_tile_count)

        # Normalize the origin to be the bottom left Coord of this grid. The
        # Tiles will be stored left-to-right, top-to-bottom but the y-axis will
        # be flipped to match normal coordinate schemas.
        self.origin = Coord(self.min_x, self.min_y)

        # Find the 1D index of the tile at every road lane connection to the
        # intersection. We'll use them to buffer entries and exits into the
        # intersection so vehicles don't crash as they enter or exit.
        self.buffer_tile_loc: Dict[Coord, int] = {}
        for start, end in lanes_by_endpoints:
            self.buffer_tile_loc[start] = self._io_coord_to_tile_id(start)
            self.buffer_tile_loc[end] = self._io_coord_to_tile_id(end)

        # Find rejection threshold for each tile from the
        # crash_probability_tolerance over the entire intersection by using a
        # simple approximation where each tile is independently contributing to
        # the total probability of rejection.
        total_tiles = self.x_tile_count * self.y_tile_count
        # The following two expressions are equivalent for small probabilities,
        # which the crash probability should always be, thanks to the magic of
        # Taylor series expansions.
        # self._threshold = 1 - (1-crash_probability_tolerance)**(
        #     1 / total_tiles) if (total_tiles > 0) else 0
        self._threshold = crash_probability_tolerance / \
            total_tiles if (total_tiles > 0) else 0
        # Above is a stricter calculation of the rejection threshold, but I
        # think this should be fine as the probability of crashes across
        # multiple tiles is highly correlated.
        # self._threshold = crash_probability_tolerance
        self.threshold_registered = True

    def check_for_collisions(self) -> None:
        """Check for collisions in the intersection.

        Collisions are only possible when stochastic movement is on, so this
        only runs in that case.
        """
        if self.threshold > 0:
            seen: Dict[Tuple[int, int], List[Vehicle]] = {}
            for lane in self.lanes:
                for vehicle in lane.vehicles:
                    y_min, x_mins, x_maxes = self._outline_to_tile_range(tuple(
                        Coord((c.x - self.origin.x)/self.tile_width,
                              (c.y - self.origin.y)/self.tile_width)
                        for c in vehicle.get_outline()))
                    for j in range(len(x_mins)):
                        y = y_min + j
                        for i in range((x_maxes[j]+1)-x_mins[j]):
                            x = x_mins[j]+i
                            loc = (x, y)
                            if loc not in seen:
                                seen[loc] = [vehicle]
                            else:
                                seen[loc].append(vehicle)
                            if len(seen[loc]) > 1:
                                info = "Potential collision at tile " + \
                                    f"{self._tile_loc_to_coord(loc)}."
                                for veh in seen[loc]:
                                    info += '\n\t' + str(veh.get_outline())
                                warn(info)
                                # raise CollisionError(
                                #     "Two or more vehicles using the tile at "
                                #     f"{self._tile_loc_to_coord(loc)}.")

    def update_active_reservations(self) -> None:
        """Given vehicle movement in the last step, update their reservations.

        Have manager and tiling compare the current positions and velocities
        of all vehicles in intersection. Use the difference to update
        reservations to reflect resolved stochasticity. Should only be
        necessary for stochastic reservations, but it might be nice for
        deterministic methods with nonzero buffers, including when a vehicle
        exits the intersection.
        """
        # TODO: (low) Adjust tiles reserved based on resolved movement.
        pass

    def pos_to_tiles(self, lane: IntersectionLane, t: int,
                     clone: Vehicle, reservation: Reservation,
                     force: bool = False, mark: bool = False
                     ) -> Optional[Dict[Tile, float]]:
        """Return a vehicle's tiles and percentage used if it works.

        Given a vehicle with a test-updated position, its in-lane progress, and
        a future timestep t, return the tiles used by this vehicle in this
        timestep given its position if the reservation is possible. If not,
        return None (unless the force flag is True, in which case return the
        tiles used regardless of whether the tiles allow it).

        We adopt the convention that points are assigned to tiles based on
        their floored x and y values, except in the case of points on the upper
        or left boundary of the tile space (self.y_tile_count or
        self.x_tile_count); in these cases, we assign that point to the tile
        that they're on the upper or right edge of (the ceil).

        Parameters
            lane: IntersectionLane
                The lane the clone is tested on.
            t: int
                The future timestep we're checking tiles for.
            clone: Vehicle
                The vehicle whose position we're translating to tiles used.
            progress: VehicleProgress
                How far along the lane the clone is at this timestep.
            reservation: Reservation
                The reservation associated with this clone.
            force: bool = False
                If force, don't bother checking if a tile is compatible with
                this vehicle's reservation before returning.
            mark: bool = False
                Whether to mark the tiles used with this potential reservation.
        """
        # Call super to extend the tile stack to cover up to t if necessary.
        super().pos_to_tiles(lane, t, clone, reservation, force, mark)

        if self.threshold == 0:
            # Fetch the vehicle's outline as provided by the lane's stochastic
            # movement model, normalize it to the grid's internal coordinate
            # system using self.origin and self.tile_width, and find the x and
            # y range covered by the outline under the assumption that the
            # outline is a convex shape.
            y_min, x_mins, x_maxes = self._outline_to_tile_range(tuple(
                Coord((c.x - self.origin.x)/self.tile_width,
                      (c.y - self.origin.y)/self.tile_width)
                for c in clone.get_outline(static_buffer=.1)))
        else:
            # Crash probability is non-zero. Look at every tile.
            y_min = 0
            x_mins = [0 for _ in range(self.y_tile_count)]
            x_maxes = [self.x_tile_count-1 for _ in range(self.y_tile_count)]

        # After the outlining process is complete, loop through the min and max
        # y-Tiles via the x-bound lists. For each y-value, add every tile
        # between the min and max x-Tiles to the return set.
        tiles_covered: Dict[Tile, float] = {}
        # Recall that the first tile layer represents the next timestep.
        t_idx = t - (SHARED.t+1)
        for j in range(len(x_mins)):
            y = y_min + j
            for i in range((x_maxes[j]+1)-x_mins[j]):
                x = x_mins[j]+i
                tile = self.tiles[t_idx][self._tile_loc_to_id((x, y))]

                # If crash probability is non-zero, find a probability of usage
                # for every tile. If not, only tiles that will be used have
                # been outlined, so they'll be reserved with 100% usage.
                p = lane.movement_model.find_probability_of_usage(
                    clone, lane.vehicle_progress[clone],
                    self._tile_loc_to_coord((x, y)), self.tile_width, t) \
                    if (self.threshold > 0) else 1

                if not tile.will_reservation_work(reservation, p):
                    lane.movement_model.clean_up_projection(clone)
                    return None

                tiles_covered[tile] = p

        # TODO: (auction) incorporate force and mark

        return tiles_covered

    def _outline_to_tile_range(self, outline: Tuple[Coord, ...]) \
            -> Tuple[int, List[int], List[int]]:

        if len(outline) < 3:
            raise ValueError('Must be a 2D outline (>= 3 vertices).')

        # Start by finding the y-range the outline covers.
        y_min: int = floor(outline[0].y)
        y_max: int = y_min
        for o in outline:
            oy = floor(o.y)
            if oy < y_min:
                y_min = oy
            if oy > y_max:
                y_max = oy
        # Initialize lists for the x-range associated with each y-value.
        x_mins: List[int] = [self.x_tile_count]*(y_max-y_min+1)
        x_maxes: List[int] = [-1]*(y_max-y_min+1)
        # Loop through the outline clockwise to fill out these lists.
        for i in range(len(outline)):
            start = outline[i]
            end = outline[i+1] if (i != len(outline)-1) else outline[0]
            y_min_seg, x_mins_seg, x_maxes_seg = self._line_to_tile_ranges(
                start, end)

            # Merge the x-mins and maxes from the segment into the overall
            # tracker. One of mins or maxes will be empty depending on if the
            # segment pointed up or down, as that indicates if the segment
            # alters mins or maxes.
            for j, x_min in enumerate(x_mins_seg):
                idx = j + y_min_seg - y_min
                x_mins[idx] = min(x_min, x_mins[idx])
            for j, x_max in enumerate(x_maxes_seg):
                idx = j + y_min_seg - y_min
                x_maxes[idx] = max(x_max, x_maxes[idx])
            # Note: this doesn't work for non-convex outlines.

        # Go through the stitched range to clip tiles to between x_min, x_max,
        # y_min, and y_max.
        return self._clip_tile_range(y_min, x_mins, x_maxes)

    def _line_to_tile_ranges(self, start: Coord, end: Coord) -> \
            Tuple[int, List[int], List[int]]:
        """Given two points on the grid, find the Tiles their connection hits.

        Assumes line segments are being provided in clockwise order, to inform
        what sides of the tiles hit are being returned.

        Uses http://www.cse.yorku.ca/~amana/research/grid.pdf to find Tiles.

        Returns y_min, x_mins, x_maxes.
        """
        dx = 0 if isclose(end.x, start.x, abs_tol=1e-9) else end.x - start.x
        dy = 0 if isclose(end.y, start.y, abs_tol=1e-9) else end.y - start.y

        x_mins: List[int] = []
        x_maxes: List[int] = []
        y_min: int
        y_max: int

        # Recall that points are assigned to tiles based on their floored x and
        # y values.
        if dy == 0:  # horizontal or a single point
            y_min = floor(start.y)
            x_mins.append(floor(min(start.x, end.x)))
            x_maxes.append(floor(max(start.x, end.x)))
        elif dx == 0:  # vertical
            y_min = floor(min(start.y, end.y))
            y_max = floor(max(start.y, end.y))
            x_value = floor(start.x)
            x_mins = [x_value]*(y_max-y_min+1)
            x_maxes = x_mins.copy()
        else:
            # Progress through every pixel the ray intersects using the
            # algorithm from http://www.cse.yorku.ca/~amana/research/grid.pdf
            # with some modifications to account for finite line segments.

            # Set up the line segment equation and bounds
            m = dy/dx
            b = end.y - m*end.x
            # def y_of_x(x: float) -> float: return m*x+b
            def x_of_y(y: float) -> float: return (y-b)/m
            x_dist_max = abs(end.x - start.x)

            # Find rasterization y-axis bounds, initialize rasterization
            # x-value bound lists, and find x_to_next_y_tile, how far along the
            # line we need to step from start in x-units to reach the next Tile
            # with a new (integer) y-value.
            step_y: int
            x_mins = []
            x_maxes = []
            if dy < 0:  # down right or left, maxes
                y_max = floor(start.y)
                y_min = floor(end.y)
                x_maxes = [floor(start.x)]*(y_max-y_min+1)
                step_y = -1
                x_of_next_y_tile = x_of_y(floor(start.y))
            else:  # up right or left, mins
                y_min = floor(start.y)
                y_max = floor(end.y)
                x_mins = [floor(start.x)]*(y_max-y_min+1)
                step_y = 1
                x_of_next_y_tile = x_of_y(ceil(start.y))
            x_to_next_y_tile: float = abs(start.x - x_of_next_y_tile)

            # Find x_to_next_x_tile, how far along the line we need to step
            # from start in x-units to reach the next Tile with a new x-value.
            x_to_next_x_tile: float = start.x % 1 if dx < 0 else 1-start.x % 1
            step_x: int = -1 if dx < 0 else 1
            x_delta_x: float = 1
            x_delta_y: float = abs(1/m)
            if isclose(x_to_next_y_tile, 0, abs_tol=1e-10) and (dy > 0):
                # Recall that tiles are inclusive of their bottom right border.
                # When we start on the border between y-tiles, x_to_next_y_tile
                # will start at 0, and in this preamble we add the tile the
                # line segment starts in. When going down (dy < 0), we traverse
                # the next y-tile instantaneously, so we need to add it, but
                # when we go up (dy > 0), we don't need to add the tile
                # immediately below and so we increment x_to_next_y_tile to
                # show that there's non-infinitesimal distance until we reach
                # the next y value.
                x_to_next_y_tile += x_delta_y

            # Find Tile coordinates of the starting Tile and log them.
            # Accommodate the special cases where the line segment starts on
            # the upper or right boundaries but does not reach the lower or
            # right bounds of the tile it's in. Remember that straight lines
            # have already been accounted for.
            x: int = floor(start.x)
            y: int = floor(start.y)

            # Log the x max or min of the starting position.
            if dy < 0:
                x_maxes[y - y_min] = x
            else:
                x_mins[y - y_min] = x

            # Traverse the line segment Tile-by-Tile in order from start to
            # end. Track if there's already been a Tile in this x-band marked;
            # if there has, new x-Tiles traversed need only be included in
            # x_mins or x_maxes if dx points in a direction that would make the
            # last x_min or x_max logged insufficiently lenient.
            marked: bool = True
            while True:  # break condition in loop
                # We have to be careful of cumulative floating point errors.
                close_call = isclose(x_to_next_x_tile, x_to_next_y_tile)
                # if close to equal and going down, prefer x update
                # if close to equal and going up, prefer y update
                if close_call and (dy < 0):
                    # If close and going down, prefer x update
                    if (x_to_next_x_tile - x_dist_max > 1e-9) or (
                        isclose(x_to_next_x_tile, x_dist_max) and (dy > 0)
                    ):
                        break
                    x += step_x
                    x_to_next_x_tile += x_delta_x
                elif close_call and (dy > 0):
                    # If close and going up, prefer y update
                    if (x_to_next_y_tile - x_dist_max > 1e-9) or (
                        isclose(x_to_next_y_tile, x_dist_max) and (dy < 0)
                    ):
                        break
                    y += step_y
                    x_to_next_y_tile += x_delta_y
                    marked = False
                elif x_to_next_x_tile < x_to_next_y_tile:
                    # If next x is closer, prefer x update; note that this
                    # section is identical to two blocks up, I just couldn't
                    # be bothered to modularize two lines and a break check.
                    if (x_to_next_x_tile - x_dist_max > 1e-9) or (
                        isclose(x_to_next_x_tile, x_dist_max) and (dy > 0)
                    ):
                        break
                    x += step_x
                    x_to_next_x_tile += x_delta_x
                else:
                    # If next y is closer, prefer y update; note that this
                    # section is identical to two blocks up, I just couldn't
                    # be bothered to modularize two lines and a break check.
                    if (x_to_next_y_tile - x_dist_max > 1e-9) or (
                        isclose(x_to_next_y_tile, x_dist_max) and (dy < 0)
                    ):
                        break
                    y += step_y
                    x_to_next_y_tile += x_delta_y
                    marked = False

                # Log the Tile into our bounds if conditions are met and mark
                # it traversed.
                if dy < 0:
                    # Line segment points down, so these are maxes. If down
                    # right (dx > 0 and dy < 0), overwrite already marked max.
                    if dx > 0 or not marked:
                        x_maxes[y - y_min] = x
                        marked = True
                else:
                    # Line segment points up (dy > 0), so these are mins. If up
                    # left (dx < 0 and dy > 0), overwrite already marked min.
                    if dx < 0 or not marked:
                        x_mins[y - y_min] = x
                        marked = True

        return y_min, x_mins, x_maxes

    def _clip_tile_range(self, y_min: int, x_mins: List[int],
                         x_maxes: List[int]
                         ) -> Tuple[int, List[int], List[int]]:
        """Go through the provided range and clip it to the grid boundaries."""

        # Find how much to clip x_mins and x_maxes from above.
        y_max = y_min + len(x_mins) - 1
        y_above_range = y_max - (self.y_tile_count - 1) if \
            (y_max >= self.y_tile_count) else 0

        # Find how far to clip x_mins and x_maxes from below.
        if y_min < 0:
            y_below_range = -y_min
            y_min = 0
        else:
            y_below_range = 0

        # Fix y_min
        if y_min >= self.y_tile_count:
            y_min = max(0, self.y_tile_count-1)

        # Clip x_mins and x_maxes according to y-values
        x_mins = x_mins[y_below_range:max(0, len(x_mins)-y_above_range)]
        x_maxes = x_maxes[y_below_range:max(0, len(x_maxes)-y_above_range)]

        # Clip x_mins and x_maxes according to x-values, deleting rows if they
        # are fully outside the range.
        first_valid_row: Optional[int] = None
        last_valid_row: int = 0
        for i in range(len(x_mins)):
            if x_maxes[i] < 0 or x_mins[i] >= self.x_tile_count:
                if first_valid_row is None:
                    continue
                else:
                    break
            else:
                if first_valid_row is None:
                    first_valid_row = i
                last_valid_row = i
            x_mins[i] = 0 if x_mins[i] <= 0 else x_mins[i]
            x_mins[i] = self.x_tile_count-1 if \
                x_mins[i] >= self.x_tile_count else x_mins[i]
            x_maxes[i] = 0 if x_maxes[i] <= 0 else x_maxes[i]
            x_maxes[i] = self.x_tile_count-1 if \
                x_maxes[i] >= self.x_tile_count else x_maxes[i]

        if first_valid_row is not None:
            y_min += first_valid_row
            x_mins = x_mins[first_valid_row:last_valid_row+1]
            x_maxes = x_maxes[first_valid_row:last_valid_row+1]
        else:
            x_mins = x_maxes = []

        return y_min, x_mins, x_maxes

    def _tile_loc_to_coord(self, tile_loc: Tuple[int, int]) -> Coord:
        """Convert a tile's (x,y) location to its true Coord."""
        return Coord(tile_loc[0]*self.tile_width + self.min_x,
                     tile_loc[1]*self.tile_width + self.min_y)

    def io_tile_buffer(self, lane: IntersectionLane, t: int,
                       clone: Vehicle, reservation: Reservation,
                       prepend: bool, force: bool = False, mark: bool = False
                       ) -> Optional[Dict[int, Dict[Tile, float]]]:
        """Should return edge buffer tiles and percentages used if it works.

        Given a vehicle with a test-updated position, its in-lane progress, and
        a future timestep t, return the edge tiles used by this vehicle a few
        timesteps before or after (depending on the prepend flag), provided the
        tiles can accept this reservation. If not, return None (unless the
        force flag is True, in which case return the tiles used regardless of
        whether the tiles allow it).

        Note that this does not return edge tiles AT the timestep t, only edge
        buffer tiles immediately before or after depending on prepend's value.

        Reserving a buffer of edge tiles ensures that there is proper spacing
        between vehicles on entry and exit to an automated intersection. See
        Dresner and Stone 2008 for more information.

        Parameters
            lane: IntersectionLane
                The lane the clone is tested on.
            t: int
                The future timestep we're checking tiles for.
            clone: Vehicle
                The vehicle whose position we're translating to tiles used.
            progress: VehicleProgress
                How far along the lane the clone is at this timestep.
            reservation: Reservation
                The reservation associated with this clone.
            prepend: bool
                If true, return edge tiles before timestep. If false, return
                edge tiles after timestep.
            force: bool = False
                If force, don't bother checking if a tile is compatible with
                this vehicle's reservation before returning.
            mark: bool = False
                Whether to mark the tiles used with this potential reservation.
        """
        super().io_tile_buffer(lane, t, clone, reservation, prepend, force,
                               mark)

        # Recall that the first tile layer represents the next timestep.
        t0 = SHARED.t + 1
        to_return: Dict[int, Dict[Tile, float]] = {}
        if prepend:
            lane.movement_model.start_projection(clone, reservation.its_exit,
                                                 lane.speed_limit)

            if t == t0:
                return to_return

            p_prepend = lane.movement_model.prepend_probabilities(
                clone, reservation.its_exit, lane.speed_limit)
            if t - len(p_prepend) < t0:
                p_prepend = p_prepend[len(p_prepend) - (t - t0):]
                t_prepend = t0
            else:
                t_prepend = t-1

            while len(self.tiles) < t_prepend - SHARED.t:
                self._add_new_layer()

            tile_id = self.buffer_tile_loc[lane.trajectory.start_coord]
            for i, p in enumerate(p_prepend):
                t_res = t_prepend + i
                tile = self.tiles[t_res - t0][tile_id]
                if not tile.will_reservation_work(reservation, p):
                    lane.movement_model.clean_up_projection(clone)
                    return None
                to_return[t_res] = {tile: p}

        else:
            p_postpend = lane.movement_model.postpend_probabilities(
                clone, Tiling._exit_res_timesteps_forward(clone.velocity), t)

            while len(self.tiles) < t + len(p_postpend) - SHARED.t:
                self._add_new_layer()

            tile_id = self.buffer_tile_loc[lane.trajectory.end_coord]
            for i, p in enumerate(p_postpend):
                t_res = t + i + 1
                tile_t_index = t_res - t0
                tile = self.tiles[tile_t_index][tile_id]
                if not tile.will_reservation_work(reservation, p):
                    return None
                to_return[t_res] = {tile: p}

        return to_return

    def _tile_loc_to_id(self, tile_loc: Tuple[int, int]) -> int:
        """Convert a tile's (x,y) location to its 1D integer ID.

        The input parameter is a 2-tuple of integer x and y values. x denotes
        left to right and y denotes down to up.
        """
        return floor(tile_loc[0] + tile_loc[1] * self.x_tile_count)

    def find_best_batch(self,
                        requests: Dict[RoadLane, List[Reservation]]
                        ) -> List[Tuple[RoadLane, Reservation]]:
        """Take potential reservation permutations and find the best batch.

        Take a list of potential reservations from each road lane with a
        reservation. For each road lane, the reservations are listed in order
        of priority, i.e., the second reservation in a RoadLane's list follows
        the first's reservation and so on.

        Run through these road lanes' reservations and return the highest value
        compatible subset of reservations by sum total VOT and the road lane
        they originate from (for use in spacing out road lane exits).
        """

        # TODO: (auction) Sort through the requests from each RoadLane and
        #       find the maximum value combination. Remember that we can't
        #       selectively reject reservations from a RoadLane; instead, we
        #       have to accept the first n consecutive reservations. If we
        #       accept less than the full list of reservations from a lane,
        #       remember to decouple the dependency from the last accepted
        #       reservation.

        raise NotImplementedError("TODO")

    def _add_new_layer(self) -> None:
        """Extend the tiling stack by one timestep.

        At this point in the cycle, everything is fully resolved at timestep
        SHARED.t, so the first layer in the tile stack (index 0) represents
        timestep SHARED.t+1. Adding a new layer to the stack means that
        the new layer represents the first timestep after the current stack's
        coverage.
        """
        new_timestep = SHARED.t + 1 + len(self.tiles)
        self.tiles.append(tuple([
            self.tile_type(self._tile_loc_to_id((x, y)), new_timestep,
                           threshold=self.threshold)
            for y in range(self.y_tile_count)
            for x in range(self.x_tile_count)]))

    def _io_coord_to_tile_id(self, coord: Coord) -> int:
        """Convert a raw Coord to tile space's 1D index.

        Assumes that the provided Coord is in the tile space. This is the only
        case where a coord on the top or right borders is mapped to a tile;
        otherwise the math doesn't work out.
        """
        x: int = (coord.x - self.origin.x)//self.tile_width
        y: int = (coord.y - self.origin.y)//self.tile_width
        if x == self.x_tile_count:
            x -= 1
        if y == self.y_tile_count:
            y -= 1
        return self._tile_loc_to_id((x, y))
