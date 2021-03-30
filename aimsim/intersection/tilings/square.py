from __future__ import annotations
from typing import TYPE_CHECKING, Optional, Set, Dict, Tuple, Type, List
from math import ceil, copysign, floor

import aimsim.shared as SHARED
from aimsim.util import Coord
from aimsim.intersection.tilings.tiling import Tiling
from aimsim.intersection.tilings.tiles import DeterministicTile

if TYPE_CHECKING:
    from aimsim.road import RoadLane
    from aimsim.intersection.tilings.tiles import Tile
    from aimsim.vehicles import Vehicle
    from aimsim.intersection import IntersectionLane
    from aimsim.intersection.reservation import Reservation
    from aimsim.lane import VehicleProgress


class SquareTiling(Tiling):
    def __init__(self,
                 upstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 downstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Dict[Coord, IntersectionLane],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tile_type: Type[Tile] = DeterministicTile,
                 cycle: Optional[List[
                     Tuple[Set[IntersectionLane], int]
                 ]] = None,
                 tile_width: Optional[int] = None  # in meters
                 ) -> None:
        """Creates a new square (pixelated) tiling.

        Additional parameters:
            tile_side_length: Optional[int]
                Only sort of optional. Dictates the size of each square tile,
                in meters.
        """
        super().__init__(
            upstream_road_lane_by_coord=upstream_road_lane_by_coord,
            downstream_road_lane_by_coord=downstream_road_lane_by_coord,
            lanes=lanes,
            lanes_by_endpoints=lanes_by_endpoints,
            tile_type=tile_type,
            cycle=cycle
        )

        if tile_width is None:
            raise ValueError("tile_width can't be None.")
        self.tile_width = tile_width

        # Identify bounding box by looking for min and max x and y coordinates
        # across all IntersectionLanes.
        self.min_x: float = float('inf')
        self.max_x: float = -float('inf')
        self.min_y: float = float('inf')
        self.max_y: float = -float('inf')
        for start, end in lanes_by_endpoints:
            for xy in (start, end):
                if xy.x < self.min_x:
                    self.min_x = xy.x
                if xy.x > self.max_x:
                    self.max_x = xy.x
                if xy.y < self.min_y:
                    self.min_y = xy.y
                if xy.y > self.max_y:
                    self.max_y = xy.y

        # # Tiles are inclusive of their bottom and left borders, so add one row
        # # or column of padding if the intersection's top or right edges fall on
        # # the top or right of the grid.
        # x_tile_count = (self.max_x - self.min_x)/self.tile_width
        # y_tile_count = (self.max_y - self.min_y)/self.tile_width
        # self.x_tile_count = ceil(x_tile_count) + \
        #     (1 if x_tile_count % 1 == 0 else 0)
        # self.y_tile_count = ceil(y_tile_count) + \
        #     (1 if y_tile_count % 1 == 0 else 0)

        x_tile_count = (self.max_x - self.min_x)/self.tile_width
        y_tile_count = (self.max_y - self.min_y)/self.tile_width
        self.x_tile_count = ceil(x_tile_count)
        self.y_tile_count = ceil(y_tile_count)

        # Tiles are inclusive of their bottom and left borders, so vehicles
        # going off-grid on the top or right borders must be specially
        # accommodated by pos_to_tile by having their shapes end just before
        # the top or right border.
        self.eps_length = self.tile_width*1e-3

        # Normalize the origin to be the bottom left Coord of this grid. The
        # Tiles will be stored left-to-right, top-to-bottom but the y-axis will
        # be flipped to match normal coordinate schemas.
        self.origin = Coord(self.min_x, self.min_y)

    def check_for_collisions(self) -> None:
        """Check for collisions in the intersection."""
        # TODO: (low) May only be necessary if we have stochastic movement.
        #       Otherwise collisions should not occur if implemented correctly.
        pass

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

        # Fetch the vehicle's outline, normalize them to the grid's internal
        # coordinate system using self.origin, and project it onto the grid.
        # TODO: (stochastic)? This can be replaced with any convex outline for
        #       the vehicle, e.g., the static and time buffers.
        outline = self._project_onto_grid(tuple(
            Coord((c.x - self.origin.x)/self.tile_width,
                  (c.y - self.origin.y)/self.tile_width)
            for c in clone.get_outline()))

        # Start by finding the y-range the outline covers.
        y_min: int = floor(outline[0].y)
        y_max: int = y_min
        for o in outline:
            oy = floor(o.y)
            if oy < y_min:
                y_min = oy
            if oy > y_max:
                y_max = oy
            # TODO: If tweaking bound check, cap at self.y_tile_count-1.
        # Initialize lists for the x-range associated with each y-value.
        x_mins: List[int] = [-1]*(y_max-y_min+1)
        x_maxes: List[int] = x_mins.copy()
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
                x_mins[j + y_min_seg - y_min] = x_min
            for j, x_max in enumerate(x_maxes_seg):
                x_maxes[j + y_min_seg - y_min] = x_max
            # TODO: (shapes) This must be changed for non-convex outlines.

        # After the outlining process is complete, loop through the min and max
        # y-Tiles via the x-bound lists. For each y-value, add every tile
        # between the min and max x-Tiles to the return set.
        tiles_covered: Dict[Tile, float] = {}
        for j in range(len(x_mins)):
            y = y_min + j
            for i in range(x_mins[j], x_maxes[j]+1):
                x = x_mins[j]+i
                tile = self.tiles[x][y]
                p = 1  # TODO: (stochastic) probabilistic reservation
                if tile.will_reservation_work(reservation, p):
                    tiles_covered[tile] = 1
                else:
                    return None

        return tiles_covered

    def _project_onto_grid(self, outline: Tuple[Coord]) -> Tuple[Coord]:
        """Given an outline, project it to be fully on the grid."""
        outline_projected: List[Coord] = []

        for i in range(len(outline)):
            start = outline[i]
            end = outline[i+1] if (i != len(outline)-1) else outline[0]
            dx = end.x - start.x
            dy = end.y - start.y
            m = dx/dy if dy != 0 else float('inf')
            b = end.y - m*end.x

            potential_start = self.__first_point_on_grid(start, end, m, b)
            if potential_start is None:
                # The entire line segment isn't on the grid. Go to the next
                # side of the shape.
                continue
            else:
                outline_projected.append(potential_start)

            # At least some part of the line segment, possibly the start point,
            # was on the grid, so we need to check if the end point is also. If
            # not, find the closest point to the segment end point that's on
            # both the line segment and grid.
            potential_end = self.__first_point_on_grid(end, start, m, b)
            # If we were able to find a potential start point, at worst it's
            # also the potential end point, so this should never return None.
            assert potential_end is not None
            if potential_end != potential_start:
                outline_projected.append(potential_end)
                # If the projected segment was reduced to a single point,
                # there's no need to add it to the outline twice.

        return tuple(outline_projected)

    def __first_point_on_grid(self, point: Coord, reference: Coord,
                              m: float, b: float) -> Optional[Coord]:
        """Return the Coord on line closest to point on the grid, if any.

        Trace along the line segment from point to reference with slope m and
        y-intercept b, until we reach the first point that's inside the grid.
        If there isn't a point on the line that's on the grid, return None.

        Parameters
            point: Coord
                The point on the line we want to find the closest grid and
                line segment point to.
            reference: Coord
                The other end of the line segment.
            m: float
                Line slope.
            b: float
                Line y-intercept.
        """

        # Handle special cases where the line is either horizontal or vertical
        # and completely out of the grid.
        if (  # dx=0, vertical line, and the fixed x is out of bounds
            (m == 0) and ((point.x < 0) or (point.x >= self.x_tile_count))
        ) or (  # dy=0, horizontal line, and the fixed y is out of bounds
            (m == float('inf')) and ((point.x < 0)
                                     or (point.x >= self.x_tile_count))
        ):
            return None
        # TODO: Fix if changing upper edge behavior.

        point_on_grid: bool = True
        x_closest: float
        y_closest: float
        # Find the x-value on the grid closest to the point.
        if point.x > self.x_tile_count - self.eps_length:
            point_on_grid = False
            x_closest = self.x_tile_count - self.eps_length
        elif point.x < 0:
            point_on_grid = False
            x_closest = 0
        else:
            x_closest = point.x
        # Find the y-value on the grid closest to the point.
        if point.y > self.y_tile_count - self.eps_length:
            point_on_grid = False
            y_closest = self.y_tile_count - self.eps_length
        elif point.y < 0:
            point_on_grid = False
            y_closest = 0
        else:
            y_closest = point.y
        # Note that because Tiles are inclusive of their lower and left borders
        # but not their upper and right borders, we need to do a bit of
        # correction when points are above (> self.y_tile_count) or to the
        # right (> self.x_tile_count) of the grid using self.eps_length.
        # TODO: Is this better than a simple boundary check?

        # We can return the point here if we've determined that it's on grid.
        if point_on_grid:
            return point

        # We've found two potential points on the grid that might be on the
        # line segment: the point where the study point's x-value was projected
        # onto the grid, and the point where the study point's y-value was.
        # Find their corresponding y- and x-value and distance to the point.
        x_of_y_closest = float('inf')
        y_of_x_closest = float('inf')
        dist_of_y_closest = float('inf')
        dist_of_x_closest = float('inf')
        dist_of_end = (point.x - reference.x)**2 + (point.y - reference.y)**2
        # We also need to check a few special cases:
        #   1. The line segment is horizontal or vertical and the start if off
        #      the grid.
        #       a. If m=0, the line segment is horizontal and y is fixed.
        #          y-closest is guaranteed to not be valid, so no need to
        #          consider it.
        #       b. If m=inf, the line segment is vertical and x is fixed.
        #          x-closest is guaranteed to not be valid, so no need to
        #          consider it.
        #   2. The projected point is off-segment, i.e., it goes the wrong way
        #      along the line to be on the line segment.
        x_min: float
        x_max: float
        y_min: float
        y_max: float
        if point.x < reference.x:
            x_min = point.x
            x_max = reference.x
        else:
            x_min = reference.x
            x_max = point.x
        if point.y < reference.y:
            y_min = point.y
            y_max = reference.y
        else:
            y_min = reference.y
            y_max = point.y
        if m != 0:  # Not horizontal, y varies. Safe to check y_closest.
            x_of_y_closest = (y_closest - b) / m if m != float('inf') \
                else point.x
            if (x_min <= x_of_y_closest <= x_max) and \
                    (y_min <= y_closest <= y_max):
                dist_of_y_closest = (point.x - x_of_y_closest)**2 + \
                    (point.y - y_closest)**2
        if m != float('inf'):  # Not vertical, x varies. Check x_closest.
            y_of_x_closest = m*x_closest + b if m != 0 else point.y
            if (x_min <= x_closest <= x_max) and \
                    (y_min <= y_of_x_closest <= y_max):
                dist_of_x_closest = (point.x - x_closest)**2 + \
                    (point.y - y_of_x_closest)**2

        # Use the distances to determine which projected point is the closest
        # grid point to the study point. If neither is closer to the study
        # point than the end of the line segment is, the entire segment is
        # off-grid so we return None.
        min_dist = min(dist_of_end, dist_of_x_closest, dist_of_y_closest)
        if min_dist == dist_of_x_closest:
            return Coord(x_closest, y_of_x_closest)
        elif min_dist == dist_of_y_closest:
            return Coord(x_of_y_closest, y_closest)
        else:
            return None

    def _line_to_tile_ranges(self, start: Coord, end: Coord) -> \
            Tuple[int, List[int], List[int]]:
        """Given two points on the grid, find the Tiles their connection hits.

        Assumes
            1. Both start and end Coords are on the grid.
            2. Line segments are being provided in clockwise order, to inform
               what sides of the tiles hit are being returned.

        Uses http://www.cse.yorku.ca/~amana/research/grid.pdf to find Tiles.

        Returns y_min, x_mins, x_maxes.
        """
        dx = end.x - start.x
        dy = end.y - start.y

        x_mins: List[int] = []
        x_maxes: List[int] = []
        y_min: int
        y_max: int

        # TODO: Alter flooring if changing to inclusive of top right borders.
        if dy == 0:  # horizontal or a single point
            y_min = y_max = floor(start.y)
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

            # Set up the line segment equation and bounds
            m = dx/dy
            b = end.y - m*end.x
            # def y_of_x(x: float) -> float: return m*x+b
            def x_of_y(y: float) -> float: return (y-b)/m
            x_dist_max = abs(end.x - start.x)

            # Find rasterization y-axis bounds, initialize rasterization
            # x-value bound lists, and find x_to_next_y_tile, how far along the
            # line we need to step from start in x-units to reach the next Tile
            # with a new (integer) y-value.
            step_y: int
            x_mins: List[int] = []
            x_maxes: List[int] = []
            if dy < 0:  # down right or left, maxes
                y_max = floor(start.y)
                y_min = floor(end.y)
                x_maxes: List[int] = [-1]*(y_max-y_min+1)
                step_y = -1
                x_of_next_y_tile = x_of_y(floor(start.y))
            else:  # up right or left, mins
                y_min = floor(start.y)
                y_max = floor(end.y)
                x_mins: List[int] = [-1]*(y_max-y_min)
                step_y = 1
                x_of_next_y_tile = x_of_y(ceil(start.y))
            x_to_next_y_tile: float = abs(start.x - x_of_next_y_tile)

            # Find x_to_next_x_tile, how far along the line we need to step
            # from start in x-units to reach the next Tile with a new x-value.
            x_to_next_x_tile: float = start.x % 1 if dx < 0 else 1-start.x % 1
            step_x: int = -1 if dx < 0 else 1
            x_delta_x: float = 1
            x_delta_y: float = abs(1/m)

            # Find Tile coordinates of the starting Tile and log them.
            x: int = floor(start.x)
            y: int = floor(start.y)
            x_mins[y - y_min] = x_maxes[y - y_min] = x
            # TODO: Consider adding special case checks at top or right border
            #       here and in the loop. Basically just, if x or y==x or y
            #       tile count, mark the tile at tile_count-1.

            # Traverse the line segment Tile-by-Tile in order from start to
            # end. Track if there's already been a Tile in this x-band marked;
            # if there has, new x-Tiles traversed need only be included in
            # x_mins or x_maxes if dx points in a direction that would make the
            # last x_min or x_max logged insufficiently lenient.
            marked: bool = False
            while True:  # break condition in loop

                # Find if the next x-tile or the next y-tile is closer. Given
                # that, update the x- or y-value to match that closer tile's
                # coordinates and calculate the x-distance of the next closest
                # tile in the same axis.
                if x_to_next_x_tile < x_to_next_y_tile:
                    if x_to_next_x_tile > x_dist_max:
                        break
                    x += step_x
                    x_to_next_x_tile += x_delta_x
                else:
                    if x_to_next_y_tile > x_dist_max:
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

    def edge_tile_buffer(self, lane: IntersectionLane, t: int,
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
        super().edge_tile_buffer(lane, t, clone, reservation, prepend, force,
                                 mark)

        raise NotImplementedError("TODO")

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
        SHARED.t, so the first layer in the tile stack (i.e., index 0)
        represents time SHARED.t+1.
        """
        new_timestep = SHARED.t + 1 + len(self.tiles)
        self.tiles.append(
            tuple([
                tuple([self.tile_type(yi*self.y_tile_count + xi,
                                      new_timestep)
                       for xi in range(self.x_tile_count)])
                for yi in range(self.y_tile_count)])
        )
