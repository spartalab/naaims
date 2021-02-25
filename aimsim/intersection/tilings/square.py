from __future__ import annotations
from typing import TYPE_CHECKING, Optional, Set, Dict, Tuple, Type, List
from math import ceil

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
                 granularity: Optional[int] = None  # in meters
                 ) -> None:
        """Creates a new square (pixelated) tiling.

        Additional parameters:
            granularity: Optional[int]
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

        if granularity is None:
            raise ValueError("granularity can't be None for SquareTiling.")
        self.granularity = granularity

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
        self.x_tile_count = ceil((self.max_x - self.min_x)/self.granularity)
        self.y_tile_count = ceil((self.max_y - self.min_y)/self.granularity)

        # TODO: (square) Use the granularity input and the start and end points
        #       of every IntersectionLane to create a grid of tiles.
        raise NotImplementedError("TODO")

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
                     clone: Vehicle, progress: VehicleProgress,
                     reservation: Reservation,
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
        super().pos_to_tiles(lane, t, clone, progress, reservation, force,
                             mark)
        raise NotImplementedError("TODO")

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
        if force and mark:
            raise ValueError("Can't force and mark tiles at the same time.")
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
        """Extend the tiling stack by one timestep."""
        # TODO: Is self.tiles[0] the current timestep or the next timestep?
        self.tiles.append(
            tuple([
                tuple([self.tile_type(yi*self.y_tile_count + xi,
                                      SHARED.t + len(self.tiles))
                       for xi in range(self.x_tile_count)])
                for yi in range(self.y_tile_count)])
        )
