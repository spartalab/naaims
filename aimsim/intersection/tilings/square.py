from abc import abstractmethod
from typing import (Optional, Iterable, Set, Dict, Tuple, Type, TypeVar, Any,
                    List)
from collections import deque

import aimsim.shared as SHARED
from aimsim.archetypes import Configurable
from aimsim.util import Coord, SpeedUpdate, VehicleSection
from aimsim.lane import VehicleProgress, ScheduledExit
from aimsim.vehicles import Vehicle
from aimsim.intersection.reservation import Reservation
from aimsim.intersection.tiles import Tile, DeterministicTile
from aimsim.intersection.tilings.tiling import Tiling
from aimsim.intersection import IntersectionLane
from aimsim.road import RoadLane


class SquareTiling(Tiling):
    def __init__(self,
                 upstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 downstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Dict[Coord, IntersectionLane],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tile_type: Type[Tile] = Tile,
                 cycle: Optional[List[
                     Tuple[Set[IntersectionLane], int]
                 ]] = None,
                 granularity: Optional[int] = None
                 ) -> None:
        super().__init__(
            upstream_road_lane_by_coord=upstream_road_lane_by_coord,
            downstream_road_lane_by_coord=downstream_road_lane_by_coord,
            lanes=lanes,
            lanes_by_endpoints=lanes_by_endpoints,
            tile_type=tile_type,
            cycle=cycle
        )
        # TODO: (square) Use the granularity input and the start and end points
        #       of every IntersectionLane to create a grid of tiles.
        raise NotImplementedError("TODO")
