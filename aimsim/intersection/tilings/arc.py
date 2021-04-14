from __future__ import annotations
from typing import Any, TYPE_CHECKING, Optional, Set, Dict, Tuple, Type, List
from collections import deque

import aimsim.shared as SHARED
from aimsim.util import Coord
from aimsim.intersection.tilings.tiling import Tiling
from aimsim.intersection.tilings.tiles import DeterministicTile

if TYPE_CHECKING:
    from aimsim.road import RoadLane
    from aimsim.intersection.tilings.tiles import Tile
    from aimsim.intersection import IntersectionLane


class ArcTiling(Tiling):

    # TODO: (arc) Points at the edge of the intersection should be considered a
    #       conflict object even if they aren't literally a conflict point,
    #       so we can enforce spacing between vehicles.

    def __init__(self,
                 incoming_road_lane_by_coord: Dict[Coord, RoadLane],
                 outgoing_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Dict[Coord, IntersectionLane],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tile_type: Type[Tile] = DeterministicTile,
                 cycle: Optional[List[
                     Tuple[Set[IntersectionLane], int]
                 ]] = None,
                 misc_spec: Dict[str, Any] = {}
                 ) -> None:
        super().__init__(
            incoming_road_lane_by_coord=incoming_road_lane_by_coord,
            outgoing_road_lane_by_coord=outgoing_road_lane_by_coord,
            lanes=lanes,
            lanes_by_endpoints=lanes_by_endpoints,
            tile_type=tile_type,
            cycle=cycle
        )
        # TODO: (arc) Does ArcTiling need a buffer argument? Anything else?
        raise NotImplementedError("TODO")

        # TODO: (arc) Old code, fix or replace.
        # # find conflicts
        # for l1, l2 in itertools.combinations(self.intersection_lanes, 2):
        #     t1, t2, point = l1.trajectory.get_intersection_point(l2.trajectory)
        #     if point == None:
        #         continue
        #     angle = l1.trajectory.get_intersection_angle(l2.trajectory, t1, t2)
        #     self.conflicts.add(ConflictRegion(l1, l2, point, angle, t1, t2))

    # class ConflictRegion:

    #     def __init__(self, traj1, traj2, point, angle, t1, t2):
    #         self.point = point
    #         self.angle = angle
    #         self.traj1 = traj1
    #         self.traj2 = traj2
    #         self.t1 = t1
    #         self.t2 = t2

    #     def get_conflict_region(self, vehicle):
    #         pass
