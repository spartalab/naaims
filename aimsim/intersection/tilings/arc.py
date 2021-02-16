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


class ArcTiling(Tiling):

    # TODO: (arc) Points at the edge of the intersection should be considered a
    #       conflict object even if they aren't literally a conflict point,
    #       so we can enforce spacing between vehicles.

    def __init__(self,
                 upstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 downstream_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Dict[Coord, IntersectionLane],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tile_type: Type[Tile] = Tile,
                 cycle: Optional[List[
                     Tuple[Set[IntersectionLane], int]
                 ]] = None
                 ) -> None:
        super().__init__(
            upstream_road_lane_by_coord=upstream_road_lane_by_coord,
            downstream_road_lane_by_coord=downstream_road_lane_by_coord,
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
