'''
The lane change manager schedules and executes vehicle lane changes between
intersections.

Lane changes are modeled as a single vehicle occupying two parallel links
simultaneously.
'''

from __future__ import annotations
from abc import abstractmethod
from typing import TYPE_CHECKING, Iterable, Dict, Any, TypeVar, Type, Set

if TYPE_CHECKING:
    from aimsim.archetypes import Configurable
    from aimsim.lane import LateralDeviation
    from aimsim.road import RoadLane
    from aimsim.vehicles import Vehicle

# [Implementation notes]
# Helper function progresses vehicles normally along a path, either per vehicle or for all
# Uses only the prop progress of vehicle in lane and its length to calculate collision(ie ignore true position since it may be off to the side due to lane change)
# Vehicle speeds are it’s speed in lane, not diagonally if lane change
# Todo: how to calculate side to side speed

# One vehicle can occupy two lanes when lane changing, so helper should have a flag to not update vehicle pos. When vehicle is in two lanes, use helper for both lanes but only update pos according to the more conservative one by taking the values that come out of the helper function without using the helper function (lcm updates manually). Also, update priority queue prop value for both lanes separately and manually. Lcm has its own memory of each vehicle’s offset from which lane as reference.
# Lcmanager can also use the helper function, put one vehicle on two trajectories, and reduce the offset for a single vehicle

# Helper needs flag to not update veh pos and prio. It returns the update it would do, which is the max distance the veh can cover in this timestep. Lcm is free to slow veh down to let it merge.

# Lcm must iterate by prio across all lanes.

# Lcmanager is given self.lanes by road

M = TypeVar('M', bound='LaneChangeManager')


class LaneChangeManager(Configurable):

    @abstractmethod
    def __init__(self,
                 lanes: Iterable[RoadLane]
                 ) -> None:
        self.lanes = lanes
        raise NotImplementedError("TODO")

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a manager spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: (spec) Interpret the string into the spec dict.
        raise NotImplementedError("TODO")

        return spec

    @classmethod
    def from_spec(cls: Type[M], spec: Dict[str, Any]) -> M:
        """Should interpret a spec dict to call the manager's init."""
        return cls(
            lanes=spec['lanes']
        )

    @abstractmethod
    def vehicles_to_slow(self, lane: RoadLane) -> Set[Vehicle]:
        """Should return a list of vehicles to slow in the provided lane.

        In the last step, the manager should have calculated which vehicles
        in each lane it wants to override the lane-following behavior for and
        slow down to allow for a merge.
        """
        raise NotImplementedError("Must be implemented in child classes.")

    @abstractmethod
    def lateral_movements(self, lane: RoadLane) -> Dict[Vehicle,
                                                        LateralDeviation]:
        """Should return lateral movements for lane change region vehicles."""
        raise NotImplementedError("Must be implemented in child classes.")

    @abstractmethod
    def update_schedule(self) -> None:
        """Should process and schedule new and queued lane change requests."""
        raise NotImplementedError("Must be implemented in child classes.")
