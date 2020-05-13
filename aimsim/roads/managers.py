'''
The lane change manager schedules and executes vehicle lane changes between
intersections.

Lane changes are modeled as a single vehicle occupying two parallel links
simultaneously.
'''

from abc import ABC

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


class LaneChangeManager(ABC):
    raise NotImplementedError("TODO")


class DummyManager(LaneChangeManager):
    raise NotImplementedError("TODO")
