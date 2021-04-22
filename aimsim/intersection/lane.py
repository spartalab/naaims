from __future__ import annotations
from copy import copy
from typing import TYPE_CHECKING, Tuple, Optional, Dict, TypeVar
from math import pi

import aimsim.shared as SHARED
from aimsim.lane import Lane
from aimsim.util import VehicleSection
from aimsim.trajectories import BezierTrajectory

if TYPE_CHECKING:
    from aimsim.road import RoadLane
    from aimsim.vehicles import Vehicle

L = TypeVar('L', bound='IntersectionLane')


class IntersectionLane(Lane):
    """
    Unlike RoadLanes, movement on IntersectionLanes may be stochastic.
    Depending on their throttle and tracking score properties, vehicles can be
    faster or slower than instructed and deviate laterally without being
    instructed to by a lane change manager.
    """

    def __init__(self,
                 start_lane: RoadLane,
                 end_lane: RoadLane,
                 speed_limit: int):
        """Create a new IntersectionLane.

        Instead of taking an explicit trajectory, the Intersection Lane infers
        its own using the Coords of the start and end road lanes and their
        headings at their start and end to create a BezierTrajectory that
        curves nicely.
        """

        self.upstream = start_lane
        self.downstream = end_lane

        trajectory = BezierTrajectory.as_intersection_connector(
            start_coord=self.upstream.trajectory.end_coord,
            start_heading=self.upstream.trajectory.get_heading(1),
            end_coord=self.downstream.trajectory.start_coord,
            end_heading=(self.downstream.trajectory.get_heading(0) + pi) % 2*pi
        )  # Flip the downstream heading so it's pointing into the intersection
        super().__init__(trajectory, min(start_lane.width, end_lane.width),
                         speed_limit)

        # Calculate the shortest amount of time (in timesteps) that it takes
        # for a vehicle to fully travel across this lane.
        self.min_traversal_time = (trajectory.length/speed_limit *
                                   SHARED.SETTINGS.steps_per_second)

        # Track vehicles' lateral deviation from centerline in meters
        self.lateral_deviation: Dict[Vehicle, float] = {}

    # Support functions for speed updates

    def controls_this_speed(self, vehicle: Vehicle) -> Tuple[bool, float,
                                                             VehicleSection]:
        """Return the vehicle's controlling section and progress."""
        front = self.vehicle_progress[vehicle].front
        if front is not None:
            # Front of the vehicle is in the lane.
            return True, front, VehicleSection.FRONT
        else:
            # Surely the rear of the vehicle must be in the lane.
            rear = self.vehicle_progress[vehicle].rear
            if rear is None:
                raise RuntimeError("Vehicle exited lane already.")
            return True, rear, VehicleSection.REAR

    def effective_speed_limit(self, p: float, vehicle: Vehicle) -> float:
        """Return the effective speed limit at the given progression."""
        return super().effective_speed_limit(p, vehicle)

        # TODO: (stochasticity) Consider making the effective speed limit a
        #       function of a vehicle's ability to follow instructions
        #       precisely (to account for it over-accelerating) and its
        #       deviation from the lane's centerline to allow it to hit the
        #       actual speed limit if it cuts the corner.
        #
        #       See the get_new_speeds todo for more information.

    def accel_update(self, vehicle: Vehicle, section: VehicleSection, p: float,
                     preceding: Optional[Vehicle]) -> float:
        """Return a vehicle's new acceleration.

        Just calls the parent's accel_update() (for now).
        """

        # TODO: (platoon) Insert vehicle chain override here.
        #       May not be necessary if the emergent behavior of breaking
        #       chains before they start results in vehicles simply following
        #       normal reservation behavior with chained accelerations.

        return super().accel_update(vehicle, section, p, preceding)

        # TODO: (stochasticity) Change the return so the speed and acceleration
        #       update is affected by both:
        #           1. A stochastic term dependent on the vehicle's reliability
        #              in following speed updates. They may exceed the actual
        #              speed limit if their acceleration control is poor.
        #           2. What side of the trajectory the vehicle is on and how
        #              far from the centerline the vehicle is laterally.
        #              Vehicles that cut corners at the speed limit will have a
        #              higher effective speed limit in-lane, and vehicles that
        #              turn too loose will have a slower effective speed limit.
        #       This would require tweaking the effective speed limit for the
        #       specific vehicle in addition to this acceleration update, since
        #       in the velocity update the effective speed limit trims the
        #       acceleration to the speed limit if it exceeds the speed limit.

    def downstream_stopping_distance(self) -> Optional[float]:
        """Check the downstream road lane's required stopping distance."""
        return self.downstream.stopping_distance_to_last_vehicle()

    # Support functions for step updates

    def remove_vehicle(self, vehicle: Vehicle) -> None:
        """Remove records for this vehicle from this trajectory."""

        # Do the normal stuff
        super().remove_vehicle(vehicle)

        # Delete the vehicle's record from our lateral deviation tracker.
        del self.lateral_deviation[vehicle]

    def lateral_deviation_for(self, vehicle: Vehicle,
                              new_progress: float) -> float:
        """Calculate and return the lateral deviation of this vehicle."""

        # TODO: (stochastic) Retrieve self.lateral_deviation[vehicle] and
        #       calculate the new deviation in this timestep. Add them
        #       together, log it back to self.lateral_deviation[vehicle],
        #       and return it.

        return 0

    # Support functions for vehicle transfers

    def add_vehicle(self, vehicle: Vehicle) -> None:
        """Create entries for this vehicle in lane support structures.

        On top of what the parent add_vehicle() does, creates a new
        lateral_deviation entry for this vehicle.
        """
        super().add_vehicle(vehicle)
        self.lateral_deviation[vehicle] = 0

    # Misc functions

    def clone(self: L) -> L:
        """Return a copy of the lane with vehicle-specific data removed."""
        clone = copy(self)
        clone.vehicles = []
        clone.vehicle_progress = {}
        clone.lateral_deviation = {}
        return clone
