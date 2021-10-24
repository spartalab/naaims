from __future__ import annotations
from copy import copy
from typing import TYPE_CHECKING, Tuple, Optional, Dict, TypeVar, List, Type
from math import ceil, sqrt

import naaims.shared as SHARED
from naaims.lane import Lane, ScheduledExit, LateralDeviation
from naaims.util import (VehicleSection, VehicleTransfer, t_to_v,
                         x_over_constant_a)
from naaims.trajectories import BezierTrajectory
from naaims.intersection.movement import (MovementModel,
                                          DeterministicModel)

if TYPE_CHECKING:
    from naaims.road import RoadLane
    from naaims.vehicles import Vehicle

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
                 speed_limit: int,
                 movement_model: Type[MovementModel] = DeterministicModel
                 ) -> None:
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
            end_heading=self.downstream.trajectory.get_heading(0)
        )
        super().__init__(trajectory, min(start_lane.width, end_lane.width),
                         speed_limit)
        assert isinstance(self.trajectory, BezierTrajectory)
        self.trajectory: BezierTrajectory

        # Calculate the shortest amount of time (in timesteps) that it takes
        # for a vehicle to fully travel across this lane.
        self.min_traversal_time = (trajectory.length/speed_limit *
                                   SHARED.SETTINGS.steps_per_second)

        # Initialize model of stochastic vehicle movement
        self.movement_model = movement_model(trajectory)

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
        rear = self.vehicle_progress[vehicle].rear
        if rear is not None:
            # Surely the rear of the vehicle must be in the lane.
            return True, rear, VehicleSection.REAR
        center = self.vehicle_progress[vehicle].center
        if center is None:
            raise RuntimeError("Vehicle exited lane already.")
        return True, center, VehicleSection.CENTER

    def effective_speed_limit(self, p: float, vehicle: Vehicle) -> float:
        """Return the effective speed limit at the given progression."""
        return super().effective_speed_limit(p, vehicle)

        # TODO: (stochasticity+) Consider making the effective speed limit a
        #       function of a vehicle's ability to follow instructions
        #       precisely (to account for it over-accelerating) and its
        #       deviation from the lane's centerline to allow it to hit the
        #       actual speed limit if it cuts the corner.
        #
        #       See the get_new_speeds todo for more information.

    def accel_update(self, vehicle: Vehicle, section: VehicleSection, p: float,
                     preceding: Optional[Vehicle]) -> float:
        """Return a vehicle's new acceleration.

        NAAIMS assumes that, in an intersection, vehicles will always move as
        fast as possible, i.e., accelerate to the speed limit and then stay
        there while any part of it is in the intersection. No braking or
        staying at any speed under the speed limit.

        The only exception to this rule would be due to stochastic deviations,
        where vehicles would be slightly faster or slower to reach the speed
        limit depending on their ability to follow their instructed
        acceleration profile.
        """

        # TODO: (platoon) Insert vehicle chain override here.
        #       May not be necessary if the emergent behavior of breaking
        #       chains before they start results in vehicles simply following
        #       normal reservation behavior with chained accelerations.

        if (vehicle.throttle_mn > 0) or (vehicle.throttle_sd > 0):
            return self.movement_model.fetch_throttle_deviation(
                vehicle, section, p)
        return super().accel_update_uncontested(vehicle, p)

        # TODO: (stochasticity+) Change so speed and acceleration can be
        # affected by the extent and location of its lateral deviation.

    def downstream_stopping_distance(self) -> Optional[float]:
        """Check the downstream road lane's required stopping distance."""
        return self.downstream.stopping_distance_to_last_vehicle()

    # Support functions for step updates

    def step_vehicles(self,
                      lateral_deviations: Dict[Vehicle, LateralDeviation] = {}
                      ) -> List[VehicleTransfer]:
        if len(lateral_deviations) > 0:
            raise ValueError("IntersectionLane should not have externally "
                             "calculated lateral deviations.")
        return super().step_vehicles()

    def lateral_deviation_for(self, vehicle: Vehicle,
                              new_progress: float) -> float:
        """Return the lateral movement for a vehicle."""
        self.lateral_deviation[vehicle] = \
            self.movement_model.fetch_lateral_deviation(vehicle,
                                                        new_progress)
        return self.lateral_deviation[vehicle]

    def remove_vehicle(self, vehicle: Vehicle) -> None:
        """Remove records for this vehicle from this trajectory.

        On top of what the parent remove_vehicle() does, deletes the vehicle
        from IntersectionLane support structures."""
        super().remove_vehicle(vehicle)
        del self.lateral_deviation[vehicle]
        self.movement_model.remove_vehicle(vehicle)

    # Support functions for vehicle transfers

    def enter_vehicle_section(self, transfer: VehicleTransfer) -> None:
        if transfer.section is VehicleSection.FRONT:
            self.movement_model.init_throttle_deviation(
                transfer.vehicle, transfer, self.speed_limit)
        elif transfer.section is VehicleSection.CENTER:
            self.movement_model.init_lateral_deviation(transfer.vehicle)
        return super().enter_vehicle_section(transfer)

    def add_vehicle(self, vehicle: Vehicle) -> None:
        """Create entries for this vehicle in lane support structures.

        On top of what the parent add_vehicle() does, creates a new
        lateral_deviation entry for this vehicle.
        """
        super().add_vehicle(vehicle)
        self.lateral_deviation[vehicle] = 0

    # Support functions for reservation logic

    def rear_exit(self, front_exit: ScheduledExit, entire_lane: bool = False
                  ) -> ScheduledExit:
        """Given a vehicle's front exit, return its rear exit.

        If entire_lane is True, return the exit from the entire intersection
        lane. If False, return its exit just from the road lane.

        As with all in-intersection behavior, assumes that the vehicle will
        accelerate to the speed limit and then stay there while any part of it
        is in the intersection. No braking or constant sub-limit speed allowed!
        """
        # Recall that a vehicle's effective length is its actual length plus
        # the fixed buffer lengths before and after it.
        if front_exit.section is not VehicleSection.FRONT:
            raise ValueError('Not a front exit.')
        x = front_exit.vehicle.length * \
            (1 + 2*SHARED.SETTINGS.length_buffer_factor)
        if self.trajectory.length < x:
            raise RuntimeError("Vehicle (plus buffer) longer than lane.")
        if entire_lane:
            # From front entrance to rear exit is two car lengths plus the
            # length of the entire intersection lane.
            x += x + self.trajectory.length
        a = SHARED.SETTINGS.min_acceleration
        v0 = front_exit.velocity
        v_full_accel = sqrt(v0**2 + 2*a*x)
        v_max = self.speed_limit
        t: int = front_exit.t
        v: float
        if v_full_accel <= v_max:
            # Accelerating its entire length still won't reach the speed limit.
            t += ceil(t_to_v(v0, a, v_full_accel))
            v = v_full_accel
        else:
            # It hits the speed limit while transitioning.
            t_to_v_max = t_to_v(v0, a, v_max)
            x_to_v_max = x_over_constant_a(v0, a, t_to_v_max)
            t_at_v_max = (x - x_to_v_max)/v_max
            t += ceil(t_to_v_max + t_at_v_max)
            v = v_max
        return ScheduledExit(front_exit.vehicle, VehicleSection.REAR, t, v)

    # Misc functions

    def clone(self: L) -> L:
        """Return a copy of the lane with vehicle-specific data removed.

        Also sets the MovementModel to a variant intended for use in
        tiling.check_request().
        """
        clone = copy(self)
        clone.vehicles = []
        clone.vehicle_progress = {}
        clone.lateral_deviation = {}
        clone.movement_model = self.movement_model.reset_for_requests()
        return clone
