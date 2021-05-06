from __future__ import annotations
from typing import TYPE_CHECKING, Iterable, Dict, Tuple, Type, Any, TypeVar

from aimsim.archetypes import Configurable
from aimsim.util import Coord, VehicleSection
from aimsim.lane import ScheduledExit
from aimsim.vehicles import Vehicle
from aimsim.intersection.tilings import Tiling, SquareTiling, ArcTiling
from aimsim.intersection.reservation import Reservation
from aimsim.intersection import IntersectionLane
from aimsim.intersection.managers.manager import IntersectionManager

if TYPE_CHECKING:
    from aimsim.road import Road
    from aimsim.road import RoadLane


class SignalsManager(IntersectionManager):
    """
    A traffic signal priority policy, i.e., red and green lights.
    """

    def process_requests(self) -> None:
        # Poll for requests from green light lanes only. We only need to look
        # at each lane once.

        # Iterate through all incoming road lanes associated with at least one
        # greenlit movement/intersection lane.
        for lane, targets in self.tiling.greenlit.items():
            # Keep looking through the vehicles in this road lane until we
            # reach a vehicle we can't issue permission to (or there are no
            # vehicles # without permission left).
            while True:
                # Get the index of the first vehicle without permission to
                # enter that wants to go down one of the greenlit intersection
                # lanes, if there is one.
                seeking_perms = lane.first_without_permission(targets)
                if seeking_perms is None:
                    # The first vehicle does not want to use any of the green
                    # lanes. Move onto the next lane.
                    break
                else:
                    index: int = seeking_perms[0]

                # Check if the downstream lane has enough room for this vehicle
                vehicle: Vehicle = lane.vehicles[index]
                if vehicle.length > self.outgoing_road_lane_by_coord[
                    vehicle.next_movements(lane.trajectory.end_coord)[0]
                ].room_to_enter(tight=False):
                    break

                # Estimate this vehicle's exit parameters and use those to see
                # if this exit gives the vehicle enough time to clear the
                # intersection without colliding with the vehicle ahead of it.
                # (I don't think we can do that fully accurately without
                # simulating their entire movements, but we can at least get an
                # approximation.)
                this_exit = lane.soonest_exit(index)
                if this_exit is None:
                    # It can't exit its RoadLane in a valid way. Move on.
                    break
                assert self.tiling.cycle is not None
                move_time: int
                # TODO: (signals) Find how long it'll take for the vehicle to
                #       cross the IntersectionLane at max acceleration to the
                #       speed limit and add a timestep of padding.
                estimated_time_to_finish: int
                # TODO: (signals) Estimate when it will itself finish exiting
                #       by assuming the vehicle continues to accelerate or stay
                #       at the speed limit as it crosses its own length into
                #       the downstream road.
                if move_time <= self.tiling.time_left_in_cycle:
                    self.tiling.issue_permission(
                        vehicle, lane, ScheduledExit(
                            vehicle=this_exit.vehicle,
                            section=VehicleSection.REAR,
                            t=this_exit.t+estimated_time_to_finish,
                            velocity=this_exit.velocity))
