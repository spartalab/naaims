"""
This module defines several intersection priority managers based on different
priority policies.
"""


from __future__ import annotations
from abc import abstractmethod
from typing import TYPE_CHECKING, Dict, Tuple, Type, Any, TypeVar

from aimsim.archetypes import Configurable
from aimsim.util import Coord
from aimsim.intersection.tilings import SquareTiling, ArcTiling

if TYPE_CHECKING:
    from aimsim.road import RoadLane
    from aimsim.intersection import IntersectionLane
    from aimsim.intersection.tilings import Tiling
    from aimsim.vehicles import Vehicle

M = TypeVar('M', bound='IntersectionManager')


class IntersectionManager(Configurable):
    """
    The manager handles whether vehicles are allowed to ask to enter the
    intersection in some future timestep and, if so, in what order permission
    is granted (either in the form of a reservation for AIM policies or simple
    permission_to_enter_intersection for traffic signal managers). Subclasses
    of this abstract IntersectionManager base class implement concrete priority
    policies for how permission is granted.

    The manager's tiling handles the actual management of traffic signal cycle
    and confirmed reservations, collecting and verifying if reservation
    requests are feasible.
    """

    def __init__(self,
                 incoming_road_lane_by_coord: Dict[Coord, RoadLane],
                 outgoing_road_lane_by_coord: Dict[Coord, RoadLane],
                 lanes: Tuple[IntersectionLane, ...],
                 lanes_by_endpoints: Dict[Tuple[Coord, Coord],
                                          IntersectionLane],
                 tiling_type: Type[Tiling],
                 tiling_spec: Dict[str, Any]
                 ) -> None:
        """Create a new IntersectionManager.

        For the most part, subclasses don't need to extend the features defined
        in this init. If they do need to, subclasses must call this for initial
        setup, then continue in their own init to set up whatever they need to.
        """
        self.incoming_road_lane_by_coord = incoming_road_lane_by_coord
        self.outgoing_road_lane_by_coord = outgoing_road_lane_by_coord
        self.lanes = lanes
        self.lanes_by_endpoints = lanes_by_endpoints

        # Finish the spec for the manager by providing the roads and lanes
        tiling_spec['incoming_road_lane_by_coord'
                    ] = self.incoming_road_lane_by_coord
        tiling_spec['outgoing_road_lane_by_coord'
                    ] = self.outgoing_road_lane_by_coord
        tiling_spec['lanes'] = self.lanes
        tiling_spec['lanes_by_endpoints'] = self.lanes_by_endpoints

        # Create the tiling
        self.tiling = tiling_type.from_spec(tiling_spec)

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a manager spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: (spec) Interpret the string into the spec dict. cycle
        #       especially will be quite complicated!
        raise NotImplementedError("TODO")

        # TODO: (spec) Enforce provision of separate tiling_type and
        #       tiling_config fields in intersection spec string.
        tiling_type: str
        # Based on the spec, identify the correct tiling type
        if tiling_type.lower() in {'square', 'squaretiling'}:
            spec['tiling_type'] = SquareTiling
        elif tiling_type.lower() in {'arc', 'arctiling'}:
            spec['tiling_type'] = ArcTiling
        else:
            raise ValueError("Unsupported Tiling type.")

        return spec

    @classmethod
    def from_spec(cls: Type[M], spec: Dict[str, Any]) -> M:
        """Should interpret a spec dict to call the manager's init."""
        return cls(
            incoming_road_lane_by_coord=spec['incoming_road_lane_by_coord'],
            outgoing_road_lane_by_coord=spec[
                'outgoing_road_lane_by_coord'],
            lanes=spec['lanes'],
            lanes_by_endpoints=spec['lanes_by_endpoints'],
            tiling_type=spec['tiling_type'],
            tiling_spec=spec['tiling_spec']
        )

    # Begin simulation cycle methods

    def update_schedule(self) -> None:
        """Update tiling, reservations, and poll for new requests to accept."""

        # First update the tiling for the new timestep.
        self.tiling.handle_new_timestep()

        # Then decide whether to poll for new reservation requests and, if so,
        # which ones to accept.
        self.process_requests()

    @abstractmethod
    def process_requests(self) -> None:
        """Should update the request queue and check for requests to accept.

        Check every incoming road for vehicles or platoons that are eligible to
        try and make a reservation (i.e., lane leaders). Accept, reject, or
        hold onto their requests for a future step based on logic in this
        function.

        This function is the core of the intersection manager. Its priorities
        are reflected in this function.
        """
        raise NotImplementedError("Should be implemented in child classes.")

    def start_reservation(self, vehicle: Vehicle) -> IntersectionLane:
        """Start a reservation in the tiling and return its lane."""
        return self.tiling.start_reservation(vehicle)

    def finish_exiting(self, vehicle: Vehicle) -> None:
        """Clean up permissions and reservations for exiting vehicle."""
        vehicle.permission_to_enter_intersection = False
        if vehicle.has_reservation:
            self.tiling.clear_reservation(vehicle)
            vehicle.has_reservation = False

    # # Begin helper methods

    # def check_empty(self) -> bool:
    #     """Check if this intersection is empty."""
    #     for lane in self.lanes:
    #         if len(lane.vehicles) > 0:
    #             return False
    #     return True
