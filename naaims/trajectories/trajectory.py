"""
A trajectory converts proportional progress along a trajectory to real (x,y)
coordinates. Lanes are initialized with a trajectory that determines how they
convert vehicle progress from in-lane logic to physical progress and vice
versa.

The `trajectories` module implements several different ways of doing so.
"""

from __future__ import annotations
from abc import abstractmethod
from math import atan2, pi, sqrt
from typing import List, Dict, Any

from naaims.archetypes import Configurable
from naaims.util import Coord


class Trajectory(Configurable):

    @abstractmethod
    def __init__(self,
                 start_coord: Coord,
                 end_coord: Coord,
                 reference_coords: List[Coord],
                 traversibility_factors: List[float] = []) -> None:
        """Create a new trajectory.

        Parameters
            start_coord: Coord
            end_coord: Coord
            reference_coords: List[Coord]
                A series of n coords that can be used for reference for any
                trajectory type. Preferred length depends on trajectory type.
            traversibility_factor: List[float]
                A series of floats between 0 and 1 that denote how difficult
                it is to traverse this trajectory. 1 is default. Perhaps
                singleton or (n-1).
        """
        self.start_coord = start_coord
        self.end_coord = end_coord
        self.reference_coords = reference_coords
        self.traversibility_factors = traversibility_factors

    @property
    @abstractmethod
    def length(self) -> float:
        raise NotImplementedError("Must be implemented in child classes.")

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a trajectory spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: (spec) Interpret the string into a spec dict.
        raise NotImplementedError("TODO")

        return spec

    @classmethod
    def from_spec(cls, spec: Dict[str, Any]) -> Trajectory:
        """Create a new Trajectory from the output of spec_from_str.

        Trajectories are static so they don't need post-processing between
        spec_from_str and this function, unlike other Configurables.
        """
        return cls(
            start_coord=spec['start_coord'],
            end_coord=spec['end_coord'],
            reference_coords=spec['reference_coords'],
            traversibility_factors=spec['traversibility_factors']
        )

    def clone_with_offset(self, offset: Coord) -> Trajectory:
        """Create a clone of this lane offset by some (x,y) distance."""
        return type(self)(
            start_coord=Coord(self.start_coord.x + offset.x,
                              self.start_coord.y + offset.y),
            end_coord=Coord(self.end_coord.x + offset.x,
                            self.end_coord.y + offset.y),
            reference_coords=[
                Coord(c.x + offset.x, c.y + offset.y)
                for c in self.reference_coords
            ],
            traversibility_factors=self.traversibility_factors
        )

    @abstractmethod
    def get_position(self, proportion: float) -> Coord:
        """Should return the Coord associated with a proportional progress."""
        raise NotImplementedError('Must be implemented in child classes.')

    def get_heading(self, proportion: float, eps: float = 1e-6) -> float:
        """Returns the forward moving angle in radians at proportion."""

        if proportion >= 1:
            a = self.get_position(proportion - eps)
            b = self.get_position(proportion)
        else:
            a = self.get_position(proportion)
            b = self.get_position(proportion + eps)

        dx = b.x - a.x
        dy = b.y - a.y
        angle = atan2(dy, dx)
        if angle < 0:
            angle += 2*pi
        return angle

    def effective_speed_limit(self, proportion: float) -> float:
        """Would use traversibility_factors to find an effective speed limit.

        Some trajectories (e.g., tight turns) require a certain speed to
        navigate properly or comfortably. This method, if implemented, would
        enforce a slower speed on different proportions of the trajectory.
        """
        # TODO: (low) implement this properly in child classes
        return float('inf')

    def __hash__(self) -> int:
        return hash(self.start_coord + self.end_coord)
