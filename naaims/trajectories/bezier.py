from __future__ import annotations
from math import isclose, pi, sqrt, tan, ceil
from typing import List, Optional

from naaims.util import Coord
from naaims.trajectories import Trajectory


class BezierTrajectory(Trajectory):
    def __init__(self,
                 start_coord: Coord,
                 end_coord: Coord,
                 reference_coords: List[Coord],
                 traversibility_factors: List[float] = []) -> None:
        """Create a new quadratic Bezier trajectory.

        See trajectory.py for more information.

        Parameters specific to BezierTrajectory
            reference_coords: Iterable[Coord]
                A series of n coords that can be used for reference for any
                trajectory type. Preferred length depends on trajectory type.
        """
        super().__init__(
            start_coord=start_coord,
            end_coord=end_coord,
            reference_coords=reference_coords,
            traversibility_factors=traversibility_factors
        )
        assert len(reference_coords) == 1
        self.control_coord: Coord = reference_coords[0]

        self._length: float = self.__find_length()
        self._straight: Optional[bool] = None

    @classmethod
    def as_intersection_connector(cls,
                                  start_coord: Coord,
                                  start_heading: float,
                                  end_coord: Coord,
                                  end_heading: float) -> BezierTrajectory:
        """Creates a Bezier trajectory for use in an IntersectionLane.

        Calculates the control coordinate and traversibility from the given
        parameters then passes them to the constructor.
        """

        if start_heading % 2*pi == (end_heading + pi) % 2*pi:
            raise ValueError("IO lanes are parallel.")

        # Find the intersection of the two headings.
        # First, check the special cases for if either heading is vertical.
        control_coord: Coord
        if isclose(start_heading, end_heading, abs_tol=1e-8):
            control_coord = Coord(
                start_coord.x + (end_coord.x - start_coord.x)/2,
                start_coord.y + (end_coord.y - start_coord.y)/2)
        elif start_heading % pi == pi/2:
            control_coord = Coord(start_coord.x,
                                  tan(end_heading)*(start_coord.x-end_coord.x
                                                    ) + end_coord.y)
        elif end_heading % pi == pi/2:
            control_coord = Coord(end_coord.x,
                                  tan(start_heading)*(end_coord.x-start_coord.x
                                                      ) + start_coord.y)
        else:
            # If they aren't vertical find the intersection per this link.
            # https://math.stackexchange.com/questions/1990698/intersection-of-two-lines-each-defined-by-a-point-and-an-angle
            m0 = tan(start_heading)
            m1 = tan(end_heading)
            x = ((m0*start_coord.x - m1*end_coord.x) -
                 (start_coord.y - end_coord.y)) / (m0 - m1)
            y = m0*(x-start_coord.x) + start_coord.y
            control_coord = Coord(x, y)

        # TODO (low): Calculate traversibility factors.

        return cls(start_coord, end_coord,
                   reference_coords=[control_coord],
                   traversibility_factors=[])

    @property
    def length(self) -> float:
        return self._length

    @staticmethod
    def __quadratic_bezier(p: float, start: float, control: float, end: float
                           ) -> float:
        """Return the (1D, quadratic) Bezier coordinate of a proportion."""
        return (1-p)*((1-p)*start+p*control) + p*((1-p)*control+p*end)

    def __find_length(self, delta: float = 0.001) -> float:
        total: float = 0.
        last_point = self.get_position(0)
        for i in range(1, ceil(1/delta)+1):
            increment = i*delta
            increment = 1 if increment > 1 else increment

            next_point = self.get_position(increment)
            total += sqrt((next_point.x - last_point.x)**2 +
                          (next_point.y - last_point.y)**2)
            last_point = next_point
        return total

    def get_position(self, proportion: float) -> Coord:
        """Return the Coord associated with a proportional progress."""

        return Coord(
            self.__quadratic_bezier(proportion,
                                    self.start_coord.x,
                                    self.control_coord.x,
                                    self.end_coord.x),
            self.__quadratic_bezier(proportion,
                                    self.start_coord.y,
                                    self.control_coord.y,
                                    self.end_coord.y)
        )

    @property
    def straight(self) -> bool:

        # TODO: (low) adapt for general bezier curves.
        if self._straight is None:
            dy = self.end_coord.y - self.start_coord.y
            dyc = self.reference_coords[0].y - self.start_coord.y

            dy_is_0 = isclose(dy, 0, abs_tol=1e-8)
            dyc_is_0 = isclose(dyc, 0, abs_tol=1e-8)
            if dy_is_0 and dyc_is_0:
                self._straight = True
            elif dy_is_0 ^ dyc_is_0:  # XOR
                self._straight = False
            else:
                dx = self.end_coord.x - self.start_coord.x
                dxc = self.reference_coords[0].x - self.start_coord.x

                m = dx/dy
                mc = dxc/dyc

                self._straight = isclose(m, mc, abs_tol=1e-8)

        return self._straight
