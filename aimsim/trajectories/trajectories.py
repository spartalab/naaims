"""
A trajectory converts proportional progress along a trajectory to real (x,y)
coordinates. Lanes are initialized with a trajectory that determines how they
convert vehicle progress from in-lane logic to physical progress and vice
versa.

The `trajectories` module implements several different ways of doing so.
"""

from collections import deque
from abc import ABC, abstractmethod
from math import acos, sqrt
from typing import TYPE_CHECKING, Tuple, Optional

import bezier
import numpy as np
import scipy.optimize

from ..util import Coord
from ..vehicles import Vehicle


class Trajectory:

    def __init__(self, length: float,
                 start_coord: Coord,
                 end_coord: Coord) -> None:
        self.length = length
        self.start_coord = start_coord
        self.end_coord = end_coord

    @abstractmethod
    def get_position(self, proportion: float) -> Coord:
        raise NotImplementedError('Must be implemented in child classes.')

    @abstractmethod
    def get_proportion(self, position: Coord) -> float:
        raise NotImplementedError('Must be implemented in child classes.')

    @abstractmethod
    def heading(self, proportion: float, eps: float = 1e-6) -> float:
        """Returns the angle that a vehicle at proportion is heading in."""

        if proportion >= 1:
            a = self.get_position(proportion - eps)
            b = self.get_position(proportion)
        else:
            a = self.get_position(proportion)
            b = self.get_position(proportion + eps)

        # TODO: check this math
        dx = b.x - a.x
        dy = b.y - a.y
        return acos(dx / sqrt(dx**2 + dy**2))

    def curvature(self, proportion: float) -> Optional[float]:
        """May return the trajectory's angle of curvature at this point.

        If this trajectory supports this feature, this will return the
        trajectory's angle of curvature at some proportion, which could be
        used in turn slowdown calculations.

        If this trajectory does not support this feature, it will return None.
        """
        return None

    @abstractmethod
    def progress(self, vehicle: Vehicle, proportion: float) -> float:
        """Given these vehicle parameters, how far does the it progress?

        Can be overridden in child classes to account for curvature.
        """

        # TODO: let the trajectory do the progression calculations, allowing
        #       for override if fancier trajectories want to account for
        #       slowdowns on curved roads
        raise NotImplementedError("TODO")


class Bezier(Trajectory):
    def __init__(self, x0, y0, x1, y1, x2, y2):
        super().__init__()
        self.p0 = (x0, y0)
        self.p1 = (x1, y1)
        self.p2 = (x2, y2)
        self.speed_limit = 0
        nodes1 = np.asfortranarray([
            [x0, x1, x2],
            [y0, y1, y2],
        ])
        self.c1 = bezier.curve.Curve(nodes1, degree=2)

    def get_x(self, t):
        return ((1 - t) ** 2 * self.p0[0]
                + 2 * t * (1 - t) * self.p1[0] + t ** 2 * self.p2[0])

    def get_y(self, t):
        return ((1 - t) ** 2 * self.p0[1]
                + 2 * t * (1 - t) * self.p1[1] + t ** 2 * self.p2[1])

    def get_position(self, t):
        return np.asarray([((1-t)**2 * self.p0[0]
                            + 2*t*(1-t) * self.p1[0] + t**2 * self.p2[0]),
                           ((1 - t) ** 2 * self.p0[1]
                            + 2 * t * (1 - t) * self.p1[1]
                            + t ** 2 * self.p2[1])])

    def get_derivative(self, t):
        """Computes dy/dx of curve given specific time

        :param t: proportion along the curve
        :return: derivative
        """
        dxdt = (2 - 2*t)*(self.p1[0] - self.p0[0]) + \
            (2*t)*(self.p2[0] - self.p1[0])
        dydt = (2 - 2*t)*(self.p1[1] - self.p0[1]) + \
            (2*t)*(self.p2[1] - self.p1[1])
        return dydt/dxdt

    def get_curve(self):
        return self.c1

    def get_intersection_point(self, other):
        """Computes intersection point between two bezier curves

        Computes intersection point between two bezier curves by minimizing the
        least squares between the x and y values of the function

        :param other: other curve
        :return: t1, t2, point where t1 is time at which curve 1 reaches
                 intersection, t2 is time at which curve 2 reaches
                 intersection, and point is actual coordinates of intersection
                 Returns (None,None,None) if no intersection point is found
        """
        def compute_dif(inp):
            return (self.get_x(inp[0]) - other.get_x(inp[1]))**2 + (self.get_y(inp[0]) - other.get_y(inp[1]))**2
        res = scipy.optimize.minimize(compute_dif, [0.5, 0.5])
        if not res.success or compute_dif(res.x) >= 1e-4:
            return None, None, None
        x = res.x
        return x[0], x[1], [self.get_x(x[0]), self.get_y(x[0])]

    def get_intersection_angle(self, other, time1, time2):
        """
        Computes the intersection angle of the curves given the the two points
        :param other: reference to other cuve
        :param time1: time for current curve
        :param time2: time for other curve
        :return: angle of intersection in radians
        """
        c1prime = self.get_derivative(time1)
        c2prime = other.get_derivative(time2)
        return np.arctan((c2prime-c1prime)/(1 + c1prime * c2prime))
