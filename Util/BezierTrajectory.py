from Util.AbstractTrajectory import AbstractTrajectory
import numpy as np
import bezier
import scipy.optimize


class BezierTrajectory(AbstractTrajectory):
    def __init__(self, x0, y0, x1, y1, x2, y2):
        super().__init__()
        self.p0 = (x0,y0)
        self.p1 = (x1,y1)
        self.p2 = (x2,y2)
        self.speed_limit = 0
        nodes1 = np.asfortranarray([
            [x0, x1, x2],
            [y0, y1, y2],
        ])
        self.c1 = bezier.curve.Curve(nodes1, degree=2)

    def progress_time(self):
        # for each vehicle at completion percentage, see if we can move it forward
        # If head vehicle can get a reservation given an approximate arrival time then
        # we should remove the vehicle from the dictionary

        timestep = 5.0
        for x in range(len(self.vehicles)):
            # x_vehicle is the current vehicle
            # b_vehicle is the vehicle that is ahead of the x_vehicle
            # timestep is the time step
            i_vehicle = self.vehicles[x]

            i_vehicle.a = i_vehicle.a_max if i_vehicle.v < self.speed_limit else 0
            i_vehicle.v = min(i_vehicle.v + i_vehicle.a*timestep, self.speed_limit)

            i_vehicle.p += (i_vehicle.v * timestep) / self.length
            #TODO: keep track of the (x,y) coordinates
            if x == 0 and i_vehicle.p >= 1:
                i_vehicle.vehicles.popleft()

    def get_x(self, t):
        return (1 - t) ** 2 * self.p0[0] + 2 * t * (1 - t) * self.p1[0] + t ** 2 * self.p2[0]
    def get_y(self, t):
        return (1 - t) ** 2 * self.p0[1] + 2 * t * (1 - t) * self.p1[1] + t ** 2 * self.p2[1]

    def get_position(self, t):
        return np.asarray([(1-t)**2 * self.p0[0] + 2*t*(1-t) * self.p1[0] + t**2 * self.p2[0],
            (1 - t) ** 2 * self.p0[1] + 2 * t * (1 - t) * self.p1[1] + t ** 2 * self.p2[1]])

    def get_derivative(self, t):
        """
        Computes dy/dx of curve given specific time
        :param t: proportion along the curve
        :return: derivative
        """
        dxdt = (2 - 2*t)*(self.p1[0] - self.p0[0]) + (2*t)*(self.p2[0] - self.p1[0])
        dydt = (2 - 2*t)*(self.p1[1] - self.p0[1]) + (2*t)*(self.p2[1] - self.p1[1])
        return dydt/dxdt

    def get_curve(self):
        return self.c1

    def get_intersection_point(self, other):
        """
        Computes intersection point between two bezier curves by minimizing the least squares between the x and y values
        of the function
        :param other: other curve
        :return: t1, t2, point where t1 is time at which curve 1 reaches intersection, t2 is time at which curve 2
                reaches intersection, and point is actual coordinates of intersection
                Returns None,None,None if no intersection point is found
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