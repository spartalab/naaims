from Util.AbstractTrajectory import AbstractTrajectory


class BezierTrajectory(AbstractTrajectory):
    def __init__(self, x0, y0, x1, y1, x2, y2):
        super().__init__()
        self.p0 = (x0,y0)
        self.p1 = (x1,y1)
        self.p2 = (x2,y2)

    def get_position(self, t):
        return (1-t)**2 * self.p0[0] + 2*t*(1-t) * self.p1[0] + t**2 * self.p2[0], \
            (1 - t) ** 2 * self.p0[1] + 2 * t * (1 - t) * self.p1[1] + t ** 2 * self.p2[1]