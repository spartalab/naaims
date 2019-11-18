from Util.AbstractTrajectory import AbstractTrajectory


class BezierTrajectory(AbstractTrajectory):
    def __init__(self, x0, y0, x1, y1, x2, y2):
        super().__init__()
        self.p0 = (x0,y0)
        self.p1 = (x1,y1)
        self.p2 = (x2,y2)
        self.speed_limit = 0

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


    def get_position(self, t):
        return (1-t)**2 * self.p0[0] + 2*t*(1-t) * self.p1[0] + t**2 * self.p2[0], \
            (1 - t) ** 2 * self.p0[1] + 2 * t * (1 - t) * self.p1[1] + t ** 2 * self.p2[1]