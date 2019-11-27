class ConflictRegion():
    def __init__(self, traj1, traj2, point, angle,t1, t2 ):
        self.point = point
        self.angle = angle
        self.traj1 = traj1
        self.traj2 = traj2
        self.t1 = t1
        self.t2 = t2

    def get_conflict_region(self, vehicle):
        pass