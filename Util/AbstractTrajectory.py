from collections import deque

class AbstractTrajectory():
    def __init__(self):
        self.length = 0
        self.vehicles = deque([])
    
    def progress_time(self):
        pass
    
    def get_position(self, t):
        pass
    def get_curve(self):
        pass
    def get_intersection_point(self, other):
        pass
    def get_intersection_angle(self, other, time1, time2):
        pass