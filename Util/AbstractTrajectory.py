from collections import deque

class AbstractTrajectory():
    def __init__(self):
        self.length = 0
        self.vehicles = deque([])
    
    def progress_time(self):
        pass
    
    def get_position(self, t):
        pass