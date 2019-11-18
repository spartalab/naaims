from Network.AbstractLane import AbstractLane

class IntersectionLane(AbstractLane):
    def __init__(self, trajectory, length, enteringIntersection=False):
        super().__init__(trajectory, length, enteringIntersection=False)
    
    def progress_time(self):
        # for each vehicle at completion percentage, see if we can move it forward
        # If head vehicle can get a reservation given an approximate arrival time then
        pass