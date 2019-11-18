from collections import deque

class AbstractLane:

    # Abstract this function
    def __init__(self, trajectory, length, enteringIntersection=False):
        #self.vehicles = dict() # Maps completion percentage to vehicle, not sure if needed
        self.vehicles = deque([])
        self.trajectory = trajectory
        self.length = length
        self.speed_limit = 0

    # Return the completion percentage of the trip
    def get_completion_percent(self):
        pass

    # Return the completion percentage to (x,y) coordinates
    def convert_completion_to_xy(self, percentage):
        return self.trajectory.get_position(percentage)
    
    # Different definition for intersection and regular lane
    def progress_time(self):
        pass

    def enter_vehicle(self, vehicle):
        pass

    def exit_vehicle(self):
        pass

