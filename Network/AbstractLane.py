from Network.Intersection import Intersection


class AbstractLane:

    # Abstract this function
    def __init__(self, trajectory ,enteringIntersection=False):
        self.vehicles = dict() # Maps completion percentage to vehicle
        self.trajectory = trajectory

    # Return the completion percentage of the trip
    def get_completion_percent(self):
        pass

    # Return the completion percentage to (x,y) coordinates
    def convert_completion_to_xy(self, percentage):
        return self.trajectory.get_position(percentage)

    def progress_time(self):
        pass

    def enter_vehicle(self, vehicle):
        pass

    def exit_vehicle(self):
        pass

