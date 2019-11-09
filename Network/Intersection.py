from IntersectionManager.FCFSManager import FCFSManager
class Intersection():
    def __init__(self, load_file, manager="FCFS"):
        self.manager = FCFSManager(self)
        self.trajectories = dict() # maps tuple of (incoming,outgoing) lane pair
        self.incomingLanes = dict() # incoming lane to trajectory
        self.outgoingLanes = dict() # maps trajectory to outgoing lane

    def enter_vehicle(self, vehicle, lane_in, lane_out):
        trajectory = self.trajectories[(lane_in, lane_out)]
        trajectory.enter_vehicle(vehicle)




