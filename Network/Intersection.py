from IntersectionManager.FCFSManager import FCFSManager
class Intersection():
    def __init__(self, load_file, manager="FCFS"):
        self.manager = FCFSManager(self)
        self.trajectories = set()
        self.incomingLanes = dict() # incoming lane to trajectory
        self.outgoingLanes = dict() # maps trajectory to outgoing lane

    def enter_vehicle(self, vehicle, lane):
        trajectory = self.incomingLanes[lane]
        trajectory.add_vehicle(vehicle)



