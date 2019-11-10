class Intersection():
    def __init__(self, manager="FCFS"):
        self.incomingLanes = dict() # incoming lane to list of intersection lane
        self.outgoingLanes = dict() # maps intersection lane to outgoing lane

    def enter_vehicle(self, vehicle, lane_in, lane_out):
        pass

    def add_incoming_lane(self, incoming_lane, intersection_lane):
    	if incoming_lane not in self.incomingLanes:
    		self.incomingLanes[incoming_lane] = [intersection_lane]
    	else:
    		self.incomingLanes[incoming_lane].append(intersection_lane)

    def add_outgoing_lane(self, outgoing_lane, intersection_lane):
    	if outgoing_lane not in self.outgoingLanes:
    		self.outgoingLanes[outgoing_lane] = [intersection_lane]
    	else:
    		self.outgoingLanes[outgoing_lane].append(intersection_lane)

    def add_trajectory(self):
    	pass




