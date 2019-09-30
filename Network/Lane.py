from Network.Intersection import Intersection


class Lane():
    def __init__(self, incomingNode, outgoingNode, length, enteringIntersection=False):
        self.incoming = incomingNode
        self.outgoing = outgoingNode
        self.length = length
        if enteringIntersection and type(outgoingNode) != Intersection:
            raise BaseException
        self.vehicles = dict() # Maps completion percentage to vehicle
    def progress_time(self):
        # for each vehicle at completion percentage, see if we can move it forward
        # If head vehicle can get a reservation given an approximate arrival time then
        pass
    def enter_vehicle(self, vehicle):
        if 0 in self.vehicles:
            raise BaseException
        else:
            self.vehicles[0] = vehicle
    def exit_vehicle(self):
        self.outgoing.enter_vehicle(self.vehicles.pop(1), self)