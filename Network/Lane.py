from Network.AbstractLane import AbstractLane

class Lane(AbstractLane):
    def __init__(self, trajectory, enteringIntersection=False):
        super().__init__(trajectory, enteringIntersection=False)

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
        pass