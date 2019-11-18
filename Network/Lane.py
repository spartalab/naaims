from Network.AbstractLane import AbstractLane
from Vehicles.AbstractVehicle import AbstractVehicle

class Lane(AbstractLane):
    def __init__(self, trajectory, length, enteringIntersection=False):
        super().__init__(trajectory, length, enteringIntersection=False)

    """
        This method progresses the movement of the cars in this lane's queue. It uses
        the equations defined in the spec to progress the completion of each vehicle. 
        The general logic of the code is to iterate through all the vehicles and then
        progress each car. We then check to see if there is a car that can enter the intersecton 
        and then pop it off from the queue and change the progress to represent the one for
        the trajectory.
    """
    def progress_time(self):
        # for each vehicle at completion percentage, see if we can move it forward
        # If head vehicle can get a reservation given an approximate arrival time then
        # we should remove the vehicle from the dictionary

        timestep = 5.0
        for x in range(len(self.vehicles)):
            # x_vehicle is the current vehicle
            # b_vehicle is the vehicle that is ahead of the x_vehicle
            # timestep is the time step
            i_vehicle = self.vehicles[x]
            h_vehicle = self.vehicles[x-1] if x != 0 else AbstractVehicle(self, None, 0, 0, 4.5, 3, 0)

            # If we can enter the intersection and the proportion completed is 1
            if x == 0 and i_vehicle.enter_intersection == True and i_vehicle.p >= 1:
                i_vehicle.a = i_vehicle.a_max if i_vehicle.v < self.speed_limit else 0
                i_vehicle.v = min(i_vehicle.v + i_vehicle.a*timestep, self.speed_limit)

                i_vehicle.p = ((i_vehicle.p + (i_vehicle.v*timestep) / self.length) - 1) * (self.length / self.trajectory.length)
                #TODO: keep track of the (x,y) coordinates
                self.trajectory.vehicles.append(self.vehicles.popleft())
                x = 0
                continue

            # Following the equations per the spec
            prog_h = (h_vehicle.l / self.length) + ((timestep * (h_vehicle.v - h_vehicle.b_max * timestep ) / self.length))
            prog_curr = (timestep * (i_vehicle.v - i_vehicle.b_max * timestep)) / self.length
            if ((h_vehicle.p - prog_h) >= (i_vehicle.p + prog_curr)): 
                if (i_vehicle.v < self.speed_limit):
                    i_vehicle.a = i_vehicle.a_max
                elif (i_vehicle.v == self.speed_limit):
                    i_vehicle.a = 0
            else:
                i_vehicle.a = i_vehicle.b
            
            i_vehicle.v = min(i_vehicle.v + i_vehicle.a*timestep, self.speed_limit)
            i_vehicle.p += (timestep * (i_vehicle.v + i_vehicle.a * timestep)) / self.length

            #TODO: keep track of the (x,y) coordinates

    def enter_vehicle(self, vehicle):
        self.vehicles.append(vehicle)

    def exit_vehicle(self):
        return self.vehicles.popleft()