class AbstractVehicle():

    # initialize some vehicle properties not in the constructor
    a = 0 # zero m/s^2
    p = 0 # progress

    def __init__(self,
        origin_lane, # lane in which this vehicle originates
        destination_lane, # lane that this vehicle wants to exit from
        a_max = 3, # maximum acceleration, in m/s^2
        b_max = 3.4, # maximum (comfortable) braking, in m/s^2 (4.5 for uncomfortable braking)
        l = 4.5, # length in meters
        w = 3, # width in meters
        v = 11 # vehicle speed, in meters per second
        ):
        '''
        Construct a vehicle instance.
        '''
        
        self.origin_lane = origin_lane
        self.destination_lane = destination_lane
        self.a_max = a_max
        self.b_max = b_max
        self.l = l
        self.w = w
        self.v = v

    def reservation_width(self):
        pass

    def get_intersection_origin(self):
        pass

    def get_intersection_destination(self):
        pass

    def get_intersection_departure(self):
        pass

    def get_intersection_trajectory(self):
        pass

    def get_trajectory_distance(self):
        pass

    def get_conflicts_reserved(self):
        pass

    def get_origin(self):
        pass

    def get_destination(self):
        pass

    def get_current_intersection(self):
        pass

    def get_next_intersection(self):
        pass
