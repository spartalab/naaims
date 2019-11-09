from IntersectionManager.Policy import default_policy as dp

class Intersection:

    def __init__(self, lanes):
        self.policy = dp.DefaultPolicy()
        self.CurrentStatusMap = {}
        self.LaneQueues = {}
        
    


    

