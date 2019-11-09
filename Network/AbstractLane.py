class AbstractLane:

	# Abstract this function
	def __init__(self, incomingNode, outgoingNode, length, enteringIntersection=False):
        self.incoming = incomingNode
        self.outgoing = outgoingNode
        self.length = length
        if enteringIntersection and type(outgoingNode) != Intersection:
            raise BaseException
        self.vehicles = dict() # Maps completion percentage to vehicle

	# Return the completion percentage of the trip
	def get_completion_percent(self):
		pass

	# Return the completion percentage to (x,y) coordinates
	def convert_completion_to_xy(self):
		pass

	def progress_time(self):
		pass

	def enter_vehicle(self, vehicle):
		pass

	def exit_vehicle(self):
		pass

