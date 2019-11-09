class AbstractLane:

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

