from Util.BezierTrajectory import BezierTrajectory
from Network.IntersectionLane import IntersectionLane
from Network.Lane import Lane

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

    def build_intersection(self, intersection_traj_df, lanes_df):
        incoming_lanes = {}
        outgoing_lanes = {}

        #TODO: determine the proper length for these lanes
        for index, row in lanes_df.iterrows():
            traj = BezierTrajectory(row['TAIL_X'], row['TAIL_Y'], row['MID_X'], row['MID_Y'], row['HEAD_X'], row['HEAD_Y'])
            if row['IO'] == 'I':
                incoming_lanes[row['ID']] = Lane(traj, 0)
            elif row['IO'] == 'O':
                outgoing_lanes[row['ID']] = Lane(traj, 0)
            else:
                print("Unexpected lane type in build_intersection.")

        for index, row in intersection_traj_df.iterrows():
            tail_traj = incoming_lanes[row['TAIL_ID']].trajectory
            head_traj = outgoing_lanes[row['HEAD_ID']].trajectory
            traj = BezierTrajectory(tail_traj.p2[0], tail_traj.p2[1], row['MID_X'], row['MID_Y'], head_traj.p0[0], head_traj.p0[1])
            #TODO: determine the proper length of the lane
            intersection_lane = IntersectionLane(traj, 0)

            self.add_incoming_lane(incoming_lanes[row['TAIL_ID']], intersection_lane)
            self.add_outgoing_lane(outgoing_lanes[row['HEAD_ID']], intersection_lane)




