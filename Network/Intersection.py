from Network.ConflictRegion import ConflictRegion
from Util.BezierTrajectory import BezierTrajectory
from Network.IntersectionLane import IntersectionLane
from Network.Lane import Lane
import itertools

class Intersection():

    def __init__(self,
        manager="FCFS", # what priority policy does this intersection use?
        v_max=70 # speed limit, in m/s
        ):
        self.incomingLanes = dict() # incoming lane to list of intersection lane
        self.outgoingLanes = dict() # maps intersection lane to outgoing lane
        self.intersection_lanes = set()
        self.conflicts = set()

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

        for _, row in lanes_df.iterrows():
            traj = BezierTrajectory(row['TAIL_X'], row['TAIL_Y'], row['MID_X'], row['MID_Y'], row['HEAD_X'], row['HEAD_Y'])
            if row['IO'] == 'I':
                incoming_lanes[row['ID']] = Lane(traj, 0)
            elif row['IO'] == 'O':
                outgoing_lanes[row['ID']] = Lane(traj, 0)
            else:
                print("Unexpected lane type in build_intersection.")

        for _, row in intersection_traj_df.iterrows():
            tail_traj = incoming_lanes[row['TAIL_ID']].trajectory
            head_traj = outgoing_lanes[row['HEAD_ID']].trajectory
            traj = BezierTrajectory(tail_traj.p2[0], tail_traj.p2[1], row['MID_X'], row['MID_Y'], head_traj.p0[0], head_traj.p0[1])
            #TODO: determine the proper length of the lane
            intersection_lane = IntersectionLane(traj, 0)

            self.intersection_lanes.add(intersection_lane)
            self.add_incoming_lane(incoming_lanes[row['TAIL_ID']], intersection_lane)
            self.add_outgoing_lane(outgoing_lanes[row['HEAD_ID']], intersection_lane)
        # self.find_conflicts()

    def find_conflicts(self):
        for l1, l2 in itertools.combinations(self.intersection_lanes, 2):
            t1, t2, point = l1.trajectory.get_intersection_point(l2.trajectory)
            if point == None:
                continue
            angle = l1.trajectory.get_intersection_angle(l2.trajectory, t1, t2)
            self.conflicts.add(ConflictRegion(l1, l2, point, angle, t1,t2))
