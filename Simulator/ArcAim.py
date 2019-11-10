from Util.BezierTrajectory import BezierTrajectory
from Network.Intersection import Intersection
from Network.IntersectionLane import IntersectionLane
from Network.Lane import Lane


import pandas as pd


class ArcAim:

    # Makes the assumption that both intersection_traj_file and lanes_file are pandas df
    def __init__(self, intersection_traj_file, lanes_file):
        self.intersection = Intersection()
        self.intersection_traj_df = intersection_traj_file
        self.lanes_df = lanes_file

    def build_intersection(self):
        incoming_lanes = {}
        outgoing_lanes = {}

        for index, row in self.lanes_df.iterrows():
            traj = BezierTrajectory(row['TAIL_X'], row['TAIL_Y'], row['MID_X'], row['MID_Y'], row['HEAD_X'], row['HEAD_Y'])
            if row['IO'] == 'I':
                incoming_lanes[row['ID']] = Lane(traj)
            elif row['IO'] == 'O':
                outgoing_lanes[row['ID']] = Lane(traj)
            else:
                print("Unexpected lane type in build_intersection.")

        for index, row in self.intersection_traj_df.iterrows():
            tail_traj = incoming_lanes[row['TAIL_ID']].trajectory
            head_traj = outgoing_lanes[row['HEAD_ID']].trajectory
            traj = BezierTrajectory(tail_traj.p2[0], tail_traj.p2[1], row['MID_X'], row['MID_Y'], head_traj.p0[0], head_traj.p0[1])
            intersection_lane = IntersectionLane(traj)

            self.intersection.add_incoming_lane(incoming_lanes[row['TAIL_ID']], intersection_lane)
            self.intersection.add_outgoing_lane(outgoing_lanes[row['HEAD_ID']], intersection_lane)










