import sys
from Simulator.ArcAim import ArcAim
from Util.DisplayIntersection import display
import pandas as pd

def main():
    sim = ArcAim(pd.read_csv("intersection_geometries/symm4way_traj.csv", delimiter=','), pd.read_csv("intersection_geometries/symm4way_lane.csv", delimiter=',')) # TODO: sanitize ArcAIM inputs
    display(sim.intersection)

if __name__ == "__main__":
    main()
