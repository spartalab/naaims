import sys

import pandas as pd

from aimsim.simulators import SingleIntersectionSimulator


def main():
    sim = SingleIntersectionSimulator(
        pd.read_csv("intersection_layouts/symm4way_traj.csv",
                    delimiter=','),
        pd.read_csv("intersection_layouts/symm4way_lane.csv",
                    delimiter=',')
    )
    sim.display()


if __name__ == "__main__":
    main()
