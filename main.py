import sys
from Simulator.ArcAim import ArcAim
from Util.DisplayIntersection import display
import pandas as pd

def main():
    sim = ArcAim(pd.read_csv(), pd.read_csv()) # TODO: sanitize ArcAIM inputs
    display(sim.intersection)

if __name__ == "main":
    main()
