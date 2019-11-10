import sys
from Simulator.ArcAim import ArcAim
from Util.DisplayIntersection import display

def main():
    sim = ArcAim(**sys.argv[1:]) # TODO: sanitize ArcAIM inputs
    display(sim.intersection)

if __name__ == "main":
    main()
