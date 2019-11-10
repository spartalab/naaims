import sys
import Simulator.ArcAim as ArcAim

def main():
    ArcAim(**sys.argv[1:]) # TODO: sanitize ArcAIM inputs

if __name__ == "main":
    main()
