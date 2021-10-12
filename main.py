from scenarios import SingleIntersectionSim
from naaims.intersection.managers import FCFSManager


def main():
    sim = SingleIntersectionSim(
        length=30, manager_type=FCFSManager, tile_width=4)
    for t in range(2*60*60):
        sim.step()


if __name__ == "__main__":
    main()
