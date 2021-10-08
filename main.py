from scenarios import SingleIntersectionSim


def main():
    sim = SingleIntersectionSim()
    for t in range(2*60*60):
        sim.step()


if __name__ == "__main__":
    main()
