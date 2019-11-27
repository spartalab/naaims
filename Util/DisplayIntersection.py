import matplotlib.pyplot as plt

from Network.Intersection import Intersection


def display(intersection: Intersection):
    lanes = list(intersection.incomingLanes.keys())
    for ls in intersection.outgoingLanes.values():
        for l in ls:
            lanes.append(l)
    print(lanes)
    plt.figure(figsize=(15, 16))
    ax = plt.gca()
    for i, r in enumerate(lanes):
        plt.xlim(-70, 70)
        plt.ylim(-60, 70)
        if r not in intersection.intersection_lanes:
            plt.arrow(r.trajectory.p0[0], r.trajectory.p0[1], r.trajectory.p2[0] - r.trajectory.p0[0], r.trajectory.p2[1] - r.trajectory.p0[1], head_width=1,
                  overhang=.5)

    for i, r in enumerate(intersection.outgoingLanes.keys()):
        if r not in intersection.intersection_lanes:
            plt.arrow(r.trajectory.p0[0], r.trajectory.p0[1], r.trajectory.p2[0] - r.trajectory.p0[0], r.trajectory.p2[1] - r.trajectory.p0[1], head_width=1, overhang=1, color='r')

    for traj in intersection.intersection_lanes:
        traj.trajectory.get_curve().plot(256, ax=ax)


    intersection.find_conflicts()
    print(len(intersection.conflicts))
    x = [cp.point[0] for cp in intersection.conflicts]
    y = [cp.point[1] for cp in intersection.conflicts]
    ax.scatter(x,y)
    plt.show()
