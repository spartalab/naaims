import matplotlib.pyplot as plt

from Network.Intersection import Intersection


def display(intersection: Intersection, plot_lim=20, figname=None):
    lanes = list(intersection.incomingLanes.keys())
    for ls in intersection.outgoingLanes.values():
        for l in ls:
            lanes.append(l)
    print(lanes)
    plt.figure(figsize=(15, 16))
    ax = plt.gca()
    plt.xlim(-plot_lim, plot_lim)
    plt.ylim(-plot_lim, plot_lim)
    for i, r in enumerate(lanes):
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
    if figname:
        plt.savefig(figname)
    plt.show()
