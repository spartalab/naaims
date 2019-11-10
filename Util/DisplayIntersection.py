import matplotlib.pyplot as plt

from Network.Intersection import Intersection


def display(intersection: Intersection):
    lanes = list(intersection.incomingLanes.keys())
    for ls in intersection.outgoingLanes.values():
        for l in ls:
            lanes.append(l)
    print(lanes)
    plt.figure(figsize=(15, 16))
    for i, r in enumerate(lanes):
        #     if r['IO'] == 'I':
        plt.xlim(-70, 70)
        plt.ylim(-60, 70)
        plt.arrow(r.trajectory.p0[0], r.trajectory.p0[1], r.trajectory.p2[0] - r.trajectory.p0[0], r.trajectory.p2[1] - r.trajectory.p0[1], head_width=1,
                  overhang=.5)

    for i, r in enumerate(intersection.outgoingLanes.keys()):
        # tail = lanes.loc[r['TAIL_ID'], ['HEAD_X', 'HEAD_Y']].values
        # head = lanes.loc[r['HEAD_ID'], ['TAIL_X', 'TAIL_Y']].values
        plt.arrow(r.trajectory.p0[0], r.trajectory.p0[1], r.trajectory.p2[0] - r.trajectory.p0[0], r.trajectory.p2[1] - r.trajectory.p0[1], head_width=1, overhang=1, color='r')

    plt.show()