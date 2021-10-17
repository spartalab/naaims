from matplotlib.animation import FFMpegFileWriter

from scenarios import SingleIntersectionSim
from naaims.intersection.managers import FCFSManager


def main(time: int = 2*60, visualize: bool = False, save_mp4: bool = False,
         mp4_filename: str = 'output/videos/single_fcfs_4.mp4'):
    """Run a simulation instance.

    Parameters
        time: int = 2*60
            How much simulation time to run it for, in seconds.
        visualize: bool = False
            Have the sim render animation.
        save_mp4: bool = False
            Save the simulation animation as an mp4. Overrides visualize.
    """
    timesteps = time*60
    if save_mp4 is True:
        visualize = True
    sim = SingleIntersectionSim(
        length=30, manager_type=FCFSManager, tile_width=4, visualize=visualize)
    if save_mp4:
        sim.animate(max_timestep=timesteps).save(
            mp4_filename, writer=FFMpegFileWriter(fps=60))
    else:
        for t in range(time*60):
            sim.step()


if __name__ == "__main__":
    main(visualize=True)
