from typing import List, Optional, Type
from importlib import reload
from statistics import mean, stdev
from os.path import exists
from warnings import warn

from matplotlib.animation import FFMpegFileWriter
from pandas import read_csv

import naaims.shared as SHARED
from scenarios import Symmetrical4Way
from naaims.intersection.managers import FCFSManager
from naaims.intersection.tilings.tiles import (Tile, DeterministicTile,
                                               StochasticTile)


def main(time: int = 2*60, vpm: float = 10,
         movement_model: str = 'deterministic',
         tile_type: Type[Tile] = DeterministicTile,
         av_percentage: float = 1.,
         acceptable_crash_mev: float = 0.,
         visualize: bool = False,
         visualize_tiles: bool = False,
         mp4_filename: Optional[str] = None,
         log_filename: Optional[str] = None,
         steps_per_second: int = 60,
         hgv_throttle_mn: float = 0.0752,
         hgv_throttle_sd: float = 0.1402,
         hgv_tracking_mn: float = -0.0888,
         hgv_tracking_sd: float = 0.0631):
    """Run a simulation instance.

    Parameters
        time: int = 2*60
            How much simulation time to run it for, in seconds.
        visualize: bool = False
            Render the simulator graphics. (Does not show.)
        visualize_tiles: bool = False
            Render the tile reservation status.
        mp4_filename: Optional[str] = None
            Save the simulation animation as an mp4 with this filename if
            provided. Overrides visualize.
        log_filename: Optional[str] = None
            Save the simulation log with this filename if provided.
        See the documentation for Symmetrical4Way for more information on the
        other parameters.
    """
    timesteps = time*steps_per_second
    if mp4_filename is not None:
        visualize = True
    sim = Symmetrical4Way(length=50, manager_type=FCFSManager,
                          tile_type=tile_type, tile_width=4,
                          vpm=vpm, movement_model=movement_model,
                          av_percentage=av_percentage,
                          acceptable_crash_mev=acceptable_crash_mev,
                          visualize=visualize, visualize_tiles=visualize_tiles,
                          steps_per_second=steps_per_second,
                          hgv_throttle_mn=hgv_throttle_mn,
                          hgv_throttle_sd=hgv_throttle_sd,
                          hgv_tracking_mn=hgv_tracking_mn,
                          hgv_tracking_sd=hgv_tracking_sd)
    if mp4_filename is not None:
        sim.animate(max_timestep=timesteps).save(  # type: ignore
            f'output/videos/{mp4_filename}.mp4',
            writer=FFMpegFileWriter(fps=steps_per_second))  # type: ignore
    else:
        for _ in range(time*steps_per_second):
            sim.step()
        if log_filename:
            sim.save_log(f'output/logs/{log_filename}.txt')


def trials(time: int = 10*60, vpm: float = 15,
           movement_model: str = 'deterministic',
           tile_type: Type[Tile] = DeterministicTile,
           av_percentage: float = 1.,
           acceptable_crash_mev: float = 0.,
           n_trials: int = 30,
           log_name: str = 'vanilla',
           steps_per_second: int = 15,
           retry_attempts: int = 10,
           hgv_throttle_mn: float = 0.0752,
           hgv_throttle_sd: float = 0.1402,
           hgv_tracking_mn: float = -0.0888,
           hgv_tracking_sd: float = 0.0631):
    """Run several trials, record their output, and return average delay.

    See main for parameter descriptions.
    """
    for i in range(n_trials):
        logname = f'output/logs/{log_name}_{i}.csv'
        if exists(logname):
            continue
        for _ in range(retry_attempts):
            try:
                sim = Symmetrical4Way(
                    length=50, manager_type=FCFSManager, tile_type=tile_type,
                    tile_width=4, vpm=vpm, movement_model=movement_model,
                    av_percentage=av_percentage,
                    acceptable_crash_mev=acceptable_crash_mev,
                    steps_per_second=steps_per_second,
                    hgv_throttle_mn=hgv_throttle_mn,
                    hgv_throttle_sd=hgv_throttle_sd,
                    hgv_tracking_mn=hgv_tracking_mn,
                    hgv_tracking_sd=hgv_tracking_sd)
                for _ in range(time*steps_per_second):
                    sim.step()
            except RuntimeError:
                warn(f"Encountered runtime error in this trial, attempt {i}")
            else:
                sim.save_log(logname)
                break
            finally:
                reload(SHARED)
        else:
            raise RuntimeError(f"Trial retry attempts exhausted.")
    traversal_time_means: List[float] = []
    for i in range(n_trials):
        df = read_csv(f'output/logs/{log_name}_{i}.csv', header=0)

        # Drop vehicles that have yet to exit.
        df.drop(df.index[df['t_exit'] < 0], axis=0,   # type: ignore
                inplace=True)

        traversal_time_means.append(
            (df['t_exit'] - df['t_spawn']).mean())  # type: ignore
    sample_mean = mean(traversal_time_means)
    sample_sd = stdev(traversal_time_means)
    with open(f'output/logs/trials_{log_name}.txt', 'w') as f:
        f.write(f'{sample_mean}\n{sample_sd}\n{n_trials}')
    return sample_mean, sample_sd


if __name__ == "__main__":
    # Test experimental setups for single intersection FCFS.
    main(30, vpm=10)
    reload(SHARED)
    main(30, movement_model='one draw', tile_type=StochasticTile,
         av_percentage=0., acceptable_crash_mev=.05, steps_per_second=15)
    reload(SHARED)
    main(30, movement_model='one draw', tile_type=StochasticTile,
         av_percentage=.5, acceptable_crash_mev=.05, steps_per_second=15)
    reload(SHARED)
    main(30, movement_model='one draw', tile_type=StochasticTile,
         av_percentage=0., acceptable_crash_mev=.05, steps_per_second=15,
         hgv_throttle_mn=0., hgv_throttle_sd=0.)
    reload(SHARED)
    main(30, movement_model='one draw', tile_type=StochasticTile,
         av_percentage=0., acceptable_crash_mev=.05, steps_per_second=15,
         hgv_tracking_mn=0., hgv_tracking_sd=0.)
    reload(SHARED)

    # Render video.
    main(2*60, vpm=30, mp4_filename='fcfs_deterministic')
    reload(SHARED)
    main(2*60, movement_model='one draw', tile_type=StochasticTile,
         av_percentage=0., acceptable_crash_mev=.05,
         mp4_filename='fcfs_stochastic_soft', visualize_tiles=True)
    reload(SHARED)
    main(2*60, movement_model='one draw', tile_type=DeterministicTile,
         av_percentage=0., acceptable_crash_mev=.05,
         mp4_filename='fcfs_stochastic_hard', visualize_tiles=True)
    reload(SHARED)
    main(2*60, movement_model='one draw', tile_type=StochasticTile,
         av_percentage=.5, acceptable_crash_mev=.05,
         mp4_filename='fcfs_stochastic_soft_50pc', visualize_tiles=True)
    reload(SHARED)
    main(2*60, movement_model='one draw', tile_type=StochasticTile,
         av_percentage=0., acceptable_crash_mev=.05,
         hgv_throttle_mn=0., hgv_throttle_sd=0.,
         mp4_filename='fcfs_stochastic_soft_0_throttle', visualize_tiles=True)
    reload(SHARED)
    main(2*60, movement_model='one draw', tile_type=StochasticTile,
         av_percentage=0., acceptable_crash_mev=.05,
         hgv_tracking_mn=0., hgv_tracking_sd=0.,
         mp4_filename='fcfs_stochastic_soft_0_tracking', visualize_tiles=True)
    reload(SHARED)

    # Run large experiments.
    trials(5*60, n_trials=30, steps_per_second=15, log_name='deterministic')
    reload(SHARED)
    trials(5*60, n_trials=30, steps_per_second=15, movement_model='one draw',
           tile_type=StochasticTile, av_percentage=0.,
           acceptable_crash_mev=.05, log_name='soft')
    reload(SHARED)
    trials(5*60, n_trials=30, steps_per_second=15, movement_model='one draw',
           tile_type=DeterministicTile, av_percentage=0.,
           acceptable_crash_mev=.05,
           log_name='hard')
    reload(SHARED)
    trials(5*60, n_trials=30, steps_per_second=15, movement_model='one draw',
           tile_type=StochasticTile, av_percentage=.5,
           acceptable_crash_mev=.05, log_name='soft_5050')
    reload(SHARED)
    trials(5*60, n_trials=30, steps_per_second=15, movement_model='one draw',
           tile_type=StochasticTile, av_percentage=0.,
           acceptable_crash_mev=.05, hgv_throttle_mn=0., hgv_throttle_sd=0.,
           log_name='soft_0_throttle')
    reload(SHARED)
    trials(5*60, n_trials=30, steps_per_second=15, movement_model='one draw',
           tile_type=StochasticTile, av_percentage=0.,
           acceptable_crash_mev=.05, hgv_tracking_mn=0., hgv_tracking_sd=0.,
           log_name='soft_0_tracking')
