from typing import List, Optional, Type
from importlib import reload
from statistics import mean

from matplotlib.animation import FFMpegFileWriter
from pandas import read_csv

import naaims.shared as SHARED
from scenarios import Symmetrical4Way
from naaims.intersection.managers import FCFSManager
from naaims.intersection.tilings.tiles import (Tile, DeterministicTile,
                                               StochasticTile)
from naaims.vehicles import Vehicle, AutomatedVehicle, HumanGuidedVehicle


def main(time: int = 2*60, vpm: float = 15,
         movement_model: str = 'deterministic',
         tile_type: Type[Tile] = DeterministicTile,
         vehicle_type: Type[Vehicle] = AutomatedVehicle,
         acceptable_crash_mev: float = 0.,
         visualize: bool = False,
         mp4_filename: Optional[str] = None,
         log_filename: Optional[str] = None,
         steps_per_second: int = 60):
    """Run a simulation instance.

    Parameters
        time: int = 2*60
            How much simulation time to run it for, in seconds.
        visualize: bool = False
            Render the simulator graphics. (Does not show.)
        mp4_filename: Optional[str] = None
            Save the simulation animation as an mp4 with this filename if
            provided. Overrides visualize.
        log_filename: Optional[str] = None
            Save the simulation log with this filename if provided.
        See the documentation for Symmetrical4Way for more information on the
        other parameters.
    """
    timesteps = time*60
    if mp4_filename is not None:
        visualize = True
    sim = Symmetrical4Way(length=50, manager_type=FCFSManager,
                          tile_type=tile_type, tile_width=4,
                          vpm=vpm, movement_model=movement_model,
                          vehicle_type=vehicle_type,
                          acceptable_crash_mev=acceptable_crash_mev,
                          visualize=visualize,
                          steps_per_second=steps_per_second)
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
           vehicle_type: Type[Vehicle] = AutomatedVehicle,
           acceptable_crash_mev: float = 0.,
           n_trials: int = 30,
           log_name: str = 'vanilla',
           steps_per_second: int = 15):
    """Run several trials, record their output, and return average delay.

    See main for parameter descriptions.
    """
    for i in range(n_trials):
        sim = Symmetrical4Way(length=50, manager_type=FCFSManager,
                              tile_type=tile_type, tile_width=4,
                              vpm=vpm, movement_model=movement_model,
                              vehicle_type=vehicle_type,
                              acceptable_crash_mev=acceptable_crash_mev,
                              steps_per_second=steps_per_second)
        for _ in range(time*steps_per_second):
            sim.step()
        sim.save_log(f'output/logs/{log_name}_{i}.csv')
        reload(SHARED)
    with open(f'output/logs/trials_{log_name}.txt', 'w') as f:
        means: List[float] = []
        for i in range(n_trials):
            df = read_csv(f'output/logs/{log_name}_{i}.csv', header=0)

            # Drop vehicles that have yet to exit.
            df.drop(df.index[df['t_exit'] < 0], axis=0,   # type: ignore
                    inplace=True)

            means.append((df['t_exit'] - df['t_spawn']).mean())  # type: ignore
        res = mean(means)
        f.write(str(res))
    return mean(means)


if __name__ == "__main__":
    # Render video for single intersection FCFS with deterministic movement.
    main(2*60, vpm=30, mp4_filename='single_fcfs_4')

    # reload(SHARED)
    # # Render video for single intersection FCFS with stochastic movement.
    # main(2*60, movement_model='one draw', tile_type=StochasticTile,
    #      vehicle_type=HumanGuidedVehicle, acceptable_crash_mev=.05,
    #      mp4_filename='fcfs_4_stochastic')

    # reload(SHARED)
    # Test if stochastic movement works and doesn't error.
    # main(30, movement_model='one draw', tile_type=DeterministicTile,
    #      steps_per_second=4)
    # reload(SHARED)
    # main(30, movement_model='one draw', tile_type=StochasticTile,
    #      vehicle_type=HumanGuidedVehicle, acceptable_crash_mev=.05,
    #      steps_per_second=4)
    # reload(SHARED)
    # main(30, movement_model='one draw', tile_type=DeterministicTile,
    #      vehicle_type=HumanGuidedVehicle, acceptable_crash_mev=.05,
    #      steps_per_second=4)

    # # reload(SHARED)
    # # Run small experiments.
    # trials(60, n_trials=1, steps_per_second=4,
    #        log_name='deterministic_small')
    # reload(SHARED)
    # trials(60, n_trials=1, steps_per_second=4,
    #        movement_model='one draw', tile_type=StochasticTile,
    #        vehicle_type=HumanGuidedVehicle, acceptable_crash_mev=.05,
    #        log_name='soft_small')
    # reload(SHARED)
    # trials(60, n_trials=1, steps_per_second=4,
    #        movement_model='one draw', tile_type=DeterministicTile,
    #        vehicle_type=HumanGuidedVehicle, acceptable_crash_mev=.05,
    #        log_name='hard_small')

    # reload(SHARED)
    # # Render a quick video for stochastic movement
    # main(30, movement_model='one draw', tile_type=StochasticTile,
    #      vehicle_type=HumanGuidedVehicle, acceptable_crash_mev=.05,
    #      mp4_filename='fcfs_4_stochastic', steps_per_second=15)

    # reload(SHARED)
    # # Run medium experiments.
    # trials(60, n_trials=5, steps_per_second=4,
    #        log_name='deterministic_medium')
    # reload(SHARED)
    # trials(60, n_trials=5, steps_per_second=4,
    #        movement_model='one draw', tile_type=StochasticTile,
    #        vehicle_type=HumanGuidedVehicle, acceptable_crash_mev=.05,
    #        log_name='soft_medium')
    # reload(SHARED)
    # trials(60, n_trials=5, steps_per_second=4,
    #        movement_model='one draw', tile_type=DeterministicTile,
    #        vehicle_type=HumanGuidedVehicle, acceptable_crash_mev=.05,
    #        log_name='hard_medium')

    # reload(SHARED)
    # # Run large experiments.
    # trials(5*60, n_trials=30, steps_per_second=15,
    #        log_name='deterministic')
    # reload(SHARED)
    # trials(5*60, n_trials=30, steps_per_second=15,
    #        movement_model='one draw', tile_type=StochasticTile,
    #        vehicle_type=HumanGuidedVehicle, acceptable_crash_mev=.05,
    #        log_name='soft')
    # reload(SHARED)
    # trials(5*60, n_trials=30, steps_per_second=15,
    #        movement_model='one draw', tile_type=DeterministicTile,
    #        vehicle_type=HumanGuidedVehicle, acceptable_crash_mev=.05,
    #        log_name='hard')
