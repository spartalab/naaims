from typing import List, Optional, Type, Dict, Any, Tuple
from importlib import reload
from statistics import mean, stdev
from random import seed
from math import isnan
from os.path import exists
from warnings import warn

from matplotlib.animation import FFMpegFileWriter
from pandas import read_csv, DataFrame, Series

import naaims.shared as SHARED
from scenarios import Symmetrical4Way
from naaims.intersection.managers import (IntersectionManager, FCFSManager,
                                          SignalManager, AuctionManager)
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
         hgv_tracking_sd: float = 0.0631,
         manager_type: Type[IntersectionManager] = FCFSManager,
         vot_mn: float = .5,
         vot_range: float = 1.,
         multiple_sequence_none: Optional[bool] = None,
         mechanism: str = 'first'):
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
    sim = Symmetrical4Way(length=50, manager_type=manager_type,
                          tile_type=tile_type, tile_width=4,
                          vpm=vpm, movement_model=movement_model,
                          av_percentage=av_percentage,
                          acceptable_crash_mev=acceptable_crash_mev,
                          visualize=visualize, visualize_tiles=visualize_tiles,
                          steps_per_second=steps_per_second,
                          hgv_throttle_mn=hgv_throttle_mn,
                          hgv_throttle_sd=hgv_throttle_sd,
                          hgv_tracking_mn=hgv_tracking_mn,
                          hgv_tracking_sd=hgv_tracking_sd,
                          vot_mn=vot_mn,
                          vot_range=vot_range,
                          multiple_sequence_none=multiple_sequence_none,
                          mechanism=mechanism)
    if mp4_filename is not None:
        sim.animate(max_timestep=timesteps).save(  # type: ignore
            f'output/videos/{mp4_filename}.mp4',
            writer=FFMpegFileWriter(fps=steps_per_second))  # type: ignore
    else:
        for _ in range(time*steps_per_second):
            sim.step()
        if log_filename:
            sim.save_log(f'output/logs/{log_filename}.txt')


def trials(time: int = 10*60,
           vpm: float = 10,
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
           hgv_tracking_sd: float = 0.0631,
           manager_type: Type[IntersectionManager] = FCFSManager,
           vot_mn: float = .5,
           vot_range: float = 1.,
           multiple_sequence_none: Optional[bool] = None,
           mechanism: str = 'first',
           replicate_reference: bool = False,
           scale_one: float = 1,
           scale_all: float = 1) -> None:
    """Run several trials, record their output, and return average delay.

    See main for parameter descriptions.
    """
    timesteps = time*steps_per_second
    scaling_filename_addendum = f'_{scale_one}x_{scale_all}x' if \
        ((scale_one != 1) or (scale_all != 1)) else ''
    vin_scaled: Optional[int] = None
    for i in range(n_trials):
        logname = f'output/logs/{log_name}_{i}.csv'
        predetermined_spawn_specs: List[Dict[str, Any]] = []
        if replicate_reference:
            predetermined_spawn_specs, vin_scaled = read_output_to_replicate(
                logname, timesteps, scale_one, scale_all)
            logname = f'output/logs/{log_name}_{i}{scaling_filename_addendum}'\
                '.csv'
        if exists(logname):
            continue
        for _ in range(retry_attempts):
            try:
                sim = Symmetrical4Way(
                    length=50, manager_type=manager_type, tile_type=tile_type,
                    tile_width=4,
                    vpm=vpm if (not replicate_reference) else 0.,
                    movement_model=movement_model,
                    av_percentage=av_percentage,
                    acceptable_crash_mev=acceptable_crash_mev,
                    steps_per_second=steps_per_second,
                    hgv_throttle_mn=hgv_throttle_mn,
                    hgv_throttle_sd=hgv_throttle_sd,
                    hgv_tracking_mn=hgv_tracking_mn,
                    hgv_tracking_sd=hgv_tracking_sd,
                    vot_mn=vot_mn,
                    vot_range=vot_range,
                    multiple_sequence_none=multiple_sequence_none,
                    mechanism=mechanism,
                    predetermined_spawn_specs=predetermined_spawn_specs
                )
                for _ in range(timesteps):
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
    delay_means: List[float] = []
    weighted_delay_means: List[float] = []
    cost_means: List[float] = []
    scale_one_weighted_ratio: List[float] = []
    scale_all_weighted_ratio: List[float] = []
    scale_one_cost_ratio: List[float] = []
    scale_all_cost_ratio: List[float] = []

    for i in range(n_trials):

        # Get VIN that was scaled, if any
        if replicate_reference:
            _, vin_scaled = read_output_to_replicate(
                f'output/logs/{log_name}_{i}.csv', timesteps, scale_one,
                scale_all)
        else:
            vin_scaled = None

        # Read scaled
        df = read_csv(f'output/logs/{log_name}_{i}{scaling_filename_addendum}'
                      '.csv', header=0, index_col=False)

        # Drop vehicles that have yet to exit.
        df.drop(df.index[df['t_exit'] < 0], axis=0,   # type: ignore
                inplace=True)

        delay = find_delay(df, steps_per_second)
        delay_means.append(delay.mean())  # type: ignore
        payment = df['payment'] if \
            ('payment' in df.columns) else float('inf')  # type: ignore
        vot = df['vot'] if \
            ('vot' in df.columns) else float('inf')  # type: ignore
        weighted_delay = (delay + payment/vot)  # type: ignore
        weighted_delay_mean: float = weighted_delay.mean()  # type: ignore
        weighted_delay_means.append(weighted_delay_mean)
        cost = (delay*vot + payment)  # type: ignore
        cost_mean: float = cost.mean()  # type: ignore
        cost_means.append(cost_mean)

        if (vin_scaled is not None) or (scale_all != 1):
            df = read_csv(f'output/logs/{log_name}_{i}.csv', header=0,
                          index_col=False)
            # Drop vehicles that have yet to exit.
            df.drop(df.index[df['t_exit'] < 0], axis=0,   # type: ignore
                    inplace=True)
            delay_original_series = find_delay(df, steps_per_second)
            weighted_delay_original_series: Series[float] = \
                delay_original_series + df['payment']/df['vot']  # type: ignore
            cost_original_series: Series[float] = delay_original_series * \
                df['vot'] + df['payment']  # type: ignore

            if vin_scaled is not None:
                weighted_delay_scaled: float = \
                    weighted_delay[vin_scaled] if vin_scaled in \
                    weighted_delay.index else float('inf')  # type: ignore
                weighted_delay_original: float = \
                    weighted_delay_original_series[vin_scaled] if (
                        vin_scaled in weighted_delay_original_series.index
                    ) else float('inf')  # type: ignore
                scale_one_weighted_ratio.append(
                    weighted_delay_scaled / weighted_delay_original
                )  # type: ignore

                cost_scaled: float = \
                    cost[vin_scaled] if vin_scaled in \
                    cost.index else float('inf')  # type: ignore
                cost_original: float = cost_original_series[
                    vin_scaled] if (vin_scaled in
                                    cost_original_series.index) else \
                    float('inf')  # type: ignore
                scale_one_cost_ratio.append(cost_scaled / cost_original
                                            )  # type: ignore

            if scale_all != 1:
                scale_all_weighted_ratio.append(
                    weighted_delay_mean /
                    weighted_delay_original_series.mean())  # type: ignore

                scale_all_cost_ratio.append(
                    cost_mean /
                    cost_original_series.mean())  # type: ignore

    sample_delay_mean = mean(delay_means)
    sample_delay_sd = stdev(delay_means)
    sample_weighted_delay_mean = mean(weighted_delay_means)
    sample_weighted_delay_sd = stdev(weighted_delay_means)
    sample_cost_mean = mean(cost_means)
    sample_cost_sd = stdev(cost_means)

    scale_one_weighted_ratio_mean: float = 0.
    scale_one_weighted_ratio_sd: float = 0.
    scale_one_cost_ratio_mean: float = 0.
    scale_one_cost_ratio_sd: float = 0.
    if vin_scaled is not None:
        scale_one_weighted_ratio = [
            n for n in scale_one_weighted_ratio if not (isnan(n) or
                                                        (n == float('inf')))]
        scale_one_weighted_ratio_mean = mean(scale_one_weighted_ratio)
        scale_one_weighted_ratio_sd = stdev(scale_one_weighted_ratio)
        scale_one_cost_ratio = [
            n for n in scale_one_cost_ratio if not (isnan(n) or
                                                    (n == float('inf')))]
        scale_one_cost_ratio_mean = mean(scale_one_cost_ratio)
        scale_one_cost_ratio_sd = stdev(scale_one_cost_ratio)

    scale_all_weighted_ratio_mean: float = 0.
    scale_all_weighted_ratio_sd: float = 0.
    scale_all_cost_ratio_mean: float = 0.
    scale_all_cost_ratio_sd: float = 0.
    if scale_all != 1:
        scale_all_weighted_ratio = [
            n for n in scale_all_weighted_ratio if not (isnan(n) or
                                                        (n == float('inf')))]
        scale_all_weighted_ratio_mean = mean(scale_all_weighted_ratio)
        scale_all_weighted_ratio_sd = stdev(scale_all_weighted_ratio)
        scale_all_cost_ratio = [
            n for n in scale_all_cost_ratio if not (isnan(n) or
                                                    (n == float('inf')))]
        scale_all_cost_ratio_mean = mean(scale_all_cost_ratio)
        scale_all_cost_ratio_sd = stdev(scale_all_cost_ratio)

    with open(f'output/logs/trials_{log_name}{scaling_filename_addendum}.txt',
              'w') as f:
        output = f'trials\n{n_trials}\n\n'\
            'Delay (mean, sd)\n'\
            f'{sample_delay_mean}\n{sample_delay_sd}\n\n'\
            'Weighted delay (mean, sd)\n'\
            f'{sample_weighted_delay_mean}\n{sample_weighted_delay_sd}\n\n'\
            'Cost incurred (mean, sd)\n'\
            f'{sample_cost_mean}\n{sample_cost_sd}\n'
        if vin_scaled is not None:
            output += '\n\n[One liar]\n\n'\
                'Weighted delay ratio (lying/true) (mean, sd)\n'\
                f'{scale_one_weighted_ratio_mean}\n'\
                f'{scale_one_weighted_ratio_sd}\n\n'\
                'Cost incurred ratio (lying/true) (mean, sd)\n'\
                f'{scale_one_cost_ratio_mean}\n'\
                f'{scale_one_cost_ratio_sd}\n'
        if scale_all != 1:
            output += '\n\n[All lying]\n\n'\
                'Weighted delay ratio (lying/true) (mean, sd)\n'\
                f'{scale_all_weighted_ratio_mean}\n'\
                f'{scale_all_weighted_ratio_sd}\n\n'\
                'Cost incurred ratio (lying/true) (mean, sd)\n'\
                f'{scale_all_cost_ratio_mean}\n'\
                f'{scale_all_cost_ratio_sd}\n'
        f.write(output)


def find_delay(df: DataFrame, steps_per_second: int, speed_limit: float = 15,
               length: float = 50) -> Series[float]:
    """Calculate delay relative to the free flow case."""

    # First, subtract approach and outgoing lane traversal times assuming that
    # the vehicle is traveling at the fastest speed possible
    delay: Series[float] = (df['t_exit'] - df['t_spawn']) / \
        steps_per_second - 2*length/speed_limit  # type: ignore

    # Next, subtract the lengths of the intersection lanes by using the
    # difference between the origin and destination IDs
    od = df['destination_target'] - df['origin']
    # Through
    delay[od.abs() == 2] -= 32  # type: ignore
    # Right turn
    delay[(od == 1) | (od == -3)] -= 9.73935/speed_limit  # type: ignore
    # Left turn
    delay[(od == -1) | (od == 3)] -= 29.218/speed_limit  # type: ignore

    return delay


def read_output_to_replicate(filename: str, timesteps: int,
                             scale_one: float = 1., scale_all: float = 1.
                             ) -> Tuple[List[Dict[str, Any]], Optional[int]]:

    # Identify which vin/index to scale for scale_one.
    key_t_spawn: int = timesteps//3
    one_vin_scaled: Optional[int] = None

    predetermined_spawns: List[Dict[str, Any]] = []
    with open(filename, 'r') as f:
        next(f)
        for vin, line in enumerate(f):
            spawn_spec: Dict[str, Any] = {}
            info = line.split(',')
            spawn_spec["vin"] = vin
            spawn_spec["t_spawn"] = int(info[0])
            spawn_spec["origin"] = int(info[3])
            spawn_spec["destination"] = int(info[4])
            spawn_spec["width"] = float(info[6])
            spawn_spec["length"] = float(info[7])
            spawn_spec["throttle_mn"] = float(info[8])
            spawn_spec["throttle_sd"] = float(info[9])
            spawn_spec["tracking_mn"] = float(info[10])
            spawn_spec["tracking_sd"] = float(info[11])
            spawn_spec["vot"] = float(info[12]) * scale_all
            if (one_vin_scaled is None) and \
                    (spawn_spec["t_spawn"] > key_t_spawn):
                spawn_spec["vot"] *= scale_one/scale_all
                one_vin_scaled = vin
            spawn_spec["type"] = info[14]
            predetermined_spawns.append(spawn_spec)
    return predetermined_spawns, one_vin_scaled


def trials_vot_misreport(time: int = 5*60, vpm: float = 10,
                         n_trials: int = 30, log_name: str = 'one_liar_first',
                         steps_per_second: int = 15,
                         vot_mn: float = .5, vot_range: float = 1.,
                         multiple_sequence_none: Optional[bool] = None,
                         mechanism: str = 'first',
                         scale_one: float = 1,
                         scale_all: float = 1):

    # Run the original trial.
    seed(0)
    trials(
        time=time, vpm=vpm, n_trials=n_trials, log_name=log_name,
        steps_per_second=steps_per_second, vot_mn=vot_mn, vot_range=vot_range,
        multiple_sequence_none=multiple_sequence_none, mechanism=mechanism,
        manager_type=AuctionManager)

    seed(0)
    trials(
        time=time, vpm=0., n_trials=n_trials, log_name=log_name,
        steps_per_second=steps_per_second, vot_mn=vot_mn, vot_range=vot_range,
        multiple_sequence_none=multiple_sequence_none, mechanism=mechanism,
        manager_type=AuctionManager,
        replicate_reference=True, scale_one=scale_one, scale_all=scale_all)


if __name__ == "__main__":
    # Test experimental setups for a single 4-way, 3-lane intersection.
    main(30, steps_per_second=15)
    reload(SHARED)
    main(30, movement_model='one draw', manager_type=SignalManager,
         av_percentage=0., acceptable_crash_mev=.05, steps_per_second=15)
    reload(SHARED)
    main(30, movement_model='one draw', tile_type=StochasticTile,
         av_percentage=.0, acceptable_crash_mev=.05, steps_per_second=15)
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
    main(30, manager_type=AuctionManager, steps_per_second=15)
    reload(SHARED)
    main(30, manager_type=AuctionManager, steps_per_second=15,
         mechanism='2nd')
    reload(SHARED)
    main(30, manager_type=AuctionManager, steps_per_second=15,
         mechanism='externality')
    reload(SHARED)
    main(30, manager_type=AuctionManager, steps_per_second=15,
         multiple_sequence_none=True)
    reload(SHARED)
    main(30, manager_type=AuctionManager, steps_per_second=15,
         mechanism='2nd', multiple_sequence_none=True)
    reload(SHARED)
    main(30, manager_type=AuctionManager, steps_per_second=15,
         mechanism='externality', multiple_sequence_none=True)
    reload(SHARED)
    main(30, manager_type=AuctionManager, steps_per_second=15,
         multiple_sequence_none=False)
    reload(SHARED)
    main(30, manager_type=AuctionManager, steps_per_second=15,
         mechanism='2nd', multiple_sequence_none=False)
    reload(SHARED)
    main(30, manager_type=AuctionManager, steps_per_second=15,
         mechanism='externality', multiple_sequence_none=False)
    reload(SHARED)

    # Render video.
    main(2*60, vpm=30, mp4_filename='fcfs_deterministic')
    reload(SHARED)
    main(2*60, movement_model='one draw', manager_type=SignalManager,
         av_percentage=1., acceptable_crash_mev=.05,
         mp4_filename='signal_deterministic')
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
    main(2*60, manager_type=AuctionManager,
         mp4_filename='auction_1st_price')
    reload(SHARED)
    main(2*60, manager_type=AuctionManager, mechanism='2nd',
         mp4_filename='auction_2nd_price')
    reload(SHARED)
    main(2*60, manager_type=AuctionManager, mechanism='externality',
         mp4_filename='auction_externality')
    reload(SHARED)
    main(2*60, manager_type=AuctionManager, multiple_sequence_none=True,
         mp4_filename='auction_1st_price_multiple')
    reload(SHARED)
    main(2*60, manager_type=AuctionManager, mechanism='2nd',
         multiple_sequence_none=True,
         mp4_filename='auction_2nd_price_multiple')
    reload(SHARED)
    main(2*60, manager_type=AuctionManager, mechanism='externality',
         multiple_sequence_none=True,
         mp4_filename='auction_externality_multiple')
    reload(SHARED)
    main(2*60, manager_type=AuctionManager, multiple_sequence_none=False,
         mp4_filename='auction_1st_price_sequence')
    reload(SHARED)
    main(2*60, manager_type=AuctionManager, mechanism='2nd',
         multiple_sequence_none=False,
         mp4_filename='auction_2nd_price_sequence')
    reload(SHARED)
    main(2*60, manager_type=AuctionManager, mechanism='externality',
         multiple_sequence_none=False,
         mp4_filename='auction_externality_sequence')
    reload(SHARED)

    # Run large experiments.
    for vpm in (5, 10, 15):
        trials(5*60, n_trials=30, steps_per_second=15,
               vpm=vpm, log_name=f'deterministic_vpm{vpm}')
        reload(SHARED)
        trials(5*60, n_trials=30, steps_per_second=15,
               manager_type=SignalManager, av_percentage=1.,
               acceptable_crash_mev=.05, vpm=vpm, log_name=f'signal_vpm{vpm}')
        reload(SHARED)
        trials(5*60, n_trials=30, steps_per_second=15,
               movement_model='one draw', tile_type=StochasticTile,
               av_percentage=0., acceptable_crash_mev=.05, vpm=vpm,
               log_name=f'soft_vpm{vpm}')
        reload(SHARED)
        trials(5*60, n_trials=30, steps_per_second=15,
               movement_model='one draw', tile_type=DeterministicTile,
               av_percentage=0., acceptable_crash_mev=.05,
               vpm=vpm, log_name=f'hard_vpm{vpm}')
        reload(SHARED)
    for i in range(10):
        pc = i/10
        if pc in {0., 1}:
            continue
        trials(5*60, n_trials=30, steps_per_second=15,
               movement_model='one draw', tile_type=StochasticTile,
               av_percentage=pc, acceptable_crash_mev=.05,
               log_name=f'soft_av{pc}')
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
    reload(SHARED)

    # Run auction experiments with misreporting VOT vehicle trials.
    for p in (.8, .85, .9, .95, 1.05, 1.1):
        trials_vot_misreport(5*60, n_trials=100, steps_per_second=15,
                             log_name='auction_1st', scale_one=p)
        reload(SHARED)
        trials_vot_misreport(5*60, n_trials=100, steps_per_second=15,
                             mechanism='2nd', log_name='auction_2nd',
                             scale_one=p)
        reload(SHARED)
        trials_vot_misreport(5*60, n_trials=100, steps_per_second=15,
                             mechanism='externality',
                             log_name='auction_externality', scale_one=p)
        reload(SHARED)
        trials_vot_misreport(5*60, n_trials=100, steps_per_second=15,
                             multiple_sequence_none=True,
                             log_name='auction_1st_multiple', scale_one=p)
        reload(SHARED)
        trials_vot_misreport(5*60, n_trials=100, steps_per_second=15,
                             mechanism='2nd', multiple_sequence_none=True,
                             log_name='auction_2nd_multiple', scale_one=p)
        reload(SHARED)
        trials_vot_misreport(5*60, n_trials=100, steps_per_second=15,
                             mechanism='externality',
                             multiple_sequence_none=True,
                             log_name='auction_externality_multiple',
                             scale_one=p)
        reload(SHARED)
        trials_vot_misreport(5*60, n_trials=100, steps_per_second=15,
                             multiple_sequence_none=False,
                             log_name='auction_1st_sequence', scale_one=p)
        reload(SHARED)
        trials_vot_misreport(5*60, n_trials=100, steps_per_second=15,
                             mechanism='2nd', multiple_sequence_none=False,
                             log_name='auction_2nd_sequence', scale_one=p)
        reload(SHARED)
        trials_vot_misreport(5*60, n_trials=100, steps_per_second=15,
                             mechanism='externality',
                             multiple_sequence_none=False,
                             log_name='auction_externality_sequence',
                             scale_one=p)
