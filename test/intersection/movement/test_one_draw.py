from random import seed
from math import atan, cos, floor, pi
from statistics import mean, stdev

from pytest import fixture, raises, approx

import naaims.shared as SHARED
from naaims.trajectories import Trajectory, BezierTrajectory
from naaims.util import (Coord, VehicleSection, VehicleTransfer,
                         t_to_v, x_over_constant_a, free_flow_exit,
                         phi_mu_sigma)
from naaims.lane import ScheduledExit, VehicleProgress
from naaims.vehicles import Vehicle
from naaims.intersection.movement import OneDrawStochasticModel

from test.test_lane import straight_trajectory

slanted = BezierTrajectory(Coord(0, 0), Coord(30, 60), [Coord(15, 30)])


def test_init():
    od = OneDrawStochasticModel(straight_trajectory)
    assert od.disable_stochasticity is False
    assert od.trajectory is straight_trajectory
    assert od.straight is True

    with raises(RuntimeError):
        od.threshold
    od.register_threshold(0.1)
    assert od.threshold == 0.1

    assert OneDrawStochasticModel(slanted).straight is False


@fixture
def od():
    model = OneDrawStochasticModel(slanted)
    model.register_threshold(1e-8)
    return model


# @fixture
# def od_mc_sampled(od: OneDrawStochasticModel, h_vehicle: Vehicle):
#     seed(0)
#     v0 = .1
#     ts0 = 2
#     en = ScheduledExit(h_vehicle, VehicleSection.FRONT, ts0, v0)
#     v_max = 15
#     assert od.prepend_probabilities(h_vehicle, en, v_max) == [1]
#     return od


@fixture
def od_disabled():
    model = OneDrawStochasticModel(straight_trajectory)
    model.register_threshold(1e-8)
    model.disable_stochasticity = True
    return model


def test_init_lateral(od: OneDrawStochasticModel, h_vehicle: Vehicle,
                      od_disabled: OneDrawStochasticModel):
    seed(0)
    od.init_lateral_deviation(h_vehicle)
    assert od.max_lateral_deviation.get(h_vehicle) == 0.019417154046806644 * \
        od.trajectory.length

    od_disabled.init_lateral_deviation(h_vehicle)
    assert h_vehicle not in od_disabled.max_lateral_deviation


def test_t_deterministic_exit():
    v0 = 1.
    a = 3
    v_max = 15.
    x_to_exit = 100
    t_a = (v_max - v0) / a
    t = (x_to_exit - v0*t_a - 1/2*a*t_a**2)/v_max + t_a
    assert OneDrawStochasticModel.t_deterministic_exit(
        v0, a, v_max, x_to_exit) == t


def test_a_p_calc():
    # Reaches the speed limit
    v0 = 1.
    a = 3
    v_max = 15.
    x_to_exit = 100
    t_to_v_max = t_to_v(v0, a, v_max)
    t_exit, _ = free_flow_exit(v0, a, v_max, t_to_v_max,
                               x_over_constant_a(v0, a, t_to_v_max),
                               x_to_exit)
    t_actual_exit = t_exit+1
    t_accel = OneDrawStochasticModel.t_accel(
        v0, v_max, x_to_exit, t_actual_exit)
    a_adjusted = OneDrawStochasticModel.get_a_adjusted(
        v0, v_max, t_accel, t_actual_exit, x_to_exit, 3)
    assert v0*t_accel + 1/2*a_adjusted*t_accel**2 + \
        v_max*(t_actual_exit - t_accel) == x_to_exit
    assert OneDrawStochasticModel.get_p_cutoff(
        v0, a_adjusted, t_accel, .5, 100.5) == (
        100.5-v_max*(t_actual_exit-t_accel))/100.5

    # Doesn't reach the speed limit
    v0 = 1.
    a = 3
    v_max = 15.
    x_to_exit = 10
    t_to_v_max = t_to_v(v0, a, v_max)
    t_exit, _ = free_flow_exit(v0, a, v_max, t_to_v_max,
                               x_over_constant_a(v0, a, t_to_v_max),
                               x_to_exit)
    t_actual_exit = t_exit+1
    t_accel = OneDrawStochasticModel.t_accel(
        v0, v_max, x_to_exit, t_actual_exit)
    a_adjusted = OneDrawStochasticModel.get_a_adjusted(
        v0, v_max, t_accel, t_actual_exit, x_to_exit, 3)
    assert v0*t_accel + 1/2*a_adjusted*t_accel**2 == x_to_exit
    assert OneDrawStochasticModel.get_p_cutoff(
        v0, a_adjusted, t_accel, .5, 10.5) == (
        10.5-v_max*(t_actual_exit-t_accel))/10.5


def test_init_throttle(od: OneDrawStochasticModel, h_vehicle: Vehicle,
                       od_disabled: OneDrawStochasticModel):
    seed(0)
    h_vehicle.velocity = v0 = .1
    entrance = VehicleTransfer(h_vehicle, VehicleSection.FRONT, .01,
                               Coord(0, 0))
    v_max = 15.
    x_to_exit = od.trajectory.length - .01 + h_vehicle.length * \
        (1 + 2*SHARED.SETTINGS.length_buffer_factor)
    od.init_throttle_deviation(h_vehicle, entrance, v_max)
    t_actual = (1-0.019417154046806644) *\
        OneDrawStochasticModel.t_deterministic_exit(v0, 3, v_max, x_to_exit)
    t_accel = OneDrawStochasticModel.t_accel(v0, v_max, x_to_exit, t_actual)

    a_adjusted = OneDrawStochasticModel.get_a_adjusted(
        v0, v_max, t_accel, t_actual, x_to_exit, 3)
    assert od.a_adjusted.get(h_vehicle) == a_adjusted
    assert od.p_cutoff.get(h_vehicle) == OneDrawStochasticModel.get_p_cutoff(
        v0, a_adjusted, t_accel, .01, od.trajectory.length)

    od_disabled.init_throttle_deviation(h_vehicle, entrance, v_max)
    assert h_vehicle not in od_disabled.a_adjusted
    assert h_vehicle not in od_disabled.p_cutoff


def test_remove(od: OneDrawStochasticModel, h_vehicle: Vehicle,
                od_disabled: OneDrawStochasticModel):
    od.init_lateral_deviation(h_vehicle)
    entrance = VehicleTransfer(h_vehicle, VehicleSection.FRONT, .01,
                               Coord(0, 0))
    od.init_throttle_deviation(h_vehicle, entrance, 15.)
    assert h_vehicle in od.a_adjusted
    assert h_vehicle in od.max_lateral_deviation
    assert h_vehicle in od.p_cutoff

    od.remove_vehicle(h_vehicle)
    assert h_vehicle not in od.a_adjusted
    assert h_vehicle not in od.max_lateral_deviation
    assert h_vehicle not in od.p_cutoff

    od_disabled.remove_vehicle(h_vehicle)


def test_lateral_scaling():
    assert OneDrawStochasticModel.lateral_scaling_factor(0) == 0
    assert OneDrawStochasticModel.lateral_scaling_factor(1) == 0
    assert OneDrawStochasticModel.lateral_scaling_factor(.5) == 1
    assert OneDrawStochasticModel.lateral_scaling_factor(.75) == .5


def test_lateral_deviation(od: OneDrawStochasticModel, h_vehicle: Vehicle,
                           od_disabled: OneDrawStochasticModel):
    assert od_disabled.fetch_lateral_deviation(h_vehicle, .5) == 0
    assert od_disabled.fetch_lateral_deviation(h_vehicle, -5) == 0

    seed(0)
    od.init_lateral_deviation(h_vehicle)
    max_lat = 0.019417154046806644 * od.trajectory.length
    assert od.fetch_lateral_deviation(h_vehicle, .5) == max_lat
    assert od.fetch_lateral_deviation(h_vehicle, .7) == approx(.6*max_lat)
    assert od.fetch_lateral_deviation(h_vehicle, 1) == 0

    with raises(ValueError):
        od.fetch_lateral_deviation(h_vehicle, 1.1)
    with raises(ValueError):
        od.fetch_lateral_deviation(h_vehicle, -.1)


def test_throttle_deviation(od: OneDrawStochasticModel, h_vehicle: Vehicle,
                            od_disabled: OneDrawStochasticModel):
    seed(0)
    od.init_lateral_deviation(h_vehicle)
    assert od_disabled.fetch_throttle_deviation(
        h_vehicle, VehicleSection.FRONT, 15.) == 0
    assert od_disabled.fetch_throttle_deviation(
        h_vehicle, VehicleSection.FRONT, -15.) == 0

    h_vehicle.velocity = .5
    entrance = VehicleTransfer(h_vehicle, VehicleSection.FRONT, .01,
                               Coord(0, 0))
    od.init_throttle_deviation(h_vehicle, entrance, 15.)
    assert od.fetch_throttle_deviation(
        h_vehicle, VehicleSection.FRONT, .1) == od.a_adjusted[h_vehicle]
    assert od.fetch_throttle_deviation(h_vehicle, VehicleSection.FRONT, .9
                                       ) == 0
    assert od.fetch_throttle_deviation(h_vehicle, VehicleSection.REAR, 0) == 0
    assert od.fetch_throttle_deviation(
        h_vehicle, VehicleSection.FRONT, od.p_cutoff[h_vehicle]
    ) == od.a_adjusted[h_vehicle]
    assert od.fetch_throttle_deviation(
        h_vehicle, VehicleSection.FRONT, od.p_cutoff[h_vehicle] + 1e-6) == 0


def test_tile_center():
    assert OneDrawStochasticModel.find_tile_center(
        Coord(3.1, 6.8), 5) == Coord(5.6, 9.3)


def test_atan():
    assert OneDrawStochasticModel.atan_full(1, 1) == pi/4
    assert OneDrawStochasticModel.atan_full(1, -1) == -pi/4
    assert OneDrawStochasticModel.atan_full(-1, 1) == 3*pi/4
    assert OneDrawStochasticModel.atan_full(-1, -1) == 5*pi/4
    assert OneDrawStochasticModel.atan_full(0, 1) == pi/2
    assert OneDrawStochasticModel.atan_full(0, -1) == -pi/2


def test_split(h_vehicle: Vehicle):
    h_vehicle.pos = Coord(2, 1)
    assert OneDrawStochasticModel.split_distance(h_vehicle,
                                                 Coord(3, 1)) == (0, 1)
    assert OneDrawStochasticModel.split_distance(
        h_vehicle, Coord(2, 3)) == approx((-2, 0))
    h_vehicle.heading = pi/2
    assert OneDrawStochasticModel.split_distance(
        h_vehicle, Coord(3, 1)) == approx((1, 0))
    assert OneDrawStochasticModel.split_distance(
        h_vehicle, Coord(2, 0)) == approx((0, -1))
    h_vehicle.heading = 3*pi/4
    assert OneDrawStochasticModel.split_distance(
        h_vehicle, Coord(1, 0)) == approx((-2**.5, 0))
    h_vehicle.heading = pi/4
    assert OneDrawStochasticModel.split_distance(
        h_vehicle, Coord(1, 2)) == approx((-2**.5, 0))
    assert OneDrawStochasticModel.split_distance(
        h_vehicle, Coord(3, 0)) == approx((2**.5, 0))

    h_vehicle.heading = 0
    h_vehicle.pos = Coord(0, 0)
    assert OneDrawStochasticModel.split_distance(h_vehicle,
                                                 Coord(1, 0))[1] == 1
    assert OneDrawStochasticModel.split_distance(h_vehicle,
                                                 Coord(2, 1))[1] == approx(2)
    assert OneDrawStochasticModel.split_distance(h_vehicle,
                                                 Coord(0, 3))[1] == approx(0)
    h_vehicle.heading = 3*pi/2
    assert OneDrawStochasticModel.split_distance(h_vehicle,
                                                 Coord(1, 0))[1] == approx(0)
    assert OneDrawStochasticModel.split_distance(h_vehicle,
                                                 Coord(0, 3))[1] == approx(-3)
    assert OneDrawStochasticModel.split_distance(h_vehicle,
                                                 Coord(2, 1))[1] == approx(-1)
    h_vehicle.heading = 3*pi/4
    h_vehicle.pos = Coord(1, 1)
    assert OneDrawStochasticModel.split_distance(h_vehicle,
                                                 Coord(2, 2))[1] == approx(0)
    assert OneDrawStochasticModel.split_distance(
        h_vehicle, Coord(0, 2))[1] == approx(2**.5)
    assert OneDrawStochasticModel.split_distance(
        h_vehicle, Coord(-1, 2))[1] == approx(3*2**.5/2)


def test_tile_incidence():
    assert OneDrawStochasticModel.tile_incidence_length(5, 0) == \
        OneDrawStochasticModel.tile_incidence_length(5, pi/2) == \
        OneDrawStochasticModel.tile_incidence_length(5, pi) == \
        OneDrawStochasticModel.tile_incidence_length(5, 3*pi/2) == 2.5
    assert OneDrawStochasticModel.tile_incidence_length(5, pi/4) == \
        OneDrawStochasticModel.tile_incidence_length(5, 3*pi/4) == \
        OneDrawStochasticModel.tile_incidence_length(5, 5*pi/4) == \
        OneDrawStochasticModel.tile_incidence_length(5, 7*pi/4) == \
        approx((2*2.5**2)**.5)
    assert OneDrawStochasticModel.tile_incidence_length(5, pi/8) == \
        approx(OneDrawStochasticModel.tile_incidence_length(5, 3*pi/8)) == \
        approx(OneDrawStochasticModel.tile_incidence_length(5, 5*pi/8)) == \
        approx(OneDrawStochasticModel.tile_incidence_length(5, 7*pi/8)) == \
        approx(OneDrawStochasticModel.tile_incidence_length(5, 9*pi/8)) == \
        approx(OneDrawStochasticModel.tile_incidence_length(5, 11*pi/8)) == \
        approx(OneDrawStochasticModel.tile_incidence_length(5, 13*pi/8)) == \
        approx(OneDrawStochasticModel.tile_incidence_length(5, 15*pi/8)) == \
        approx(2.5/cos(pi/8))


def test_overlap():
    assert OneDrawStochasticModel.check_d_overlap(8.9, 2.5, 3, .1) is False
    assert OneDrawStochasticModel.check_d_overlap(8.9, 2.5, 11.2, .1) is True
    assert OneDrawStochasticModel.check_d_overlap(8.9, 2.5, .1, 11.5) is True
    assert OneDrawStochasticModel.check_d_overlap(8.9, 2.5, .09, 11.5) is False


def test_p_tracking(od: OneDrawStochasticModel,
                    od_disabled: OneDrawStochasticModel):
    assert od.p_tracking(-10, 0, 5, 2) == approx(phi_mu_sigma(
        -9/od.trajectory.length, 0, 5) - phi_mu_sigma(
            -11/od.trajectory.length, 0, 5))
    assert od.p_tracking(0, 0.1, 1.1, 3) == approx(phi_mu_sigma(
        1.5/od.trajectory.length, .1, 1.1) - phi_mu_sigma(
            -1.5/od.trajectory.length, .1, 1.1))
    assert od_disabled.p_tracking(2, 99, 999, .1) == 0
    assert od_disabled.p_tracking(.04, 99, 999, .1) == 1


def test_probability_tracking(od: OneDrawStochasticModel, h_vehicle: Vehicle,
                              od_disabled: OneDrawStochasticModel):
    assert od_disabled.find_probability_tracking(
        h_vehicle, VehicleProgress(1, .9, .8), 5, 10
    ) == od.find_probability_tracking(
        h_vehicle, VehicleProgress(None, None, None), 5, 10) == 0
    assert od_disabled.find_probability_tracking(
        h_vehicle, VehicleProgress(1, .9, .8), 5, 1
    ) == od.find_probability_tracking(
        h_vehicle, VehicleProgress(None, None, None), 5, 1) == 1

    assert od.find_probability_tracking(
        h_vehicle, VehicleProgress(None, .6, None), 5, 1) == od.p_tracking(
            1, h_vehicle.throttle_mn*.8, h_vehicle.throttle_sd*.8,
            h_vehicle.width)


def test_progress_lambda(od: OneDrawStochasticModel, h_vehicle: Vehicle):
    # Doesn't reach the speed limit before exit.
    ts = 2
    en = ScheduledExit(h_vehicle, VehicleSection.FRONT, ts, .1)
    v_max = 15
    t_exit = 20
    t_accel = OneDrawStochasticModel.t_accel(
        en.velocity, v_max, od.trajectory.length, t_exit)
    a = OneDrawStochasticModel.get_a_adjusted(
        en.velocity, v_max, t_accel, t_exit, od.trajectory.length, 3)
    p = od.progress_lambda_factory(h_vehicle, en, a, v_max)
    assert p(ts + floor(t_accel) *
             SHARED.SETTINGS.steps_per_second) == approx((1, False))
    assert p(ts) == approx((0, False))
    assert p(ts+1000) == approx((0.6985853110694441, False))
    assert p(ts+2000) == approx((2.744650844777781, True))

    # Does reach the speed limit before exit.
    ts = 2
    en = ScheduledExit(h_vehicle, VehicleSection.FRONT, ts, .1)
    v_max = 15
    t_exit = 5
    t_accel = OneDrawStochasticModel.t_accel(
        en.velocity, v_max, od.trajectory.length, t_exit)
    a = OneDrawStochasticModel.get_a_adjusted(
        en.velocity, v_max, t_accel, t_exit, od.trajectory.length, 3)
    p = od.progress_lambda_factory(h_vehicle, en, a, v_max)
    assert p(ts + floor(t_exit * SHARED.SETTINGS.steps_per_second)
             ) == approx((1, False))
    assert p(ts) == approx((0, False))
    assert p(ts+100) == approx((0.25464400750006483, False))
    assert p(ts+1000) == approx((3.6087459737497736, True))


def test_prepend(od: OneDrawStochasticModel, h_vehicle: Vehicle):
    seed(0)
    v0 = .1
    ts0 = 2
    en = ScheduledExit(h_vehicle, VehicleSection.FRONT, ts0, v0)
    v_max = 15
    assert h_vehicle not in od.progress_mc
    assert od.prepend_probabilities(h_vehicle, en, v_max) == [1]
    assert h_vehicle in od.progress_mc
    x_to_exit = od.trajectory.length + \
        h_vehicle.length * (1 + 2*SHARED.SETTINGS.length_buffer_factor)
    t_fastest_exit = OneDrawStochasticModel.t_deterministic_exit(
        v0, SHARED.SETTINGS.min_acceleration, v_max, x_to_exit)
    assert mean([p(ts0)[0] for p in od.progress_mc[h_vehicle]]) == approx(0)
    assert mean([p(ts0 + floor(t_fastest_exit*(1-h_vehicle.throttle_mn) *
                   SHARED.SETTINGS.steps_per_second)
                   )[0] for p in od.progress_mc[h_vehicle]]) == approx(
                       x_to_exit / od.trajectory.length, 1e-2)
    # This is n=1000 so we can be pretty loose with the accuracy.


def test_project_past_end(od: OneDrawStochasticModel, h_vehicle: Vehicle):
    pos_end = od.trajectory.get_position(1)
    assert od.project_pos_past_end(1) == pos_end
    assert od.project_pos_past_end(2) == approx(
        Coord(pos_end.x*2, pos_end.y*2))
    assert od.project_pos_past_end(1.1) == approx(
        Coord(pos_end.x*1.1, pos_end.y*1.1))


def test_create_mc_sample(od: OneDrawStochasticModel, h_vehicle: Vehicle):
    seed(0)
    v0 = .1
    ts0 = 2
    en = ScheduledExit(h_vehicle, VehicleSection.FRONT, ts0, v0)
    v_max = 15
    x_to_exit = od.trajectory.length + \
        h_vehicle.length * (1 + 2*SHARED.SETTINGS.length_buffer_factor)
    t_fastest_exit = OneDrawStochasticModel.t_deterministic_exit(
        v0, SHARED.SETTINGS.min_acceleration, v_max, x_to_exit)
    t_actual = t_fastest_exit*(1-h_vehicle.throttle_mn)
    t_accel = OneDrawStochasticModel.t_accel(v0, v_max, x_to_exit, t_actual)
    a_mn = OneDrawStochasticModel.get_a_adjusted(v0, v_max, t_accel, t_actual,
                                                 x_to_exit, 3)
    assert od.prepend_probabilities(h_vehicle, en, v_max) == [1]
    d, complete = od.create_mc_sample(h_vehicle, ts0 + 10)
    assert not any(complete)
    assert 0 < (max(d) - min(d)) < .005
    ts_exit_mn = ts0 + floor(t_fastest_exit*(1-h_vehicle.throttle_mn) *
                             SHARED.SETTINGS.steps_per_second)
    p = od.progress_lambda_factory(h_vehicle, en, a_mn, v_max)
    d, complete = od.create_mc_sample(
        h_vehicle, ts_exit_mn+1)
    assert mean(complete) == approx(.5, abs=.05)
    assert mean(d) == approx(OneDrawStochasticModel.split_distance(
        h_vehicle, od.project_pos_past_end(x_to_exit/od.trajectory.length)
    )[1], 1e-2)


def test_check_update_mc(od: OneDrawStochasticModel, h_vehicle: Vehicle):
    seed(0)
    v0 = .1
    ts0 = 2
    en = ScheduledExit(h_vehicle, VehicleSection.FRONT, ts0, v0)
    v_max = 15
    assert h_vehicle not in od.d_mc
    assert h_vehicle not in od.t_of_mc_cached
    assert od.prepend_probabilities(h_vehicle, en, v_max) == [1]
    od.check_update_mc(h_vehicle, 2)
    assert h_vehicle in od.d_mc
    assert od.t_of_mc_cached.get(h_vehicle) == 2
    od.check_update_mc(h_vehicle, 2)
    assert od.t_of_mc_cached.get(h_vehicle) == 2

    x_to_exit = od.trajectory.length + \
        h_vehicle.length * (1 + 2*SHARED.SETTINGS.length_buffer_factor)
    t_fastest_exit = OneDrawStochasticModel.t_deterministic_exit(
        v0, SHARED.SETTINGS.min_acceleration, v_max, x_to_exit)
    t_actual = t_fastest_exit*(1-h_vehicle.throttle_mn)
    t_accel = OneDrawStochasticModel.t_accel(v0, v_max, x_to_exit, t_actual)
    a_mn = OneDrawStochasticModel.get_a_adjusted(v0, v_max, t_accel, t_actual,
                                                 x_to_exit, 3)
    d, complete = od.create_mc_sample(h_vehicle, ts0 + 10)
    od.check_update_mc(h_vehicle, ts0 + 10)
    assert od.d_mc[h_vehicle] == d
    assert od.mc_complete[h_vehicle] == complete
    assert not any(complete)
    assert 0 < (max(d) - min(d)) < .005
    ts_exit_mn = ts0 + floor(t_fastest_exit*(1-h_vehicle.throttle_mn) *
                             SHARED.SETTINGS.steps_per_second)
    p = od.progress_lambda_factory(h_vehicle, en, a_mn, v_max)
    d, complete = od.create_mc_sample(h_vehicle, ts_exit_mn+1)
    od.check_update_mc(h_vehicle, ts_exit_mn+1)
    assert od.d_mc[h_vehicle] == d
    assert od.mc_complete[h_vehicle] == complete
    assert mean(complete) == approx(.5, abs=.05)
    assert mean(d) == approx(OneDrawStochasticModel.split_distance(
        h_vehicle, od.project_pos_past_end(x_to_exit/od.trajectory.length)
    )[1], 1e-2)

    od.postpend_probabilities(h_vehicle, 11, ts0)
    assert h_vehicle not in od.progress_mc
    assert h_vehicle not in od.d_mc
    assert h_vehicle not in od.t_of_mc_cached


def test_probability_throttle(od: OneDrawStochasticModel, h_vehicle: Vehicle):
    seed(0)
    v0 = .1
    ts0 = 2
    en = ScheduledExit(h_vehicle, VehicleSection.FRONT, ts0, v0)
    v_max = 15
    x_to_exit = od.trajectory.length + \
        h_vehicle.length * (1 + 2*SHARED.SETTINGS.length_buffer_factor)
    t_fastest_exit = OneDrawStochasticModel.t_deterministic_exit(
        v0, SHARED.SETTINGS.min_acceleration, v_max, x_to_exit)
    assert od.prepend_probabilities(h_vehicle, en, v_max) == [1]

    # 0 stdev case
    tile_width = 5
    assert od.find_probability_throttle(h_vehicle, tile_width, ts0, 0) == 1
    assert od.find_probability_throttle(
        h_vehicle, tile_width, ts0, h_vehicle.length/2 + .1 + 5/2 - 1e-6) == 1
    assert od.find_probability_throttle(
        h_vehicle, tile_width, ts0, h_vehicle.length/2 + .1 + 5/2 + 1e-6) == 0
    tile_width = 3
    assert od.find_probability_throttle(h_vehicle, tile_width, ts0, 0) == 1
    assert od.find_probability_throttle(
        h_vehicle, tile_width, ts0,
        h_vehicle.length/2 + .1 + tile_width/2 - 1e-6) == 1
    assert od.find_probability_throttle(
        h_vehicle, tile_width, ts0,
        h_vehicle.length/2 + .1 + tile_width/2 + 1e-6) == 0

    # >0 stdev cases
    d: float = 5
    assert od.find_probability_throttle(h_vehicle, tile_width, ts0 + 10, d) \
        == phi_mu_sigma(d + h_vehicle.length, mean(od.d_mc[h_vehicle]),
                        stdev(od.d_mc[h_vehicle])) - \
        phi_mu_sigma(d - h_vehicle.length, mean(od.d_mc[h_vehicle]),
                     stdev(od.d_mc[h_vehicle])) == 0

    ts_exit_mn = ts0 + floor(t_fastest_exit*(1-h_vehicle.throttle_mn) *
                             SHARED.SETTINGS.steps_per_second)
    # p = od.progress_lambda_factory(h_vehicle, en, a_mn, v_max)
    d = OneDrawStochasticModel.split_distance(
        h_vehicle, od.project_pos_past_end(x_to_exit/od.trajectory.length)
    )[1]
    assert od.find_probability_throttle(h_vehicle, tile_width, ts_exit_mn, d) \
        == phi_mu_sigma(d + h_vehicle.length/2, mean(od.d_mc[h_vehicle]),
                        stdev(od.d_mc[h_vehicle])) - \
        phi_mu_sigma(d - h_vehicle.length/2, mean(od.d_mc[h_vehicle]),
                     stdev(od.d_mc[h_vehicle])) == approx(1, 1e-5)


def test_postpend(od: OneDrawStochasticModel, h_vehicle: Vehicle,
                  vehicle: Vehicle):
    seed(0)
    v0 = .1
    ts0 = 2
    en = ScheduledExit(h_vehicle, VehicleSection.FRONT, ts0, v0)
    v_max = 15
    assert od.prepend_probabilities(h_vehicle, en, v_max) == [1]

    # Default behavior for low probabilities or distribution is 0.
    assert od.postpend_probabilities(
        h_vehicle, 100, ts0 + 1000) == [1 for _ in range(100)] == \
        od.postpend_probabilities(vehicle, 100, ts0 + 1000)

    assert od.prepend_probabilities(h_vehicle, en, v_max) == [1]
    x_to_exit = od.trajectory.length + \
        h_vehicle.length * (1 + 2*SHARED.SETTINGS.length_buffer_factor)
    t_fastest_exit = OneDrawStochasticModel.t_deterministic_exit(
        v0, SHARED.SETTINGS.min_acceleration, v_max, x_to_exit)
    ts_exit_mn = ts0 + floor(t_fastest_exit*(1-h_vehicle.throttle_mn) *
                             SHARED.SETTINGS.steps_per_second)
    od.check_update_mc(h_vehicle, ts_exit_mn)
    ps = od.postpend_probabilities(h_vehicle, 2, ts_exit_mn)
    assert len(ps) > 2
    ps_2 = ps.copy()
    ps_2.sort(reverse=True)
    assert ps == ps_2

    assert od.prepend_probabilities(h_vehicle, en, v_max) == [1]
    od.check_update_mc(h_vehicle, ts_exit_mn)
    ps = od.postpend_probabilities(h_vehicle, 100, ts_exit_mn)
    assert ps == [1 for _ in range(100)]
