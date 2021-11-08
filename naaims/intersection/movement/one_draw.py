from __future__ import annotations
from typing import TYPE_CHECKING, Callable, Dict, List, Optional, Tuple
from random import gauss
from math import atan, pi, sin, cos
from statistics import mean, stdev
from copy import copy

import naaims.shared as SHARED
from naaims.lane import ScheduledExit
from naaims.intersection.movement.model import MovementModel
from naaims.util import (Coord, VehicleSection, x_over_constant_a, t_to_v,
                         free_flow_exit, phi_mu_sigma)

if TYPE_CHECKING:
    from naaims.vehicles import Vehicle
    from naaims.util import VehicleTransfer
    from naaims.trajectories import Trajectory
    from naaims.lane import VehicleProgress


class OneDrawStochasticModel(MovementModel):

    def __init__(self, trajectory: Optional[Trajectory] = None):
        """Instantiates a new one-draw stochastic movement model.

        This model of stochastic movement draws stochasticity terms only once
        as the vehicle enters the intersection, and calculates its deviation
        throughout the intersection based on that one draw. Deviation in the
        tracking (lateral) direction and throttle (longitudinal) directions are
        treated as independent Gaussian (normal) distributions.

        Tracking (lateral) deviation is described as the maximum distance this
        vehicle tends to deviate from the given trajectory at the trajectory's
        middle. Positive values means the deviation tends toward the right of
        the arc, while negative values means deviation toward the left.

        TODO: (low) this model isn't realistic for shallow curves... unless
              drivers are bad at anything that's not a straight line, which
              isn't implausible...
        """
        super().__init__(trajectory)
        self.disable_stochasticity = False

        assert trajectory is not None
        self.trajectory = trajectory

        # Initialize realized movement structures.
        self.max_lateral_deviation: Dict[Vehicle, float] = {}
        self.a_adjusted: Dict[Vehicle, float] = {}
        self.p_cutoff: Dict[Vehicle, float] = {}
        self.for_checking_requests: bool = False

        # Initialize reservation request projection structures.
        self.progress_mc: Dict[Vehicle, List[
            Callable[[int], Tuple[float, bool]]]] = {}
        self.d_mc: Dict[Vehicle, List[float]] = {}
        self.mc_complete: Dict[Vehicle, List[bool]] = {}
        self.t_of_mc_cached: Dict[Vehicle, int] = {}

    # Functions to calculate a vehicle's REALIZED deviation once they're in
    # the intersection.

    def init_lateral_deviation(self, vehicle: Vehicle) -> None:
        """Draw and save deviation terms using vehicle properties."""

        # Do nothing if this instance is not exhibiting stochastic behavior,
        # i.e., when calculating a reservation request.
        if self.disable_stochasticity or self.trajectory.straight:
            return None

        # A positive draw from the vehicle's tracking distribution means that
        # it tends to cut corners and take turns tighter than it should.
        tracking_draw = gauss(vehicle.tracking_mn, vehicle.tracking_sd)
        # A positive tracking deviation means that it deviates to the right, so
        # the sign for the tracking_draw would need to be flipped for left turn
        # trajectories.
        self.max_lateral_deviation[vehicle] = tracking_draw * \
            self.trajectory.length

    def init_throttle_deviation(self, vehicle: Vehicle,
                                entrance: VehicleTransfer,
                                v_max: float) -> None:
        """Initialize variables for a vehicle's realized throttle deviation.

        Called as soon as the front of the vehicle enters the intersection.
        """

        # Do nothing if this instance is not exhibiting stochastic behavior,
        # i.e., when calculating a reservation request.
        if self.disable_stochasticity:
            return None

        # A positive draw from the vehicle's throttle distribution means that
        # it tends to accelerate more than it should.
        v0 = vehicle.velocity
        a = SHARED.SETTINGS.min_acceleration
        assert entrance.distance_left is not None
        x_to_exit = self.trajectory.length - entrance.distance_left + \
            vehicle.length_buffered
        t_deterministic_exit = OneDrawStochasticModel.t_deterministic_exit(
            v0, a, v_max, x_to_exit)
        t_actual_exit = OneDrawStochasticModel.t_exit_sample(
            t_deterministic_exit, vehicle.throttle_mn, vehicle.throttle_sd)

        # Given t_actual_exit, find the adjusted acceleration value as well as
        # the time and proportion of the trajectory the vehicle spends at that
        # new acceleration value.
        t_accel = OneDrawStochasticModel.t_accel(
            v0, v_max, x_to_exit, t_actual_exit)
        a_adjusted = OneDrawStochasticModel.get_a_adjusted(
            v0, v_max, t_accel, t_actual_exit, x_to_exit, a)
        if a_adjusted < 0:
            # Allowing a_adjusted to go negative breaks the probability model
            a_adjusted = 0
        self.a_adjusted[vehicle] = a_adjusted
        self.p_cutoff[vehicle] = OneDrawStochasticModel.get_p_cutoff(
            v0, a_adjusted, t_accel, entrance.distance_left,
            self.trajectory.length, vehicle.length_half_buffered)

    @staticmethod
    def t_deterministic_exit(v0: float, a: float, v_max: float,
                             x_to_exit: float) -> float:
        t_to_v_max = t_to_v(v0, a, v_max)
        x_to_v_max = x_over_constant_a(v0, a, t_to_v_max)
        t_fastest_exit, _ = free_flow_exit(v0, a, v_max, t_to_v_max,
                                           x_to_v_max, x_to_exit)
        return t_fastest_exit

    @staticmethod
    def t_exit_sample(t_deterministic_exit: float, throttle_mn: float,
                      throttle_sd: float) -> float:
        return max(0, t_deterministic_exit * (1-gauss(throttle_mn,
                                                      throttle_sd)))

    @staticmethod
    def t_accel(v0: float, v_max: float, x_to_exit: float,
                t_exit: float) -> float:
        """Find how long the vehicle will accelerate for.

        Given the distance the vehicle has to cover, how long it has to do it,
        and the starting and ending velocities, find the time it needs to
        accelerate for.
        """
        # See dissertation Appendix B for derivation.
        t_accel = (x_to_exit - v_max * t_exit) / \
            (v0/2 - v_max/2) if (v0 < v_max) else 0.
        if t_accel < 0:
            # braking necessary
            return 0.
        elif t_accel > t_exit:
            # can't reach v_max before exit, so just accelerate the entire time
            # TODO: Leads to a stdev of 0.
            return t_exit
        else:
            return t_accel

    @staticmethod
    def get_a_adjusted(v0: float, v_max: float, t_accel: float, t_exit: float,
                       x_to_exit: float, a: float) -> float:
        if t_accel == 0:
            # default to the original acceleration
            return a
        if t_accel == t_exit:
            # doesn't reach v_max, feather it
            return (2*x_to_exit - 2*t_accel*v0)/t_accel**2
        a_adjusted = (v_max - v0)/t_accel
        if a_adjusted < 0:
            # Allowing a_adjusted to go negative breaks the probability model
            a_adjusted = 0
        return a_adjusted

    @staticmethod
    def get_p_cutoff(v0: float, a_adjusted: float, t_accel: float,
                     distance_left: float, traj_length: float,
                     correction: float) -> float:
        return (x_over_constant_a(v0, a_adjusted, t_accel) + distance_left -
                correction)/traj_length if (a_adjusted >= 0) else \
            float('inf')

    def remove_vehicle(self, vehicle: Vehicle) -> None:
        """Delete vehicle's saved deviation terms."""
        if self.disable_stochasticity:
            return None
        if not self.trajectory.straight:
            del self.max_lateral_deviation[vehicle]
        del self.a_adjusted[vehicle]
        del self.p_cutoff[vehicle]

    def fetch_lateral_deviation(self, vehicle: Vehicle, p: float) -> float:
        """Return vehicle's next throttle and tracking deviation.

        Vehicles start deviating as their center section enters the
        intersection, which scales up linearly as they progress to the middle
        of their trajectory (longitudinally), and decrease symmetrically back
        to 0 until their center exits the intersection.
        """
        if self.disable_stochasticity or self.trajectory.straight:
            return 0
        if p < 0 or p > 1:
            raise ValueError("Lateral deviation only valid between 0 and 1 p.")
        return self.max_lateral_deviation[vehicle] * \
            OneDrawStochasticModel.lateral_scaling_factor(p)

    @staticmethod
    def lateral_scaling_factor(p: float) -> float:
        """Scale the lateral deviation factor by progress from center."""
        return (1-2*abs(.5-p))

    def fetch_throttle_deviation(self, vehicle: Vehicle,
                                 section: VehicleSection, p: float
                                 ) -> Optional[float]:
        if self.disable_stochasticity:
            return super().fetch_throttle_deviation(vehicle, section, p)
        if section is not VehicleSection.FRONT:
            # Front has already exited. Approximate its proportional progress
            # by setting p to 1.
            p = 1
        if p > self.p_cutoff[vehicle]:
            # Vehicle has reached speed limit, so no acceleration.
            return 0
        else:
            return self.a_adjusted[vehicle]

    # Functions to find a vehicle's THEORETICAL deviation for use in Tilings
    # and reservation requests.

    def find_probability_of_usage(self, vehicle: Vehicle, vp: VehicleProgress,
                                  tile_pos: Coord, tile_width: float, t: int
                                  ) -> float:

        tile_center_pos = OneDrawStochasticModel.find_tile_center(
            tile_pos, tile_width)
        d_tracking, d_throttle = OneDrawStochasticModel.split_distance(
            vehicle, tile_center_pos)

        # Treat the probability that a point is covered as two independent
        # distributions covering the perpendicular directions.
        p_tracking = self.find_probability_tracking(vehicle, vp, tile_width,
                                                    d_tracking)
        p_throttle = self.find_probability_throttle(vehicle, tile_width, t,
                                                    d_throttle)

        # Multiply by tile size to turn point probability into a probability
        # that the vehicle covers any portion of this tile.
        return p_tracking * p_throttle

    @staticmethod
    def find_tile_center(tile_pos: Coord, tile_width: float) -> Coord:
        return Coord(tile_pos.x + tile_width/2, tile_pos.y + tile_width/2)

    @staticmethod
    def split_distance(vehicle: Vehicle, pos: Coord) -> Tuple[float, float]:
        """Find the lateral and longitudinal distance from a vehicle to a pos.

        Find the length and angle of the vector from the center of the vehicle
        to a position. Use this to find the deviation of the position parallel
        and perpendicular to the vehicle's current heading.
        """
        dx = pos.x - vehicle.pos.x
        dy = pos.y - vehicle.pos.y
        delta = (dx**2 + dy**2)**.5
        theta = OneDrawStochasticModel.atan_full(dx, dy) - vehicle.heading
        d_tracking = -sin(theta)*delta  # convention is right is positive
        d_throttle = cos(theta)*delta
        return d_tracking, d_throttle

    @staticmethod
    def atan_full(dx: float, dy: float) -> float:
        if dx > 0:
            return atan(dy/dx)
        elif dx < 0:
            return atan(dy/dx) + pi
        else:
            return (1 if dy > 0 else -1)*pi/2

    def find_probability_tracking(self, vehicle: Vehicle, vp: VehicleProgress,
                                  tile_width: float, d_tracking: float,
                                  static_buffer: float = .1) -> float:
        # TODO: (low) Link static buffer with tiling.

        # Find the lateral deviation distribution based on the progress of the
        # center of the vehicle. The mean and stdev terms scale linearly as the
        # vehicle progresses through the intersection lane trajectory.
        if (vp.center is None) or self.trajectory.straight:
            # Assume the vehicle is tracking perfectly before its center
            # section enters or if the trajectory is straight or near-straight.
            scaling = 0.
        else:
            scaling = OneDrawStochasticModel.lateral_scaling_factor(vp.center)
        tracking_mn = vehicle.tracking_mn * scaling
        tracking_sd = vehicle.tracking_sd * scaling

        if tracking_sd != 0:
            return self.p_tracking(d_tracking, tracking_mn, tracking_sd,
                                   vehicle.width)
        else:
            # If sigma is 0, we can't use a normal distribution. The vehicle is
            # just consistently off by a factor of tracking_mn.

            # Expand the search area to the entire tile length in the vehicle's
            # lateral axis and check for overlap using the vehicle's heading
            # as the angle of incidence.

            # All that matters is how close the lateral deviation axis
            # intersects the square tile to a vertex vs. edge center.
            l_segment = OneDrawStochasticModel.tile_incidence_length(
                tile_width, vehicle.heading)
            return float(OneDrawStochasticModel.check_d_overlap(
                d_tracking, l_segment, vehicle.width/2 + static_buffer,
                tracking_mn))

    def p_tracking(self, d: float, mu: float, sigma: float, width: float
                   ) -> float:
        """Probability that the vehicle will deviate to d laterally."""

        # If this is a through or close-to-through movement, the vehicle is
        # assumed to have perfect accuracy at trajectory tracking, so this
        # reduces to a binary problem of whether the tile sits under the
        # vehicle or not.
        if self.trajectory.straight:
            return float(-width/2 <= d <= width/2)

        # Recall that the tracking distribution is the deviation relative to
        # the trajectory's length, so we need to divide the deviation
        # d by the trajectory length.
        d_max = (d + width/2)/self.trajectory.length
        d_min = (d - width/2)/self.trajectory.length
        return phi_mu_sigma(d_max, mu, sigma) - phi_mu_sigma(d_min, mu, sigma)

    @staticmethod
    def tile_incidence_length(tile_width: float, theta: float) -> float:
        theta = (pi/4) - abs((pi/4) - theta % (pi/2))
        return (tile_width/2) / abs(cos(theta))

    @staticmethod
    def check_d_overlap(d: float, tile_incidence_length: float,
                        vehicle_incidence_length: float, offset: float
                        ) -> bool:
        tile_left = d - tile_incidence_length
        tile_right = d + tile_incidence_length
        vehicle_left = offset - vehicle_incidence_length
        vehicle_right = offset + vehicle_incidence_length
        return max(tile_left, vehicle_left) <= min(tile_right, vehicle_right)

    def find_probability_throttle(self, vehicle: Vehicle, tile_width: float,
                                  t: int, d_throttle: float,
                                  static_buffer: float = .1) -> float:
        # TODO: (low) Link static buffer with tiling.

        self.check_update_mc(vehicle, t)
        d_mc = self.d_mc[vehicle]

        # Use the monte carlo sim to find the sample mean and variance to use
        # as estimates of the population distribution.
        throttle_mn = mean(d_mc)
        throttle_sd = stdev(d_mc) if (len(d_mc) > 1) else 0

        if throttle_sd != 0:
            d_max = (d_throttle + (vehicle.length/2 + static_buffer))
            d_min = (d_throttle - (vehicle.length/2 + static_buffer))
            return phi_mu_sigma(d_max, throttle_mn, throttle_sd) - \
                phi_mu_sigma(d_min, throttle_mn, throttle_sd)
        else:
            # If sigma is 0, we can't use a normal distribution. The vehicle is
            # just consistently off by a factor of throttle_mn.

            # Expand the search area to the entire tile length in the vehicle's
            # lateral axis and check for overlap using the vehicle's heading
            # as the angle of incidence.

            # All that matters is how close the lateral deviation axis
            # intersects the square tile to a vertex vs. edge center.
            l_segment = OneDrawStochasticModel.tile_incidence_length(
                tile_width, vehicle.heading)
            return float(OneDrawStochasticModel.check_d_overlap(
                d_throttle, l_segment, vehicle.length/2 + static_buffer,
                throttle_mn))

    def check_update_mc(self, vehicle: Vehicle, t: int) -> None:
        """Update the progress monte carlo sim, if necessary.

        If this is the first time this model has seen this vehicle and t tuple,
        update and cache the monte carlo sim of progress values.
        """
        if self.t_of_mc_cached.get(vehicle) != t:
            self.d_mc[vehicle], self.mc_complete[vehicle] = \
                self.create_mc_sample(vehicle, t)
            self.t_of_mc_cached[vehicle] = t

    def create_mc_sample(self, vehicle: Vehicle, t: int) -> \
            Tuple[List[float], List[bool]]:
        """Returns throttle deviation and completed sample distributions at t.

        Returns
            Deviation sample along the trajectory of the vehicle.
            boolean of if the vehicle has COMPLETELY exited (front and rear
                sections) at this timestep.
        """
        d_mc: List[float] = []
        mc_complete: List[bool] = []
        for progress_mc in self.progress_mc[vehicle]:
            p_mc, complete = progress_mc(t)
            mc_complete.append(complete)
            pos_mc: Coord
            # If the vehicle has progressed past the trajectory's endpoint,
            # project its position based on the end of the trajectory.
            if (p_mc > 1) or (p_mc < 0):
                pos_mc = self.project_pos_past_end(p_mc)
            else:
                pos_mc = self.trajectory.get_position(p_mc)

            # Find the difference of the real position from where the vehicle
            # should be at this point by splitting up the observed difference
            # into longitudinal and latitudinal deviation, and discard the
            # latitudinal deviation.
            d_mc.append(OneDrawStochasticModel.split_distance(vehicle,
                                                              pos_mc)[1])
            # This is technically less correct but the other option would be
            # mixing latitudinal and longitudinal deviations, which violates
            # the independence assumption.
        return d_mc, mc_complete

    def project_pos_past_end(self, p: float) -> Coord:
        """Return the projected coordinate of the progress value.

        This monte carlo trial vehicle's center section hasn't entered or has
        already exited. Approximate its distance by drawing a straight line out
        from the end of the trajectory.
        """
        if p > 1:
            p = p - 1
            p_bounded = 1
            angular_correction = 0.
        elif p < 0:
            p = -p
            p_bounded = 0
            angular_correction = pi
        else:
            raise ValueError("No need for projection.")
        heading = self.trajectory.get_heading(p_bounded) + angular_correction
        extend = p * self.trajectory.length
        x_extension = cos(heading) * extend
        y_extension = sin(heading) * extend
        pos1 = self.trajectory.get_position(p_bounded)
        return Coord(pos1.x + x_extension, pos1.y + y_extension)

    def start_projection(self, vehicle: Vehicle, entrance: ScheduledExit,
                         v_max: float, n: int = 30) -> None:

        # Run an n-sample monte carlo sim of acceleration profiles, for use in
        # finding throttle deviation likelihoods.
        v0 = entrance.velocity
        x_to_exit = self.trajectory.length + vehicle.length_buffered
        t_deterministic_exit = OneDrawStochasticModel.t_deterministic_exit(
            v0, SHARED.SETTINGS.min_acceleration, v_max, x_to_exit)
        progress_mc: List[Callable[[int], Tuple[float, bool]]] = []
        for _ in range(n if ((vehicle.throttle_sd != 0) or
                             (entrance.velocity == v_max)) else 1):
            # Sample a traversal time from the vehicle's throttle distribution
            t_actual_exit = OneDrawStochasticModel.t_exit_sample(
                t_deterministic_exit, vehicle.throttle_mn, vehicle.throttle_sd)
            t_accel = OneDrawStochasticModel.t_accel(
                v0, v_max, x_to_exit, t_actual_exit)
            # Calculate the acceleration value from the sampled time
            a_adjusted = OneDrawStochasticModel.get_a_adjusted(
                v0, v_max, t_accel, t_actual_exit, x_to_exit,
                SHARED.SETTINGS.min_acceleration)
            if a_adjusted < 0:
                a_adjusted = 0
            # Create and save a lambda function that uses the acceleration
            # value, speed limit, and so on to find the progress along the
            # trajectory at a given time.
            progress_mc.append(self.progress_lambda_factory(
                v0, entrance.t, a_adjusted, v_max, x_to_exit, t_accel,
                vehicle.length_half_buffered))
            # runs n times to create a monte carlo distribution, unless the
            # vehicle has 0 standard deviation, in which case just run once.
        self.progress_mc[vehicle] = progress_mc

    def progress_lambda_factory(self, v0: float, ts0: int, a: float,
                                v_max: float, x_to_exit: float, t_accel: float,
                                correction: float
                                ) -> Callable[[int], Tuple[float, bool]]:
        """(Progress, complete) function at t given an acceleration profile.

        The parameters given are for the entrance of the FRONT section of the
        vehicle (because that's when the intersection takes control), but this
        should return the progress of the CENTER section of the vehicle, as
        that's what's used to calculate its throttle deviation.

        Note that x_to_exit is not the same as the trajectory length, so the
        second return bool provides additional context about when the REAR of
        the vehicle leaves the intersection in addition to the front, which can
        be found from the progress returned and the trajectory length.
        """
        def progress(ts: int) -> Tuple[float, bool]:
            t = (ts - ts0)/SHARED.SETTINGS.steps_per_second
            x = (x_over_constant_a(v0, a, t_accel) +
                 x_over_constant_a(v_max, 0, t-t_accel)
                 ) if (t > t_accel) else x_over_constant_a(v0, a, t)
            return (x - correction)/self.trajectory.length, (x > x_to_exit)
        return progress

    def postpend_probabilities(self, vehicle: Vehicle, timesteps_forward: int,
                               t: int) -> List[float]:

        default = super().postpend_probabilities(vehicle, timesteps_forward, t)

        if (vehicle.throttle_mn == 0) and (vehicle.throttle_sd == 0):
            return default

        # Take the probability that this tile was used at this timestep and
        # extend it timesteps_forward to sort-of mimic the normal behavior
        # of postpending reservations.
        _, d_throttle = self.split_distance(vehicle, self.trajectory.end_coord)
        to_return: List[float] = [
            self.find_probability_throttle(vehicle, 0, t, d_throttle) for _ in
            range(timesteps_forward)]

        # Tag on more usage probabilities until every instance in the monte
        # carlo sim has left the intersection.
        while not all(self.mc_complete[vehicle]):
            t += 1
            to_return.append(self.find_probability_throttle(vehicle, 0, t,
                                                            d_throttle))

        # Clean up after throttle
        self.clean_up_projection(vehicle)

        if len(to_return) <= len(default):
            # Ensure that no stochastic exiting reservation buffer is more
            # lenient than the default behavior.
            return default
        else:
            return to_return

    def clean_up_projection(self, vehicle: Vehicle) -> None:
        if vehicle in self.progress_mc:
            del self.progress_mc[vehicle]
        if vehicle in self.d_mc:
            del self.d_mc[vehicle]
        if vehicle in self.mc_complete:
            del self.mc_complete[vehicle]
        if vehicle in self.t_of_mc_cached:
            del self.t_of_mc_cached[vehicle]

    def reset_for_requests(self) -> MovementModel:
        """Returns a reset instance of this model for projections.

        Clones of IntersectionLanes, and thus MovementModels, are
        made for mocking requests and checking tile usage. For this use case,
        stochasticity is measured probabilistically in the tile space and not
        realized, so we return a clean instance with stochastic movement (but
        not probability calculations) disabled.
        """
        clone = copy(self)

        clone.max_lateral_deviation = {}
        clone.a_adjusted = {}
        clone.p_cutoff = {}

        clone.progress_mc = {}
        clone.d_mc = {}
        clone.mc_complete = {}
        clone.t_of_mc_cached = {}

        clone.disable_stochasticity = True

        return clone
