from __future__ import annotations
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, List, Optional
from naaims.lane import ScheduledExit

from naaims.util import Coord, VehicleSection

if TYPE_CHECKING:
    from naaims.vehicles import Vehicle
    from naaims.util import VehicleTransfer
    from naaims.trajectories import Trajectory
    from naaims.lane import VehicleProgress


class MovementModel(ABC):

    # TODO: (signal) how do movement models work with traffic signals?

    def __init__(self, trajectory: Optional[Trajectory] = None):
        """Instantiate a new stochastic movement model.

        This will define how the IntersectionLane will handle stochastic
        vehicle movement.
        """
        self.__threshold: float = 0.
        self.threshold_registered = False

    def register_threshold(self, threshold: float) -> None:
        """Register probability threshold with the movement model.

        In the intersection constructor, lanes get made first which means that
        the movement model is instantiated before the tiling is, but the
        rejection threshold is dependent on the number of tiles, so it needs to
        be registered outside of the constructor.
        """
        if self.threshold_registered:
            raise RuntimeError("Rejection threshold already registered.")
        self.__threshold = threshold
        self.threshold_registered = True

    @property
    def threshold(self) -> float:
        if not self.threshold_registered:
            raise RuntimeError("Probability threshold not yet registered.")
        return self.__threshold

    def init_lateral_deviation(self, vehicle: Vehicle) -> None:
        """Register vehicle's laterals with the stochastic model."""
        return None

    def init_throttle_deviation(self, vehicle: Vehicle,
                                entrance: VehicleTransfer,
                                v_max: float) -> None:
        """Register vehicle's throttle with the stochastic model."""
        return None

    def remove_vehicle(self, vehicle: Vehicle) -> None:
        """Remove vehicle from the stochastic model's memory."""
        return None

    def fetch_lateral_deviation(self, vehicle: Vehicle, p: float) -> float:
        """Returns the vehicle's next tracking deviation (defaults to 0)."""
        return 0

    def fetch_throttle_deviation(self, vehicle: Vehicle,
                                 section: VehicleSection, p: float
                                 ) -> Optional[float]:
        """Returns the vehicle's next acceleration, if deviating.

        Overrides the accelerate/brake/constant speed pattern established in
        the update_speeds logic with an aberrant acceleration value, if the
        model supports it. Otherwise, return None and default to normal
        behavior.
        """
        return None

    def find_probability_of_usage(self, vehicle: Vehicle, vp: VehicleProgress,
                                  tile_pos: Coord, tile_width: float, t: int
                                  ) -> float:
        """Return the probability that the vehicle uses this tile.

        This method assumes that the probability that the vehicle uses this
        tile is non-zero, and further that it's rounding up to 1.
        """
        # TODO: (tiling) The use of tile_width makes this only usable for
        #       SquareTilings. Adjust for alternative tilings.
        return 1

    def start_projection(self, vehicle: Vehicle, entrance: ScheduledExit,
                         v_max: float) -> None:
        """Starts the projection for this vehicle."""
        pass

    def prepend_probabilities(self, vehicle: Vehicle, entrance: ScheduledExit,
                              v_max: float) -> List[float]:
        """Return count and fraction of tiles to reserve before entrance.

        That is, the deterministic scheduled entrance into the intersection
        found by the Tiling. By defult, finds the tile one timestep before the
        current one to account for rounding errors; otherwise soonest_exit
        ought to guarantee that there will be no conflicts as vehicles enter
        the intersection.

        Only applicable for probabilistic reservations.
        """
        return [1]

    def postpend_probabilities(self, vehicle: Vehicle, timesteps_forward: int,
                               t: int) -> List[float]:
        """Return count and fraction of tiles to reserve after exit.

        That is, the deterministic scheduled exit from the intersection found
        by the Tiling. By default and at minimum, reserves the number of
        timesteps_forward suggested by the Tiling (which assumes deterministic
        movement).
        """
        return [1 for _ in range(timesteps_forward)]

    def clean_up_projection(self, vehicle: Vehicle) -> None:
        """Clean up any projection support structures used for this vehicle."""
        return None

    @abstractmethod
    def reset_for_requests(self) -> MovementModel:
        """Returns a reset instance of this MovementModel."""
        raise NotImplementedError("Should be implemented in child classes.")
