"""
This module loads and shares setting options with all simulation sub-modules.
"""

from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from naaims.pathfinder import Pathfinder


class Settings:

    not_read_error = RuntimeError("Shared settings not yet loaded.")
    pathfinder_created_error = RuntimeError("Pathfinder not yet created.")

    def __init__(self) -> None:
        self.already_loaded: bool = False
        self.__pathfinder_created: bool = False
        self.__pathfinder: Pathfinder
        self.__steps_per_second: int
        self.__speed_limit: int
        self.__min_braking: float
        self.__min_acceleration: float
        self.__length_buffer_factor: float
        self.__max_stopping_distance: float
        self.__max_vehicle_length: float
        self.__min_entrance_length: float
        self.__timestep_length: float

    @property
    def pathfinder(self) -> Pathfinder:
        if not self.__pathfinder_created:
            raise Settings.pathfinder_created_error
        return self.__pathfinder

    @pathfinder.setter
    def pathfinder(self, p: Pathfinder) -> None:
        if self.__pathfinder_created:
            raise Settings.pathfinder_created_error
        self.__pathfinder = p
        self.__pathfinder_created = True

    @property
    def steps_per_second(self) -> int:
        if not self.already_loaded:
            raise Settings.not_read_error
        return self.__steps_per_second

    @property
    def speed_limit(self) -> int:
        if not self.already_loaded:
            raise Settings.not_read_error
        return self.__speed_limit

    @property
    def min_braking(self) -> float:
        if not self.already_loaded:
            raise Settings.not_read_error
        return self.__min_braking

    @property
    def min_acceleration(self) -> float:
        if not self.already_loaded:
            raise Settings.not_read_error
        return self.__min_acceleration

    @property
    def length_buffer_factor(self) -> float:
        if not self.already_loaded:
            raise Settings.not_read_error
        return self.__length_buffer_factor

    @property
    def max_stopping_distance(self) -> float:
        if not self.already_loaded:
            raise Settings.not_read_error
        return self.__max_stopping_distance

    @property
    def max_vehicle_length(self) -> float:
        if not self.already_loaded:
            raise Settings.not_read_error
        return self.__max_vehicle_length

    @property
    def min_entrance_length(self) -> float:
        if not self.already_loaded:
            raise Settings.not_read_error
        return self.__min_entrance_length

    @property
    def TIMESTEP_LENGTH(self) -> float:
        if not self.already_loaded:
            raise Settings.not_read_error
        return self.__timestep_length

    def load(self,
             steps_per_second: int = 60,
             speed_limit: int = 15,
             min_braking: float = -2.6,  # -3.4
             min_acceleration: float = 3,  # 3.4
             max_vehicle_length: float = 5.5,  # TODO: what to do with this
             length_buffer_factor: float = 0.1) -> None:
        """Set up shared settings. Only run once per simulation instance.

        Parameters
            steps_per_second: int = 60
                The number of steps the simulation calculates per second of
                simulated time.
            speed_limit: int = 15
                The default speed limit in meters per second (can be overridden
                by specific roads or intersections in their specifications).
            min_braking: float = -2.6
                Maximum comfortable deceleration (braking) for all vehicles in
                m/s^2. Because this is based on human comfort, this should be
                also be lower than the maximum deceleration possible by every
                vehicle in the simulation, to the extent that we don't need to
                specify a separate deceleration for every vehicle in the
                simulation. Although this is called min_braking, this should be
                the braking rate expressed in negative values, corresponding to
                negative acceleration.
            min_acceleration: float = 3
                Like min_braking, for a vehicle's (positive) acceleration rate.
            max_vehicle_length: float = 5.5
            length_buffer_factor: float = 0.1
                Amount of clearance to maintain before and after each vehicle
                (separately), as a function of the vehicle's length. The
                default option, 0.1, makes a vehicle 10% longer in front and
                10% longer behind than it actually is.
            entrance_length: float (TODO: (low) not implemented)
                Worst case distance required for a simulated vehicle to go from
                the speed limit to a dead halt at its maximum braking speed.

                A semi truck is usually 83m long. Technically the minimum
                length required assuming all reservations need to get through
                the intersection at maximum acceleration is the time to
                traverse the length of the longest movement at full
                acceleration plus the length of the vehicle, which could end up
                being some super ridiculous length. Maybe this isn't worth it.
        """

        if not self.already_loaded:

            if steps_per_second <= 0:
                raise ValueError("steps_per_second must be greater than 0.")
            self.__steps_per_second = steps_per_second

            if speed_limit <= 0:
                raise ValueError("speed_limit must be greater than 0.")
            self.__speed_limit = speed_limit

            if min_braking >= 0:
                raise ValueError("min_braking must be negative.")
            self.__min_braking = min_braking

            if min_acceleration <= 0:
                raise ValueError("min_acceleration must be positive.")
            self.__min_acceleration = min_acceleration

            if length_buffer_factor < 0:
                raise ValueError("length_buffer_factor must be at least 0.")
            self.__length_buffer_factor = length_buffer_factor

            self.__max_stopping_distance = speed_limit**2/(2*-min_braking)

            if max_vehicle_length <= 0:
                raise ValueError("max_vehicle_length must be greater than 0.")
            self.__max_vehicle_length = max_vehicle_length

            self.__min_entrance_length = self.__max_stopping_distance + \
                max_vehicle_length

            self.__timestep_length = steps_per_second**(-1)

            self.already_loaded = True

        else:
            raise RuntimeError('Settings already loaded.')


SETTINGS: Settings = Settings()

# shared vin counter
# TODO: (parallel) Not thread safe. Fix for multiprocessing.
vin_counter = 0

# Initialize global simulation timestep.
t = 0
