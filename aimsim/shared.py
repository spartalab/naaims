"""
This module reads configurations from file and does stuff with it.

[MAIN] # config options \\
steps_per_second
    The number of steps the simulation calculates per second of simulated time.
speed_limit
    The default speed limit in meters per second (can be overridden by specific
    roads or intersections in their specifications).
max_braking
    Maximum comfortable deceleration (braking) for all vehicles in m/s^2.
    Because this is based on human comfort, this should be also be lower than
    the maximum deceleration possible by every vehicle in the simulation, to
    the extent that we don't need to specify a separate deceleration for every
    vehicle in the simulation.
max_vehicle_length
length_buffer_factor
    Amount of clearance to maintain before and after each vehicle (separately),
    as a function of the vehicle's length. The default option makes a vehicle
    10% longer in front and 10% longer behind than it actually is.
entrance_length
    Worst case distance required for a simulated vehicle to go from the speed
    limit to a dead halt at its maximum braking speed.

    A semi is usually 83m long. Technically the minimum length required
    assuming all reservations need to get through the intersection at maximum
    acceleration is the time to traverse the length of the longest movement at
    full acceleration plus the length of the vehicle, which could end up being
    some super ridiculous length. Maybe this isn't worth it.
"""

from typing import Dict, Any
from configparser import ConfigParser

from aimsim.pathfinder import Pathfinder

# defaults
SETTINGS: Dict[str, Any] = {
    'steps_per_second': '60',
    'speed_limit': '15',
    'max_braking': '-2.6',  # -3.4
    'max_vehicle_length': '5.5',  # TODO: what to do with this
    'length_buffer_factor': '0.1'
}
# TODO: sanity check these defaults

config_file_already_read: bool = False

# declare required variables
pathfinder: Pathfinder
steps_per_second: int
speed_limit: int
max_braking: float
length_buffer_factor: float
max_stopping_distance: float
max_vehicle_length: float
min_entrance_length: float
TIMESTEP_LENGTH: float


def read(config_filename: str = './config.ini') -> None:
    global config_file_already_read
    if not config_file_already_read:
        global SETTINGS
        c = ConfigParser(defaults=SETTINGS)
        c.read(config_filename)
        SETTINGS = dict(c.defaults())
        for key, value in SETTINGS.items():
            SETTINGS[key] = eval(value)
        config_file_already_read = True

        # unpack required variables
        global steps_per_second
        global speed_limit
        global max_braking
        global length_buffer_factor
        global max_stopping_distance
        global max_vehicle_length
        global min_entrance_length
        global TIMESTEP_LENGTH
        steps_per_second = SETTINGS['steps_per_second']
        if steps_per_second <= 0:
            raise ValueError("steps_per_second must be greater than 0.")
        speed_limit = SETTINGS['speed_limit']
        if speed_limit <= 0:
            raise ValueError("speed_limit must be greater than 0.")
        max_braking = SETTINGS['max_braking']
        if max_braking >= 0:
            raise ValueError("max_braking must be less than 0.")
        length_buffer_factor = SETTINGS['length_buffer_factor']
        if length_buffer_factor < 0:
            raise ValueError("length_buffer_factor must be at least 0.")
        max_stopping_distance = speed_limit**2/(2*-max_braking)
        max_vehicle_length = SETTINGS['max_vehicle_length']
        if max_vehicle_length <= 0:
            raise ValueError("max_vehicle_length must be greater than 0.")
        min_entrance_length = max_stopping_distance + max_vehicle_length
        TIMESTEP_LENGTH = steps_per_second**(-1)
    else:
        raise RuntimeError('Config file already read.')


# shared vin counter
# TODO: (parallel) Not thread safe. Fix for multiprocessing.
vin_counter = 0

# Initialize global simulation timestep.
t = 0
