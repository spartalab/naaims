"""
This module reads configurations from file and does stuff with it.

[MAIN] # config options \\
steps_per_second -- how many steps does the simulation calculate per
                    second of simulated time?
speed_limit -- default speed limit in meters per second
entrance_length
    Worst case distance required for a simulated vehicle to go from the speed
    limit to a dead halt at its maximum braking speed.

    A semi is usually 83m long. Technically the minimum length required
    assuming all reservations need to get through the intersection at maxmimum
    acceleration is the time to traverse the length of the longest movement at
    full acceleration plus the length of the vehicle, which could end up being
    some super ridiculous length. Maybe this isn't worth it.

    TODO: calculate for every vehicle to find the worst case instead of having
    it provided in the config.
max deceleration
    Maximum comfortable deceleration (braking) for all vehicles in m/s^2.
    Because this is based on human comfort, this should be also be lower than
    the maximum deceleration possible by every vehicle in the simulation, to
    the extent that we don't need to specify a separate deceleration for every
    vehicle in the simulation.
"""

from typing import Dict, Any
from configparser import ConfigParser

# defaults
SETTINGS: Dict[str, Any] = {
    'steps_per_second': '60',
    'speed_limit': '15',
    'max_deceleration': '2.6',
    'max_vehicle_length': '5.5',  # TODO: what to do with this
}
# TODO: sanity check these defaults

config_file_already_read: bool = False

# initialize required variables
steps_per_second = SETTINGS['steps_per_second']
speed_limit = SETTINGS['speed_limit']
max_deceleration = SETTINGS['max_deceleration']
max_stopping_distance = speed_limit**2/(2*max_deceleration)
max_vehicle_length = SETTINGS['max_vehicle_length']
min_entrance_length = max_stopping_distance + max_vehicle_length
TIMESTEP_LENGTH = steps_per_second**(-1)


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
        global max_deceleration
        global max_stopping_distance
        global max_vehicle_length
        global min_entrance_length
        global TIMESTEP_LENGTH
        steps_per_second = SETTINGS['steps_per_second']
        speed_limit = SETTINGS['speed_limit']
        max_deceleration = SETTINGS['max_deceleration']
        max_stopping_distance = speed_limit**2/(2*max_deceleration)
        max_vehicle_length = SETTINGS['max_vehicle_length']
        min_entrance_length = max_stopping_distance + max_vehicle_length
        TIMESTEP_LENGTH = steps_per_second**(-1)
    else:
        raise RuntimeError('Config file already read.')


# shared vin counter
# TODO: not thread safe. fix for multiprocessing.
vin_counter = 0

# time counter
t = 0
