"""
The Generator module allows for the procedural generation of new vehicles
according to set patterns (e.g. a set of fixed vehicles, vehicles with random
dimensions and acceleration, etc.).
"""


from abc import abstractmethod
from typing import TypeVar, Dict, Any, Type, List, Optional, Tuple
from random import choices, gauss

import aimsim.shared as SHARED
from ..archetypes import Configurable
from ..vehicles import Vehicle

G = TypeVar('G', bound='Generator')


class Generator(Configurable):
    """
    Generate new vehicles with parameters determined by the implementation of
    child generators.
    """

    @abstractmethod
    def __init__(self,
                 vehicle_types: List[Type[Vehicle]],
                 destinations: int,
                 type_probs: Optional[List[float]] = None,
                 d_probs: Optional[List[float]] = None,
                 pair_id: Optional[int] = None) -> None:
        """Should create a new Generator."""

        if type_probs is not None:
            if ((len(vehicle_types) != len(type_probs))
                    or (sum(type_probs) != 1.)):
                raise ValueError('If not all vehicle types have equal '
                                 'likelihood, each type must have a '
                                 'probability and they must sum to 1.')
            self.type_probs = type_probs
        else:
            self.type_probs = [1/len(vehicle_types)]*len(vehicle_types)
        self.vehicle_types = vehicle_types

        if d_probs is not None:
            if (destinations != len(d_probs)) or (sum(d_probs) != 1.):
                raise ValueError('If not all destinations have equal '
                                 'likelihood, each destination must have a '
                                 'probability and they must sum to 1.')
            self.d_probs = d_probs
        else:
            if pair_id is not None:
                # Spawners and removers are created in pairs (e.g., all
                # two-lane roads). Prevent vehicles from trying to u-turn.
                self.d_probs = [1/(destinations-1)]*destinations
                self.d_probs[pair_id] = 0
            else:
                self.d_probs = [1/destinations]*destinations
        self.destinations = destinations

    @staticmethod
    @abstractmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        raise NotImplementedError("Must be implemented in child classes.")

    @classmethod
    @abstractmethod
    def from_spec(cls: Type[G], spec: Dict[str, Any]) -> G:
        raise NotImplementedError("Must be implemented in child classes.")

    @abstractmethod
    def create_vehicle(self) -> Vehicle:
        """Should create a new vehicle."""
        raise NotImplementedError("Must be implemented in child classes.")

    def _assign_new_vin(self) -> int:
        """Return the latest VIN for assignment to a newly created vehicle."""
        # TODO: not thread-safe. must be changed for multiprocessing.
        vin = SHARED.vin_counter
        SHARED.vin_counter += 1
        return vin

    def _generate_type_and_dest(self) -> Tuple[Type[Vehicle], int]:
        """Randomly generate a valid vehicle type and destination."""
        return (choices(self.vehicle_types, weights=self.type_probs)[0],
                choices(list(range(self.destinations)),
                        weights=self.d_probs)[0])


class NormalGenerator(Generator):
    """
    Generate new vehicles with parameters determined by normal distributions
    unique to each parameter.
    """

    def __init__(self,
                 vehicle_types: List[Type[Vehicle]],
                 destinations: int,
                 type_probs: Optional[List[float]] = None,
                 d_probs: Optional[List[float]] = None,
                 pair_id: Optional[int] = None,
                 max_accel_mn: float = 3,  # maximum acceleration, in m/s^2
                 max_accel_sd: float = 0,
                 max_braking_mn: float = -3.4,  # or -4.5, braking in m/s^2
                 max_braking_sd: float = 0,
                 length_mn: float = 4.5,  # length in meters
                 length_sd: float = 0,
                 width_mn: float = 3,  # width in meters
                 width_sd: float = 0,
                 throttle_score_mn: float = 0,
                 throttle_score_sd: float = 0,
                 tracking_score_mn: float = 0,
                 tracking_score_sd: float = 0,
                 vot_mn: float = 0,  # value of time
                 vot_sd: float = 0) -> None:
        """Create a new normally distributed Generator."""
        super().__init__(
            vehicle_types=vehicle_types,
            destinations=destinations,
            type_probs=type_probs,
            d_probs=d_probs,
        )

        if max_accel_mn <= 0:
            raise ValueError("Max acceleration must be greater than zero.")
        self.max_accel_mn = max_accel_mn

        if max_braking_mn >= SHARED.max_braking:
            raise ValueError(
                f"Max braking must be at most {SHARED.max_braking}.")
        self.max_braking_mn = max_braking_mn

        if (length_mn <= 0) or (width_mn <= 0):
            raise ValueError("Length and width must be greater than 0.")
        self.length_mn = length_mn
        self.width_mn = width_mn

        if vot_mn < 0:
            raise ValueError("VOT must be at least 0.")
        self.vot_mn = vot_mn

        self.throttle_score_mn = throttle_score_mn
        self.tracking_score_mn = tracking_score_mn

        if ((max_accel_sd < 0) or (max_braking_sd < 0) or (length_sd < 0)
                or (width_sd < 0) or (vot_sd < 0) or (throttle_score_sd < 0)
                or (tracking_score_sd < 0)):
            raise ValueError("Standard deviation must be at least zero.")
        self.max_accel_sd = max_accel_sd
        self.max_braking_sd = max_braking_sd
        self.length_sd = length_sd
        self.width_sd = width_sd
        self.vot_sd = vot_sd
        self.throttle_score_sd = throttle_score_sd
        self.tracking_score_sd = tracking_score_sd

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a spawner spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: interpret the string into the spec dict
        raise NotImplementedError("TODO")

        return spec

    @classmethod
    def from_spec(cls, spec: Dict[str, Any]) -> NormalGenerator:
        """Create a new Generator using the given spec."""
        return cls(
            vehicle_types=spec['vehicle_types'],
            destinations=spec['destinations'],
            type_probs=spec['type_probs'],
            d_probs=spec['d_probs'],
            pair_id=spec['pair_id'],
            max_accel_mn=spec['max_accel_mn'],
            max_accel_sd=spec['max_accel_sd'],
            max_braking_mn=spec['max_braking_mn'],
            max_braking_sd=spec['max_braking_sd'],
            length_mn=spec['length_mn'],
            length_sd=spec['length_sd'],
            width_mn=spec['width_mn'],
            width_sd=spec['width_sd'],
            throttle_score_mn=spec['throttle_score_mn'],
            throttle_score_sd=spec['throttle_score_sd'],
            tracking_score_mn=spec['tracking_score_mn'],
            tracking_score_sd=spec['tracking_score_sd'],
            vot_mn=spec['vot_mn'],
            vot_sd=spec['vot_sd']
        )

    def create_vehicle(self) -> Vehicle:
        """Create a new vehicle with normally distributed parameters."""

        vtype, dest = self._generate_type_and_dest()
        max_accel = gauss(self.max_accel_mn, self.max_accel_sd)
        max_braking = gauss(self.max_braking_mn, self.max_braking_sd)
        length = gauss(self.length_mn, self.length_sd)
        width = gauss(self.width_mn, self.width_sd)
        throttle = gauss(self.throttle_score_mn, self.throttle_score_sd)
        tracking = gauss(self.tracking_score_mn, self.tracking_score_sd)
        vot = gauss(self.vot_mn, self.vot_sd)
        return vtype(
            vin=self._assign_new_vin(),
            destination=dest,
            max_accel=max_accel if max_accel > 0 else 1,
            max_braking=(max_braking if (max_braking < SHARED.max_braking)
                         else SHARED.max_braking),
            length=length if length > 0 else 1,
            width=width if width > 0 else 1,
            throttle_score=throttle,
            tracking_score=tracking,
            vot=vot if vot >= 0 else 0
        )
