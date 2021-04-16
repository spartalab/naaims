
from __future__ import annotations
from typing import TYPE_CHECKING, Dict, Any, Type, List, Optional
from random import gauss

import aimsim.shared as SHARED
from aimsim.endpoints.factories.factory import VehicleFactory

if TYPE_CHECKING:
    from aimsim.vehicles import Vehicle


class GaussianVehicleFactory(VehicleFactory):
    """
    Generate new vehicles with parameters determined by normal distributions
    unique to each parameter.
    """

    def __init__(self,
                 vehicle_type: Type[Vehicle],
                 num_destinations: int,
                 destination_probabilities: Optional[List[float]] = None,
                 source_node_id: Optional[int] = None,
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
        """Create a new normally distributed VehicleFactory."""
        super().__init__(
            vehicle_type=vehicle_type,
            num_destinations=num_destinations,
            destination_probabilities=destination_probabilities,
            source_node_id=source_node_id
        )

        if max_accel_mn <= 0:
            raise ValueError("Max acceleration must be greater than zero.")
        self.max_accel_mn = max_accel_mn

        if max_braking_mn > SHARED.SETTINGS.min_braking:
            raise ValueError(
                f"Max braking must be at most {SHARED.SETTINGS.min_braking}.")
        self.max_braking_mn = max_braking_mn

        if max_accel_mn < SHARED.SETTINGS.min_acceleration:
            raise ValueError(
                f"Max accel must be at least "
                f"{SHARED.SETTINGS.min_acceleration}.")
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
    def from_spec(cls, spec: Dict[str, Any]) -> GaussianVehicleFactory:
        """Create a new VehicleFactory using the given spec."""
        return cls(
            vehicle_type=spec['vehicle_type'],
            num_destinations=spec['num_destinations'],
            destination_probabilities=spec['destination_probabilities'],
            source_node_id=spec['source_node_id'],
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

        dest = self._pick_vehicle_type()
        max_accel = gauss(self.max_accel_mn, self.max_accel_sd)
        max_braking = gauss(self.max_braking_mn, self.max_braking_sd)
        length = gauss(self.length_mn, self.length_sd)
        width = gauss(self.width_mn, self.width_sd)
        throttle = gauss(self.throttle_score_mn, self.throttle_score_sd)
        tracking = gauss(self.tracking_score_mn, self.tracking_score_sd)
        vot = gauss(self.vot_mn, self.vot_sd)
        return self.vehicle_type(
            vin=self._assign_new_vin(),
            destination=dest,
            max_accel=max_accel if max_accel > SHARED.SETTINGS.min_acceleration
            else SHARED.SETTINGS.min_acceleration,
            max_braking=(max_braking if (
                max_braking < SHARED.SETTINGS.min_braking
            ) else SHARED.SETTINGS.min_braking),
            length=length if length > 0 else 1,
            width=width if width > 0 else 1,
            throttle_score=throttle,
            tracking_score=tracking,
            vot=vot if vot >= 0 else 0
        )
