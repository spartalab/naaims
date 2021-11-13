
from __future__ import annotations
from typing import TYPE_CHECKING, Dict, Any, Type, List, Optional
from random import uniform

import naaims.shared as SHARED
from naaims.endpoints.factories.factory import VehicleFactory

if TYPE_CHECKING:
    from naaims.vehicles import Vehicle


class UniformVehicleFactory(VehicleFactory):
    """
    Generate new vehicles with parameters determined by uniform distributions
    unique to each parameter.
    """

    def __init__(self,
                 vehicle_type: Type[Vehicle],
                 num_destinations: int,
                 destination_probabilities: Optional[List[float]] = None,
                 source_node_id: Optional[int] = None,
                 max_accel_mn: float = 3,  # maximum acceleration, in m/s^2
                 max_accel_range: float = 0,
                 max_braking_mn: float = -3.4,  # or -4.5, braking in m/s^2
                 max_braking_range: float = 0,
                 length_mn: float = 4.5,  # length in meters
                 length_range: float = 0,
                 width_mn: float = 3,  # width in meters
                 width_range: float = 0,
                 throttle_mn_mn: float = 0,
                 throttle_mn_range: float = 0,
                 throttle_sd_mn: float = 0,
                 throttle_sd_range: float = 0,
                 tracking_mn_mn: float = 0,
                 tracking_mn_range: float = 0,
                 tracking_sd_mn: float = 0,
                 tracking_sd_range: float = 0,
                 vot_mn: float = 0,  # value of time
                 vot_range: float = 0) -> None:
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

        self.throttle_mn_mn = throttle_mn_mn
        self.throttle_sd_mn = throttle_sd_mn
        self.tracking_mn_mn = tracking_mn_mn
        self.tracking_sd_mn = tracking_sd_mn

        if ((max_accel_range < 0) or (max_braking_range < 0) or
            (length_range < 0) or (width_range < 0) or (vot_range < 0)
            or (throttle_mn_range < 0) or (throttle_sd_range < 0)
                or (tracking_mn_range < 0) or (tracking_sd_range < 0)):
            raise ValueError("Standard deviation must be at least zero.")
        self.max_accel_range = max_accel_range
        self.max_braking_range = max_braking_range
        self.length_range = length_range
        self.width_range = width_range
        self.vot_sd = vot_range
        self.throttle_mn_range = throttle_mn_range
        self.throttle_sd_range = throttle_sd_range
        self.tracking_mn_range = tracking_mn_range
        self.tracking_sd_range = tracking_sd_range

    @staticmethod
    def spec_from_str(spec_str: str) -> Dict[str, Any]:
        """Reads a spec string into a spawner spec dict."""

        spec: Dict[str, Any] = {}

        # TODO: interpret the string into the spec dict
        raise NotImplementedError("TODO")

        return spec

    @classmethod
    def from_spec(cls, spec: Dict[str, Any]) -> UniformVehicleFactory:
        """Create a new VehicleFactory using the given spec."""
        return cls(
            vehicle_type=spec['vehicle_type'],
            num_destinations=spec['num_destinations'],
            destination_probabilities=spec['destination_probabilities'],
            source_node_id=spec['source_node_id'],
            max_accel_mn=spec['max_accel_mn'],
            max_accel_range=spec['max_accel_range'],
            max_braking_mn=spec['max_braking_mn'],
            max_braking_range=spec['max_braking_range'],
            length_mn=spec['length_mn'],
            length_range=spec['length_range'],
            width_mn=spec['width_mn'],
            width_range=spec['width_range'],
            throttle_mn_mn=spec.get('throttle_mn_mn', 0),
            throttle_mn_range=spec.get('throttle_mn_range', 0),
            throttle_sd_mn=spec.get('throttle_sd_mn', 0),
            throttle_sd_range=spec.get('throttle_sd_range', 0),
            tracking_mn_mn=spec.get('tracking_mn_mn', 0),
            tracking_mn_range=spec.get('tracking_mn_range', 0),
            tracking_sd_mn=spec.get('tracking_sd_mn', 0),
            tracking_sd_range=spec.get('tracking_sd_range', 0),
            vot_mn=spec['vot_mn'],
            vot_range=spec['vot_range']
        )

    def create_vehicle(self) -> Vehicle:
        """Create a new vehicle with normally distributed parameters."""

        dest = self.pick_destination()
        max_accel = uniform(self.max_accel_mn - self.max_accel_range/2,
                            self.max_accel_mn + self.max_accel_range/2)
        max_braking = uniform(self.max_braking_mn - self.max_braking_range/2,
                              self.max_braking_mn + self.max_braking_range/2)
        length = uniform(self.length_mn - self.length_range/2,
                         self.length_mn + self.length_range/2)
        width = uniform(self.width_mn - self.width_range/2,
                        self.width_mn + self.width_range/2)
        throttle_mn = uniform(self.throttle_mn_mn - self.throttle_mn_range/2,
                              self.throttle_mn_mn + self.throttle_mn_range/2)
        throttle_sd = uniform(self.throttle_sd_mn - self.throttle_sd_range/2,
                              self.throttle_sd_mn + self.throttle_sd_range/2)
        tracking_mn = uniform(self.tracking_mn_mn - self.tracking_mn_range/2,
                              self.tracking_mn_mn + self.tracking_mn_range/2)
        tracking_sd = uniform(self.tracking_sd_mn - self.tracking_sd_range/2,
                              self.tracking_sd_mn - self.tracking_sd_range/2)
        vot = uniform(self.vot_mn, self.vot_sd)
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
            throttle_mn=throttle_mn,
            throttle_sd=throttle_sd,
            tracking_mn=tracking_mn,
            tracking_sd=tracking_sd,
            vot=vot if vot >= 0 else 0
        )
