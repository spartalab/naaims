from __future__ import annotations

from naaims.vehicles.vehicle import Vehicle


class AutomatedVehicle(Vehicle):
    """
    Since the default vehicle is fully automated, all this class does is change
    the name of the base class so it's clear what type of vehicle we're using.
    """

    def __init__(self,
                 vin: int,
                 destination: int,
                 max_accel: float = 3,
                 max_braking: float = -3.4,
                 length: float = 4.5,
                 width: float = 3,
                 throttle_mn: float = 0,
                 throttle_sd: float = 0,
                 tracking_mn: float = 0,
                 tracking_sd: float = 0,
                 vot: float = 0
                 ) -> None:
        super().__init__(vin=vin,
                         destination=destination,
                         max_accel=max_accel,
                         max_braking=max_braking,
                         length=length,
                         width=width,
                         throttle_mn=throttle_mn,
                         throttle_sd=throttle_sd,
                         tracking_mn=tracking_mn,
                         tracking_sd=tracking_sd,
                         vot=vot
                         )
