from __future__ import annotations

from naaims.vehicles.automated import AutomatedVehicle


class HumanGuidedVehicle(AutomatedVehicle):
    """
    Does the same thing as Automated, but will have a different type signature.

    Default throttle and tracking values sourced from Jundi and Dr. Boyle's
    driving simulator studies.
    """

    def __init__(self,
                 vin: int,
                 destination: int,
                 max_accel: float = 3,
                 max_braking: float = -3.4,
                 length: float = 4.5,
                 width: float = 3,
                 throttle_mn: float = 0.0752,
                 throttle_sd: float = 0.1402,
                 tracking_mn: float = -0.0888,
                 tracking_sd: float = 0.0631,
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
