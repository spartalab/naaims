"""
Implements a movement model that always returns 0 deviation (no offset and no
stochasticity).
"""

from __future__ import annotations

from naaims.intersection.movement.model import MovementModel


class DeterministicModel(MovementModel):

    def reset_for_requests(self) -> MovementModel:
        """Returns a reset instance of this deterministic movement model."""
        return DeterministicModel()
