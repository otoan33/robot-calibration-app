from typing import Any

import numpy as np

from .base import ObservationModel


class IdentityObservationModel(ObservationModel):
    """手先位置 XYZ をそのまま観測値とする。"""

    output_size = 3

    def transform(self, positions: np.ndarray, sequence_ids: np.ndarray | None, times: np.ndarray | None) -> np.ndarray:
        return np.asarray(positions, dtype=np.float64)

    def parameter_vector(self) -> np.ndarray:
        return np.empty(0, dtype=np.float64)

    def set_parameter_vector(self, values: np.ndarray) -> None:
        pass

    def save(self) -> dict[str, Any]:
        return {}

    def load(self, parameters: dict[str, Any]) -> None:
        pass
