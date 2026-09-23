from typing import Any

import numpy as np

from .base import ObservationModel


class RelativeObservationModel(ObservationModel):
    """計測系列ごとに、最初の点からの相対位置を観測値とする。"""

    output_size = 3

    def transform(self, positions: np.ndarray, sequence_ids: np.ndarray | None, times: np.ndarray | None) -> np.ndarray:
        sequence_ids = np.asarray(sequence_ids)
        # 時刻が無ければ入力順で最初の点を決める
        order = np.arange(len(sequence_ids)) if times is None else np.asarray(times, dtype=np.float64)
        result = np.asarray(positions, dtype=np.float64).copy()
        for sequence_id in np.unique(sequence_ids):
            indices = np.flatnonzero(sequence_ids == sequence_id)
            result[indices] -= result[indices[np.argmin(order[indices])]]
        return result

    def parameter_vector(self) -> np.ndarray:
        return np.empty(0, dtype=np.float64)

    def set_parameter_vector(self, values: np.ndarray) -> None:
        pass

    def save(self) -> dict[str, Any]:
        return {}

    def load(self, parameters: dict[str, Any]) -> None:
        pass
