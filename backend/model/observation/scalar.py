from typing import Any

import numpy as np

from .base import ObservationModel


class ScalarObservationModel(ObservationModel):
    """指定方向への射影に、ゲインとバイアスを掛けた 1 次元の値を観測値とする（変位計など）。"""

    output_size = 1

    def __init__(self, axis: Any = (1.0, 0.0, 0.0), gain: float = 1.0, bias: float = 0.0, **_: Any) -> None:
        axis_array = np.asarray(axis, dtype=np.float64)
        self.axis = axis_array / np.linalg.norm(axis_array)
        self.gain = float(gain)
        self.bias = float(bias)

    def transform(self, positions: np.ndarray, sequence_ids: np.ndarray | None, times: np.ndarray | None) -> np.ndarray:
        return (self.gain * (np.asarray(positions, dtype=np.float64) @ self.axis) + self.bias).reshape(-1, 1)

    # ゲインとバイアスは学習対象としてロボットのパラメータと同時に推定する
    def parameter_vector(self) -> np.ndarray:
        return np.array([self.gain, self.bias], dtype=np.float64)

    def set_parameter_vector(self, values: np.ndarray) -> None:
        self.gain, self.bias = np.asarray(values, dtype=np.float64)

    def save(self) -> dict[str, Any]:
        return {"axis": self.axis.tolist(), "gain": self.gain, "bias": self.bias}

    def load(self, parameters: dict[str, Any]) -> None:
        self.__init__(**parameters)
