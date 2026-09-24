from abc import ABC, abstractmethod
from typing import Any

import numpy as np


class ObservationModel(ABC):
    """ロボットの手先位置を、計測器で観測される値へ変換するモデル。"""

    output_size: int

    @abstractmethod
    def transform(self, positions: np.ndarray, sequence_ids: np.ndarray | None, times: np.ndarray | None) -> np.ndarray: ...

    @abstractmethod
    def parameter_vector(self) -> np.ndarray: ...

    @abstractmethod
    def set_parameter_vector(self, values: np.ndarray) -> None: ...

    @abstractmethod
    def save(self) -> dict[str, Any]: ...

    @abstractmethod
    def load(self, parameters: dict[str, Any]) -> None: ...

    # 平坦化して送られてきた観測値を (N, 出力次元) に揃える
    def validate_targets(self, values: Any, count: int) -> np.ndarray:
        return np.asarray(values, dtype=np.float64).reshape(count, self.output_size)
