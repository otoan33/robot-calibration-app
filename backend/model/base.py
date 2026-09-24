"""解析 API から利用するモデルの共通インターフェース。"""
from abc import ABC, abstractmethod
from typing import Any

import numpy as np


class BaseModel(ABC):
    """学習モデルと推論専用モデルを同じレジストリで扱うための共通契約。"""

    @abstractmethod
    def fit(self, X: Any, y: Any): ...

    @abstractmethod
    def predict(self, X: Any) -> Any: ...

    @abstractmethod
    def score(self, X: Any, y: Any) -> float: ...


# 決定係数 R²（多次元の出力は列ごとの平均からのばらつきで評価する）
def r2_score(actual: np.ndarray, predicted: np.ndarray) -> float:
    residual_sum = float(np.sum((actual - predicted) ** 2))
    total_sum = float(np.sum((actual - np.mean(actual, axis=0)) ** 2))
    if total_sum == 0.0:
        return 1.0 if residual_sum == 0.0 else 0.0
    return 1.0 - residual_sum / total_sum
