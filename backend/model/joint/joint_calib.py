"""関節角の周期誤差を補正するモデル。"""
from typing import Any

import numpy as np
from scipy.optimize import curve_fit

from ..base import BaseModel, r2_score


class JointCalibModel(BaseModel):
    """周期正弦波の和で関節誤差を近似する（旧 ``JointWave``）。

    ``periods`` は補正したい周期 [deg]。入力 X は関節角 [deg]、出力 y は対応する補正量。
    """

    def __init__(self, periods: list[float] | np.ndarray, maxfev: int = 10_000, **_: Any) -> None:
        self.periods = np.asarray(periods, dtype=np.float64)
        self.maxfev = int(maxfev)
        self.amplitudes_: np.ndarray | None = None
        self.offsets_: np.ndarray | None = None

    def fit(self, X: Any, y: Any) -> "JointCalibModel":
        """関節角と実測補正量から、周期ごとの振幅・位相を推定する。"""
        initial = np.ones(self.periods.size * 2, dtype=np.float64)
        parameters, _ = curve_fit(self._wave, self._flat(X), self._flat(y), p0=initial, maxfev=self.maxfev)

        # 振幅は正に揃え、符号の反転は位相を 180° ずらして表す
        amplitudes, offsets = parameters.reshape(2, -1)
        negative = amplitudes < 0.0
        amplitudes[negative] *= -1.0
        offsets[negative] += 180.0
        self.amplitudes_, self.offsets_ = amplitudes, offsets
        return self

    def predict(self, X: Any) -> np.ndarray:
        """関節角に対応する補正量を返す。"""
        return self._wave(self._flat(X), *np.hstack((self.amplitudes_, self.offsets_)))

    def save(self) -> dict[str, list[float]]:
        return {"periods": self.periods.tolist(), "amplitudes": self.amplitudes_.tolist(), "offsets": self.offsets_.tolist()}

    def score(self, X: Any, y: Any) -> float:
        return r2_score(self._flat(y), self.predict(X))

    def _wave(self, joints: np.ndarray, *parameters: float) -> np.ndarray:
        amplitudes, offsets = np.asarray(parameters, dtype=np.float64).reshape(2, -1)
        return np.sum(amplitudes * np.sin(2.0 * np.pi * (joints[:, None] / self.periods + offsets / 360.0)), axis=1)

    # (N, 1) で送られてくる関節角・補正量を 1 次元にする
    @staticmethod
    def _flat(values: Any) -> np.ndarray:
        return np.asarray(values, dtype=np.float64).reshape(-1)
