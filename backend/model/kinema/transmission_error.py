"""関節の角度伝達誤差（減速機の回転に同期した周期誤差）のモデル。"""
from typing import Any

import numpy as np


class TransmissionError:
    """各関節の実角度 = 指令角 + Σ A·sin(2π(θ/P + φ/360)) とする。

    周期 P は機種ごとに固定。最適化では位相の周回や振幅の符号で解が割れないよう、
    ``a·sin + b·cos`` の線形な係数で推定し、保存・表示のときだけ振幅 A と位相 φ [deg] に直す。
    """

    def __init__(self, periods: list[np.ndarray], joints: list[int] | tuple[int, ...] = ()) -> None:
        self.periods = [np.asarray(values, dtype=np.float64) for values in periods]
        self.coefficients = [np.zeros((2, values.size), dtype=np.float64) for values in self.periods]
        # 推定対象の関節（1 始まり）
        self.joints = list(joints)

    def apply(self, joints: np.ndarray) -> np.ndarray:
        """指令角 ``(N, 6)`` [deg] に伝達誤差を加えた実角度を返す。"""
        result = np.array(joints, dtype=np.float64)
        for index, (periods, (sine, cosine)) in enumerate(zip(self.periods, self.coefficients)):
            phase = 2.0 * np.pi * joints[:, index, None] / periods
            result[:, index] += np.sin(phase) @ sine + np.cos(phase) @ cosine
        return result

    # 推定対象の関節の係数だけを最適化ベクトルとして出し入れする
    def parameter_vector(self) -> np.ndarray:
        return np.concatenate([self.coefficients[joint - 1].reshape(-1) for joint in self.joints] + [np.empty(0)])

    def set_parameter_vector(self, values: np.ndarray) -> None:
        start = 0
        for joint in self.joints:
            size = self.coefficients[joint - 1].size
            self.coefficients[joint - 1] = np.asarray(values[start:start + size], dtype=np.float64).reshape(2, -1)
            start += size

    # 推定結果を人が読める名前付きの辞書にする
    def parameter_map(self, values: np.ndarray) -> dict[str, float]:
        names = [f"trans[J{joint},P{period:g}].{kind}" for joint in self.joints for kind in ("sin", "cos") for period in self.periods[joint - 1]]
        return {name: float(value) for name, value in zip(names, values, strict=True)}

    def save(self) -> dict[str, dict[str, list[float]]]:
        """軸ごとに JointCalibModel.save() と同じ形式（periods, amplitudes, offsets）で返す。"""
        return {
            f"J{index + 1}": {"periods": periods.tolist(), "amplitudes": np.hypot(sine, cosine).tolist(), "offsets": np.degrees(np.arctan2(cosine, sine)).tolist()}
            for index, (periods, (sine, cosine)) in enumerate(zip(self.periods, self.coefficients))
        }

    # 周期は機種の値を使い、振幅・位相から係数を復元する
    def load(self, parameters: dict[str, Any]) -> None:
        for name, values in parameters.items():
            amplitudes, offsets = np.asarray(values["amplitudes"], dtype=np.float64), np.radians(values["offsets"])
            self.coefficients[int(name[1:]) - 1] = np.vstack((amplitudes * np.cos(offsets), amplitudes * np.sin(offsets)))
