"""たわみ補正キネマ（CorrectedKinema）に関節の角度伝達誤差を加えたキャリブレーションモデル。"""
from typing import Any

import numpy as np

from ..observation import observation_type
from .input import KinemaInput
from .kinema_model import KinemaModel
from .tool import apply_tool_offsets
from .transmission_error import TransmissionError

ALL_JOINTS = (1, 2, 3, 4, 5, 6)


class KinemaJointModel(KinemaModel):
    """関節 J1〜J6 の角度が周期的な伝達誤差を持つとして、キネマと伝達誤差を同定する。

    ``pattern`` は ``TRAIN_PATTERNS`` の名前。各段階は (キネマも推定するか, 伝達誤差を推定する関節) で、
    キネマを推定する段階では ``calibration_mode``（all_actual など）の幾何パラメータを動かす。
    """

    TRAIN_PATTERNS = {
        "trans_j1": [(False, (1,))],
        "trans_all": [(False, ALL_JOINTS)],
        "kinema_trans_j1": [(True, (1,))],
        "kinema_trans_all": [(True, ALL_JOINTS)],
        "kinema_then_trans_j1": [(True, ()), (False, (1,))],
        "kinema_then_trans_all": [(True, ()), (False, ALL_JOINTS)],
    }

    def __init__(self, pattern: str = "kinema_trans_all", stages: list[tuple[bool, list[int]]] | None = None, **settings: Any) -> None:
        # 伝達誤差は補正キネマの関節角に乗せるため、mode は actual に固定する
        super().__init__(**{**settings, "mode": "actual"})
        self.kinema_mode = self.kinema.calibration_mode
        # stages を直接渡せば、名前付きパターン以外の組み合わせも試せる
        self.stages = stages or self.TRAIN_PATTERNS[pattern]
        self.transmission = TransmissionError(self.params.trans_err_periods)

    # 段階ごとに推定対象を切り替えて、KinemaModel の同時最小二乗を順に行う
    def fit(self, X: Any, y: Any) -> "KinemaJointModel":
        for use_kinema, joints in self.stages:
            self.kinema.set_calibration_mode(self.kinema_mode if use_kinema else "none")
            self.transmission.joints = list(joints)
            super().fit(X, y)
        return self

    def save(self) -> dict[str, Any]:
        return {"robot_model": self._save_robot(), "transmission_error": self.transmission.save(), "observation_model": {"type": observation_type(self.observation), "parameters": self.observation.save()}}

    # キネマのみの保存ファイルも読めるよう、伝達誤差はあるときだけ反映する
    def load(self, parameters: dict[str, Any]) -> None:
        super().load(parameters)
        self.transmission.load(parameters.get("transmission_error", {}))

    # 指令角に伝達誤差を加えた実角度で順運動学を解く
    def _predict_positions(self, data: KinemaInput) -> np.ndarray:
        return apply_tool_offsets(self.kinema.predict(self.transmission.apply(data.joints)), self.tool_offsets, data.tool_indices)[:, :3]

    # 最適化ベクトルは [キネマの補正対象, 伝達誤差の係数] の順
    def _get_robot_parameters(self, tool_parameter_indices: np.ndarray) -> np.ndarray:
        return np.hstack((self.kinema._get_calibration_parameters(), self.transmission.parameter_vector()))

    def _set_robot_parameters(self, values: np.ndarray, tool_parameter_indices: np.ndarray) -> None:
        kinema_count = len(self.kinema._get_calibration_parameters())
        self.kinema._set_calibration_parameters(values[:kinema_count])
        self.transmission.set_parameter_vector(values[kinema_count:])

    def _parameter_map(self, values: np.ndarray, robot_count: int, tool_parameter_indices: np.ndarray) -> dict[str, float]:
        kinema_count = len(self.kinema._get_calibration_parameters())
        result = self.kinema._calibration_parameter_map(values[:kinema_count])
        result.update(self.transmission.parameter_map(values[kinema_count:robot_count]))
        result.update({f"observation[{index}]": float(value) for index, value in enumerate(values[robot_count:])})
        return result
