"""公称 DH パラメータによる 6 軸ロボットの順運動学。"""
from typing import Any

import numpy as np

from ..util.Rotation import TransRotToEulerZYX
from .kinema_parameter import KinemaModelParam


class IdealKinema:
    """公称 DH パラメータで順運動学を計算し、各関節の原点角度（と工具オフセット）を補正対象にする。"""

    # 旧 KinemaCalib の理想キネマモード。どのモードも DH パラメータの θ オフセット（平坦化添字）を補正する
    CALIBRATION_INDICES_BY_MODE = dict.fromkeys(("local", "origin", "local_tool", "origin_tool", "local_ideal", "origin_ideal"), (3, 7, 11, 15, 19, 23))
    # 工具オフセットも同時に推定するモード
    TOOL_OFFSET_MODES = frozenset(("local_tool", "origin_tool"))

    def __init__(self, params: KinemaModelParam, calibration_mode: str = "local", max_nfev: int | None = None) -> None:
        self.params = params
        self.calibration_mode = calibration_mode.lower()
        self.calibration_parameter_indices = self.CALIBRATION_INDICES_BY_MODE[self.calibration_mode]
        self.optimize_tool_offsets = self.calibration_mode in self.TOOL_OFFSET_MODES
        self.max_nfev = max_nfev
        self.calibration_result_: dict[str, Any] | None = None

    def predict(self, joints: np.ndarray) -> np.ndarray:
        """関節角 ``(N, 6)`` [deg] から手先姿勢 ``(N, 6)`` [mm, deg] を返す。"""
        return np.vstack([self.forward(joint) for joint in joints])

    def forward(self, joint: np.ndarray) -> np.ndarray:
        """1 組の関節角から、DH パラメータの同次変換を順に掛けて手先姿勢を計算する。"""
        # J4, J6 は回転方向の定義が DH と逆向き
        joint_vector = joint * np.array([1, 1, 1, -1, 1, -1], dtype=np.float64)
        transform = np.eye(4, dtype=np.float64)
        for index in range(7):
            alpha = np.radians(self.params.dh_param[index, 1])
            theta = np.radians(self.params.dh_param[index, 3] + (joint_vector[index] if index < 6 else 0.0))
            ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
            transform = transform @ np.array([
                [ct, -st, 0, self.params.dh_param[index, 0]],
                [ca * st, ca * ct, -sa, -self.params.dh_param[index, 2] * sa],
                [sa * st, sa * ct, ca, self.params.dh_param[index, 2] * ca],
                [0, 0, 0, 1],
            ], dtype=np.float64)
        return np.hstack((transform[:3, 3], TransRotToEulerZYX(transform[:3, :3])))

    # 最適化で動かすパラメータ（DH の補正対象 + 推定する工具オフセット）を 1 本のベクトルにする
    def _get_calibration_parameters(self, tool_offsets: np.ndarray, tool_parameter_indices: np.ndarray) -> np.ndarray:
        geometry = np.array([self.params.dh_param[index // 4, index % 4] for index in self.calibration_parameter_indices], dtype=np.float64)
        return np.hstack((geometry, tool_offsets[tool_parameter_indices].reshape(-1)))

    def _set_calibration_parameters(self, values: np.ndarray, tool_offsets: np.ndarray, tool_parameter_indices: np.ndarray) -> None:
        geometry_count = len(self.calibration_parameter_indices)
        for index, value in zip(self.calibration_parameter_indices, values[:geometry_count], strict=True):
            self.params.dh_param[index // 4, index % 4] = value
        tool_offsets[tool_parameter_indices] = values[geometry_count:].reshape(-1, 3)

    # 推定結果を人が読める名前付きの辞書にする
    def _calibration_parameter_map(self, values: np.ndarray, tool_parameter_indices: np.ndarray) -> dict[str, float]:
        geometry_count = len(self.calibration_parameter_indices)
        parameters = {f"dh[{index // 4},{index % 4}]": float(value) for index, value in zip(self.calibration_parameter_indices, values[:geometry_count], strict=True)}
        parameters.update({
            f"tool_offsets[{tool_index},{axis}]": float(value)
            for tool_index, offset in zip(tool_parameter_indices, values[geometry_count:].reshape(-1, 3), strict=True)
            for axis, value in enumerate(offset)
        })
        return parameters
