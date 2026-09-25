from typing import Any

import numpy as np

from ..util.Rotation import TransEulerZYXToRot, TransRotToEulerZYX
from .base import ObservationModel


class RigidObservationModel(ObservationModel):
    """ロボット座標の手先位置を、計測器の座標系へ剛体変換した値を観測値とする（計測値 = R·p + T）。

    パラメータは ``[x, y, z, roll, pitch, yaw]``（mm, ZYX オイラー角 deg）で、ロボットのパラメータと同時に推定する。
    """

    output_size = 3

    def __init__(self, transform: Any = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), **_: Any) -> None:
        self.transform_ = np.asarray(transform, dtype=np.float64)

    def transform(self, positions: np.ndarray, sequence_ids: np.ndarray | None, times: np.ndarray | None) -> np.ndarray:
        return np.asarray(positions, dtype=np.float64) @ TransEulerZYXToRot(self.transform_[3:]).T + self.transform_[:3]

    def initialize(self, positions: np.ndarray, measured: np.ndarray) -> None:
        """対応する点の組から、最小二乗で最もよく重なる剛体変換（Kabsch 法）を初期値にする。"""
        positions_center, measured_center = positions.mean(axis=0), measured.mean(axis=0)
        u, _, vt = np.linalg.svd((positions - positions_center).T @ (measured - measured_center))
        # 鏡映にならないよう、行列式が負なら最小特異値の軸を反転する
        vt[-1] *= np.sign(np.linalg.det(vt.T @ u.T))
        rotation = vt.T @ u.T
        self.transform_ = np.hstack((measured_center - rotation @ positions_center, TransRotToEulerZYX(rotation)))

    def parameter_vector(self) -> np.ndarray:
        return self.transform_.copy()

    def set_parameter_vector(self, values: np.ndarray) -> None:
        self.transform_ = np.asarray(values, dtype=np.float64).copy()

    def save(self) -> dict[str, Any]:
        return {"transform": self.transform_.tolist()}

    def load(self, parameters: dict[str, Any]) -> None:
        self.__init__(**parameters)
