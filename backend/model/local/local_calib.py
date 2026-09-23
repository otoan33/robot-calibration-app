"""基準座標系へ変換するローカル座標キャリブレーションモデル。"""
from typing import Any

import numpy as np
from scipy.optimize import leastsq

from ..base import BaseModel, r2_score
from ..util.Rotation import TransEulerZYXToRot, TransRotToEulerZYX


class LocalCalibModel(BaseModel):
    """実測座標を基準座標へ変換する 6 自由度の剛体変換（XYZ + ZYX オイラー角）を推定する。

    ``fit(X, y)`` では ``X`` に実測座標 ``(N, 3)``、``y`` に基準座標 ``(N, 3)`` または平坦化した ``3N`` 要素を渡す。
    """

    def __init__(self, **_: Any) -> None:
        self.parameters_: np.ndarray | None = None

    def fit(self, X: Any, y: Any) -> "LocalCalibModel":
        measured = np.asarray(X, dtype=np.float64)
        reference = np.asarray(y, dtype=np.float64).reshape(len(measured), 3)
        result, _ = leastsq(self._residuals, np.zeros(6, dtype=np.float64), args=(reference, measured))
        self.parameters_ = np.asarray(result, dtype=np.float64)
        return self

    def predict(self, X: Any) -> np.ndarray:
        """実測座標を基準座標へ変換する。"""
        return self._transform(np.asarray(X, dtype=np.float64), self.parameters_)

    def inverse_predict(self, X: Any) -> np.ndarray:
        """基準座標を実測側の座標系へ逆変換する。"""
        rotation = TransEulerZYXToRot(self.parameters_[3:])
        return (rotation.T @ (np.asarray(X, dtype=np.float64) - self.parameters_[:3]).T).T

    def score(self, X: Any, y: Any) -> float:
        """全座標成分をまとめた決定係数 R² を返す。"""
        return r2_score(np.asarray(y, dtype=np.float64).reshape(-1), self.predict(X).reshape(-1))

    def calc_local_3points(self, origin: Any, x_point: Any, y_point: Any) -> np.ndarray:
        """原点・X 軸上・Y 軸上の 3 点からローカル座標系 [x, y, z, roll, pitch, yaw] を組み立てる。"""
        point0, point_x, point_y = (np.asarray(value, dtype=np.float64) for value in (origin, x_point, y_point))
        x_axis = point_x - point0
        normal = np.cross(x_axis, point_y - point0)
        x_axis /= np.linalg.norm(x_axis)
        normal /= np.linalg.norm(normal)
        return np.hstack((point0, TransRotToEulerZYX(np.column_stack((x_axis, np.cross(normal, x_axis), normal)))))

    @staticmethod
    def _residuals(parameters: np.ndarray, reference: np.ndarray, measured: np.ndarray) -> np.ndarray:
        return (reference - LocalCalibModel._transform(measured, parameters)).reshape(-1)

    @staticmethod
    def _transform(points: np.ndarray, parameters: np.ndarray) -> np.ndarray:
        return parameters[:3] + (TransEulerZYXToRot(parameters[3:]) @ points.T).T
