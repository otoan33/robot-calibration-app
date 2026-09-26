"""手先位置の目標から関節角を求める処理（ロボット側の補正と、逐次最適化の軌道修正で共用）。"""
from collections.abc import Callable

import numpy as np

# 数値微分の刻み [deg]
STEP = 1e-4


def position_jacobian(forward: Callable[[np.ndarray], np.ndarray], joints: np.ndarray) -> np.ndarray:
    """関節角 ``(N, 6)`` での手先位置の偏微分 ``(N, 3, 6)`` [mm/deg] を、全点まとめて中心差分で求める。"""
    columns = []
    for axis in range(6):
        delta = np.zeros(6)
        delta[axis] = STEP
        columns.append((forward(joints + delta) - forward(joints - delta)) / (2 * STEP))
    return np.stack(columns, axis=-1)


def solve_joints(forward: Callable[[np.ndarray], np.ndarray], targets: np.ndarray, joints: np.ndarray, iterations: int = 5) -> np.ndarray:
    """``forward(関節角)`` が目標の手先位置 ``(N, 3)`` になる関節角を、初期値からの変化が最小になるように解く（ガウス・ニュートン法）。"""
    joints = np.array(joints, dtype=np.float64)
    for _ in range(iterations):
        joints += np.einsum("nij,nj->ni", np.linalg.pinv(position_jacobian(forward, joints)), targets - forward(joints))
    return joints
