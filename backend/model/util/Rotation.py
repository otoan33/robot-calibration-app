"""回転行列と ZYX オイラー角の変換（先頭に点の次元 ``(N, ...)`` が付いていてもまとめて変換する）。"""
import numpy as np


def TransEulerZYXToRot(euler: np.ndarray) -> np.ndarray:
    """ZYX オイラー角 ``[roll, pitch, yaw]`` [deg] を回転行列へ変換する。"""
    roll, pitch, yaw = np.moveaxis(np.radians(np.asarray(euler, dtype=np.float64)), -1, 0)
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rows = [
        [cr * cp, cr * sp * sy - sr * cy, cr * sp * cy + sr * sy],
        [sr * cp, sr * sp * sy + cr * cy, sr * sp * cy - cr * sy],
        [-sp, cp * sy, cp * cy],
    ]
    return np.stack([np.stack(row, axis=-1) for row in rows], axis=-2)


def TransRotToEulerZYX(rotation: np.ndarray) -> np.ndarray:
    """回転行列を ZYX オイラー角 ``[roll, pitch, yaw]`` [deg] に変換する。"""
    matrix = np.asarray(rotation, dtype=np.float64)
    pitch = np.degrees(np.arctan2(-matrix[..., 2, 0], np.hypot(matrix[..., 0, 0], matrix[..., 1, 0])))

    # pitch = ±90° の特異姿勢では roll と yaw を分離できないため roll = 0 とする
    singular = np.isclose(np.abs(pitch), 90.0, atol=1e-9)
    roll = np.where(singular, 0.0, np.degrees(np.arctan2(matrix[..., 1, 0], matrix[..., 0, 0])))
    yaw = np.where(singular, np.degrees(np.arctan2(np.where(pitch > 0.0, 1.0, -1.0) * matrix[..., 0, 1], matrix[..., 1, 1])), np.degrees(np.arctan2(matrix[..., 2, 1], matrix[..., 2, 2])))
    return np.stack((roll, pitch, yaw), axis=-1)
