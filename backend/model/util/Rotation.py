"""回転行列と ZYX オイラー角の変換。"""
import numpy as np


def TransEulerZYXToRot(euler: np.ndarray) -> np.ndarray:
    """ZYX オイラー角 ``[roll, pitch, yaw]`` [deg] を回転行列へ変換する。"""
    roll, pitch, yaw = np.radians(np.asarray(euler, dtype=np.float64))
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([
        [cr * cp, cr * sp * sy - sr * cy, cr * sp * cy + sr * sy],
        [sr * cp, sr * sp * sy + cr * cy, sr * sp * cy - cr * sy],
        [-sp, cp * sy, cp * cy],
    ], dtype=np.float64)


def TransRotToEulerZYX(rotation: np.ndarray) -> np.ndarray:
    """回転行列を ZYX オイラー角 ``[roll, pitch, yaw]`` [deg] に変換する。"""
    matrix = np.asarray(rotation, dtype=np.float64)
    pitch = np.degrees(np.arctan2(-matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0])))
    cosine_pitch = np.cos(np.radians(pitch))

    # pitch = ±90° の特異姿勢では roll と yaw を分離できないため roll = 0 とする
    if np.isclose(abs(pitch), 90.0, atol=1e-9):
        roll = 0.0
        yaw = np.degrees(np.arctan2(matrix[0, 1], matrix[1, 1]) if pitch > 0.0 else np.arctan2(-matrix[0, 1], matrix[1, 1]))
    else:
        roll = np.degrees(np.arctan2(matrix[1, 0] / cosine_pitch, matrix[0, 0] / cosine_pitch))
        yaw = np.degrees(np.arctan2(matrix[2, 1] / cosine_pitch, matrix[2, 2] / cosine_pitch))
    return np.array([roll, pitch, yaw], dtype=np.float64)
