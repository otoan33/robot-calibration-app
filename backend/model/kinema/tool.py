"""工具先端オフセットを手先姿勢へ反映する処理。"""
import numpy as np

from ..util.Rotation import TransEulerZYXToRot


def apply_tool_offsets(poses: np.ndarray, tool_offsets: np.ndarray, tool_indices: np.ndarray) -> np.ndarray:
    """各手先姿勢へ、手先の向きで回転させた工具の XYZ オフセット [mm] を加算する。"""
    result = poses.copy()
    result[:, :3] += np.einsum("nij,nj->ni", TransEulerZYXToRot(poses[:, 3:]), tool_offsets[tool_indices])
    return result
