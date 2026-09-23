from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class KinemaInput:
    joints: np.ndarray
    tool_indices: np.ndarray
    sequence_ids: np.ndarray | None = None
    times: np.ndarray | None = None


# X の列構成 (N, 6) / (N, 7) / (N, 9) = 6 列 + tool_id（1 始まり）+ sequence_id, time を分解する
def parse_kinema_input(values: Any) -> KinemaInput:
    array = np.asarray(values, dtype=np.float64)
    tool_indices = array[:, 6].astype(np.int64) - 1 if array.shape[1] in (7, 9) else np.zeros(len(array), dtype=np.int64)
    if array.shape[1] == 9:
        return KinemaInput(array[:, :6], tool_indices, array[:, 7].astype(np.int64), array[:, 8])
    return KinemaInput(array[:, :6], tool_indices)
