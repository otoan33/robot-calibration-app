"""直線軌跡精度（進行方向に対する横ずれ）のグラフ描画。"""
import matplotlib.pyplot as plt
import numpy as np
from pydantic import BaseModel, Field

from . import png_response


class PathAccuracySeries(BaseModel):
    name: str
    color: str
    pos_measured: list[list[float]]
    pos_reference: list[list[float]]


class PathAccuracyInput(BaseModel):
    datasets: list[PathAccuracySeries] = Field(min_length=1)
    title: str = "Improvement in Path Accuracy"
    dpi: int = 100


# 計測軌跡を「進行方向の位置」と「進行方向に直交する 2 成分のずれ」に分ける
def _split_trajectory(measured: np.ndarray, reference: np.ndarray):
    moving = np.flatnonzero(np.abs(reference[-1] - reference[0]) > 10)

    # 3 軸とも動く斜め移動: 始点→終点を x 軸とする座標系に回転して評価する
    if len(moving) == 3:
        nx = (measured[-1] - measured[0]) / np.linalg.norm(measured[-1] - measured[0])
        ny = np.cross([0, 0, 1], nx)
        ny /= np.linalg.norm(ny)
        converted = (measured - measured[0]) @ np.array([nx, ny, np.cross(nx, ny)]).T
        return converted[:, 0], converted[:, 1:]

    # 軸方向の移動: 始点を指令値に合わせ、他の 2 軸は始点と終点を結ぶ直線からのずれを取る
    main = moving[0]
    others = [a for a in range(3) if a != main]
    pos_main = measured[:, main] - (measured[0, main] - reference[0, main])
    rate = (pos_main - pos_main[0]) / (pos_main[-1] - pos_main[0])
    subs = measured[:, others] - measured[0, others] - np.outer(rate, measured[-1, others] - measured[0, others])
    return pos_main, subs


def render_straight_path(body: PathAccuracyInput):
    """進行方向に沿った横ずれ 2 成分と、横ずれ平面上の軌跡・最大ずれ円を描く。"""
    # 軸ラベルは 1 つ目の系列の指令軌跡で最も大きく動く軸を基準に決める
    first_ref = np.asarray(body.datasets[0].pos_reference, dtype=float)
    main_axis = int(np.argmax(np.abs(first_ref[-1] - first_ref[0])))
    names = [n for i, n in enumerate("XYZ") if i != main_axis]

    fig, ax = plt.subplot_mosaic([["dx", "xy"], ["dy", "xy"]], figsize=(12, 4), gridspec_kw={"width_ratios": [4.5, 1.1], "wspace": 0.22, "hspace": 0.5})
    ax_dif, ax_xy = [ax["dx"], ax["dy"]], ax["xy"]
    fig.suptitle(body.title, fontsize=14)
    circle = np.radians(np.arange(360))
    radii = []
    for d in body.datasets:
        main, subs = _split_trajectory(np.asarray(d.pos_measured, dtype=float), np.asarray(d.pos_reference, dtype=float))
        radius = np.max(np.linalg.norm(subs, axis=1))
        radii.append(radius)
        for i in range(2):
            ax_dif[i].plot(main, subs[:, i], color=d.color, linewidth=1.5, label=d.name)
        # 横ずれ平面での軌跡と、最大ずれを半径とする円
        ax_xy.plot(subs[:, 0], subs[:, 1], linewidth=0.5, color=d.color)
        ax_xy.plot(radius * np.cos(circle), radius * np.sin(circle), linewidth=0.5, color=d.color)

    for a, name in zip(ax_dif, names):
        a.set(ylabel=f"d{name} [mm]", xlabel=f"{'XYZ'[main_axis]} [mm]", ylim=(-0.5, 0.5))
        a.grid()
        a.legend(loc="lower right", ncols=3)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.set(xlabel=f"d{names[0]} [mm]", ylabel=f"d{names[1]} [mm]")
    ax_xy.set_title("\n".join(f"{d.name}: {r:.3f} [mm]" for d, r in zip(body.datasets, radii)), fontsize=12)
    fig.tight_layout()
    return png_response(fig, body.dpi)


FUNCTIONS = {"straight_path": render_straight_path}
