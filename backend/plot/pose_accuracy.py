"""姿勢精度（各点の X/Y/Z 誤差）のグラフ描画。"""
import matplotlib.pyplot as plt
import numpy as np
from pydantic import BaseModel, Field

from . import png_response


class PoseAccuracyData(BaseModel):
    dx: list[float] = Field(min_length=1)
    dy: list[float] = Field(min_length=1)
    dz: list[float] = Field(min_length=1)


class PoseAccuracySeries(BaseModel):
    name: str = "After"
    color: str = "#1f77b4"
    data: PoseAccuracyData


class PoseAccuracyInput(BaseModel):
    datasets: list[PoseAccuracySeries] = Field(min_length=1)
    ylim: list[float] | None = None
    title: str = "Improvement in Pose Accuracy (in each axis direction)"
    dpi: int = 100


AXES = ("dx", "dy", "dz")


# 各系列の誤差ノルム（点ごとの位置ずれの大きさ）
def _norms(body: PoseAccuracyInput) -> list[np.ndarray]:
    return [np.linalg.norm([d.data.dx, d.data.dy, d.data.dz], axis=0) for d in body.datasets]


# 全系列の最大値に余白を付けた軸範囲（全て 0 でも範囲が潰れないよう下限を設ける）
def _limit(values) -> float:
    return max(1.2 * max(np.max(np.abs(v)) for v in values), 1e-12)


def render_each_axis(body: PoseAccuracyInput):
    """dX/dY/dZ を 3 段に分けて比較する。"""
    lim = _limit([getattr(d.data, a) for d in body.datasets for a in AXES])
    fig, axes = plt.subplots(3, 1, figsize=(9, 5.5))
    fig.suptitle(body.title, fontsize=15)
    for i, (ax, axis) in enumerate(zip(axes, AXES)):
        for d in body.datasets:
            ax.plot(getattr(d.data, axis), color=d.color, label=d.name, marker=".")
        ax.axhline(0, linestyle="--", linewidth=0.5, color="black")
        ax.grid(linestyle="--")
        ax.set_ylabel(f"d{axis[1].upper()} [mm]")
        ax.set_ylim(body.ylim or (-lim, lim))
    axes[0].legend(bbox_to_anchor=(1.02, 1.0), loc="upper left")
    axes[2].set_xlabel("Point No.")
    fig.tight_layout()
    return png_response(fig, body.dpi)


def render_each_axis_overlay(body: PoseAccuracyInput):
    """dX/dY/dZ を 1 枚に重ねて表示する。"""
    lim = _limit([getattr(d.data, a) for d in body.datasets for a in AXES])
    fig, ax = plt.subplots(figsize=(9, 3))
    fig.suptitle(body.title, fontsize=15)
    for d in body.datasets:
        for axis in AXES:
            ax.plot(getattr(d.data, axis), color=d.color, marker=".", label=f"{d.name} d{axis[1].upper()}")
    ax.axhline(0, linestyle="--", linewidth=0.5, color="black")
    ax.grid(linestyle="--")
    ax.set(ylim=(-lim, lim), xlabel="Point No.", ylabel="Error [mm]")
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left")
    fig.tight_layout()
    return png_response(fig, body.dpi)


def render_norm(body: PoseAccuracyInput):
    """点ごとの誤差ノルムを系列ごとに比較する。"""
    norms = _norms(body)
    fig, ax = plt.subplots(figsize=(9, 3))
    fig.suptitle(body.title, fontsize=15)
    for d, norm in zip(body.datasets, norms):
        ax.plot(norm, color=d.color, label=d.name, marker=".")
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left")
    ax.axhline(0, linestyle="--", linewidth=0.5, color="black")
    ax.grid(linestyle="--")
    ax.set(ylim=(-0.05, _limit(norms)), xlabel="Point No.", ylabel="Norm [mm]")
    fig.tight_layout()
    return png_response(fig, body.dpi)


def render_norm_with_bar(body: PoseAccuracyInput):
    """誤差ノルムの推移と、系列ごとの平均・最大を棒グラフで並べて示す。"""
    norms = _norms(body)
    lim = _limit(norms)
    fig, (line_ax, bar_ax) = plt.subplots(1, 2, figsize=(10, 4), gridspec_kw={"width_ratios": (2, 1)})
    fig.suptitle(body.title, fontsize=15)

    # 左: 点ごとの誤差ノルム
    for d, norm in zip(body.datasets, norms):
        line_ax.plot(norm, color=d.color, label=d.name, marker=".")
    line_ax.legend(loc="upper right", ncol=2)
    line_ax.axhline(0, linestyle="--", linewidth=0.5, color="black")
    line_ax.grid(linestyle="--")
    line_ax.set(ylim=(-0.05, lim), xlabel="Point No.", ylabel="Norm [mm]")

    # 右: 平均を棒、最大をエラーバーの上端で示し、数値も添える
    x = np.arange(len(body.datasets))
    bar_ax.bar(x, [norm.mean() for norm in norms], width=0.6, color=[d.color for d in body.datasets])
    for i, (d, norm) in enumerate(zip(body.datasets, norms)):
        mean, maximum = norm.mean(), norm.max()
        bar_ax.errorbar(i, mean, yerr=[[0], [maximum - mean]], fmt="none", ecolor=d.color, capsize=3, lw=2)
        bar_ax.text(i + 0.05, mean + 0.05, f"Ave.\n{mean:.3f}", ha="left", color=d.color)
        bar_ax.text(i - 0.05, maximum + 0.05, f"Max.\n{maximum:.3f}", ha="right", color=d.color)
    bar_ax.set_ylim((-0.05, lim))
    bar_ax.axhline(0, linewidth=0.5, color="black")
    bar_ax.set_xticks(x, [d.name for d in body.datasets])
    fig.tight_layout()
    return png_response(fig, body.dpi)


FUNCTIONS = {
    "each_axis": render_each_axis,
    "each_axis_overlay": render_each_axis_overlay,
    "norm": render_norm,
    "norm_with_bar": render_norm_with_bar,
}
