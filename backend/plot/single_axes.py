"""単軸の角度伝達誤差と、その空間周波数スペクトルのグラフ描画。"""
import matplotlib.pyplot as plt
import numpy as np
from pydantic import BaseModel, Field

from . import png_response


class SingleAxisData(BaseModel):
    angle: list[float] = Field(min_length=2)
    error: list[float] = Field(min_length=2)


class SingleAxisSeries(BaseModel):
    name: str = "Before"
    color: str = "#1f77b4"
    data: SingleAxisData


class SingleAxisInput(BaseModel):
    datasets: list[SingleAxisSeries] = Field(min_length=1)
    joint_no: int = Field(default=1, ge=1, le=6)
    gear_rate: float = Field(default=120.0, gt=0)
    title: str = "Single-axis angular transmission error"
    frequency_limit: float = Field(default=3.0, gt=0)
    dpi: int = Field(default=100, ge=1, le=600)


# 角度に対する誤差の片側振幅スペクトル（angle は等間隔で増加している前提）
def exec_fft(angle: np.ndarray, error: np.ndarray):
    n = len(angle)
    return np.fft.fftfreq(n, angle[1] - angle[0])[: n // 2], 2.0 / n * np.abs(np.fft.fft(error))[: n // 2]


def render_angle_error(body: SingleAxisInput):
    """角度伝達誤差と、減速比由来の周波数を示した FFT スペクトルを上下に並べる。"""
    series = [(d, np.asarray(d.data.angle, dtype=float), np.asarray(d.data.error, dtype=float)) for d in body.datasets]
    lim = max(1.2 * max(np.max(np.abs(error)) for _, _, error in series), 1e-12)
    fig, (error_ax, fft_ax) = plt.subplots(2, 1, figsize=(9, 6), constrained_layout=True)
    fig.suptitle(f"{body.title} (J{body.joint_no})", fontsize=14)

    # 上: 角度ごとの誤差（両端 25% ずつを切り落として中央部を表示する）
    for d, angle, error in series:
        error_ax.plot(angle, error, color=d.color, label=d.name)
    error_ax.axhline(0, linestyle="--", linewidth=0.5, color="black")
    error_ax.set(xlabel="Angle [deg]", ylabel="dAngle [deg]", ylim=(-lim, lim))
    error_ax.grid(linestyle="--")
    error_ax.legend(loc="upper right")
    xmin, xmax = error_ax.get_xlim()
    error_ax.set_xlim(xmin + (xmax - xmin) * 0.25, xmax - (xmax - xmin) * 0.25)

    # 下: スペクトルと、減速比から決まる 1 次・2 次の周波数
    for d, angle, error in series:
        fft_ax.plot(*exec_fft(angle, error), color=d.color, marker=".", label=d.name)
    fft_ax.axvline(body.gear_rate / 180.0, color="red", linestyle="--", linewidth=1, label="gear frequency")
    fft_ax.axvline(body.gear_rate / 90.0, color="red", linestyle="--", linewidth=1)
    fft_ax.set(xlim=(0, body.frequency_limit), xlabel="Frequency [1/deg]", ylabel="Amplitude [deg]")
    fft_ax.grid(linestyle="--")
    fft_ax.legend(loc="upper right")
    return png_response(fig, body.dpi)


FUNCTIONS = {"angle_error": render_angle_error}
