"""ロボット API のシミュレータ（:8002）。実機のロボット API も同じ仕様（/info, /parameters, /run）で作る。

真のロボットは公称キネマに乱数の幾何誤差・剛性率・伝達誤差を加えたもので、計測は計測器の座標・時刻で返す。
環境変数 SIM_SEED で真値の乱数、SIM_BACKLASH で同定モデルにないバックラッシの大きさ（倍率、0 で無し）を変えられる。
"""
import os
from typing import Any

import numpy as np
import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel

from backend.model.kinema.compensation import solve_joints
from backend.model.kinema.corrected_kinema import CorrectedKinema
from backend.model.kinema.kinema_joint_model import KinemaJointModel
from backend.model.util.Rotation import TransEulerZYXToRot

ROBOT = {"robot_type": "R6E", "tool_offsets": [[50.0, 0.0, 100.0]]}
# 計測器の座標（ロボット → 計測器の剛体変換 [mm, deg]）・時計のずれ [s]・サンプリング周期 [s]・ノイズ [mm]
MEASURE_TRANSFORM = np.array([1500.0, -300.0, 200.0, 30.0, 0.5, -0.3])
CLOCK_OFFSET, BT_PERIOD, NOISE = 12.345, 0.01, 0.003
# 同定モデルにない誤差：動作方向で向きが変わる関節の角度ずれ [deg]
BACKLASH = np.array([0.004, 0.004, 0.004, 0.006, 0.006, 0.008]) * float(os.environ.get("SIM_BACKLASH", "1"))

app = FastAPI(title="robot simulator API")
rng = np.random.default_rng(int(os.environ.get("SIM_SEED", "0")))

# 真のロボット：同定対象のパラメータ（ベース位置以外の幾何・剛性率・伝達誤差）を公称値からずらす
true_robot = KinemaJointModel(**ROBOT)
for index in (i for i in CorrectedKinema.CALIBRATION_INDICES_BY_MODE["all_actual"] if i >= 6):
    true_robot.params.geometry_param_corr[index // 6, index % 6] += rng.normal(0.0, 0.3 if index % 6 < 3 else 0.03)
true_robot.params.stiffness_rate[1:5] *= 1.0 + rng.normal(0.0, 0.1, 4)
true_robot.transmission.coefficients = [rng.normal(0.0, 0.003, values.shape) for values in true_robot.transmission.coefficients]

# 指令の基準になる公称キネマと、/parameters で受け取ったパラメータで補正する制御側のキネマ
nominal = KinemaJointModel(**ROBOT)
controller = KinemaJointModel(**ROBOT)


class RunRequest(BaseModel):
    time: list[float]
    joints: list[list[float]]


@app.get("/info")
def info():
    return ROBOT


# 補正パラメータ（KinemaJointModel / TrajectoryCalibModel の save() 形式）を制御側に設定する
@app.post("/parameters")
def parameters(request: dict[str, Any]):
    controller.load(request)
    return {"status": "loaded"}


@app.post("/run")
def run(request: RunRequest):
    time, command = np.asarray(request.time), np.asarray(request.joints)
    # 制御側：補正キネマの手先が公称キネマの手先（指令）に一致する関節角をモータへ出す
    motor = solve_joints(controller.predict_positions, nominal.predict_positions(command), command)
    # 実機：動作方向に応じたバックラッシを受けた関節角で、真のキネマの手先が動く
    actual = motor - BACKLASH * np.tanh(np.gradient(motor, time, axis=0) / 1.0)
    positions = true_robot.predict_positions(actual)
    # 計測器：自分の周期・時計で、計測器の座標系の位置をノイズ付きで記録する
    bt_time = np.arange(time[0], time[-1], BT_PERIOD)
    xyz = np.column_stack([np.interp(bt_time, time, values) for values in positions.T]) @ TransEulerZYXToRot(MEASURE_TRANSFORM[3:]).T + MEASURE_TRANSFORM[:3]
    return {"fm": {"time": time.tolist(), "joints": motor.tolist()}, "bt": {"time": (bt_time + CLOCK_OFFSET).tolist(), "xyz": (xyz + rng.normal(0.0, NOISE, xyz.shape)).tolist()}}


# exe のランチャーから別プロセスで起動するための入口
def serve(port: int):
    uvicorn.run(app, host="127.0.0.1", port=port)


if __name__ == "__main__":
    serve(8002)
