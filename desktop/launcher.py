"""Windows exe 用のエントリポイント。描画 API を別スレッド、解析 API を別プロセス、frontend をメインスレッドで起動する。"""
import argparse
import multiprocessing
import os
import threading

import uvicorn
from nicegui import ui

if __name__ in {"__main__", "__mp_main__"}:
    # PyInstaller でビルドした exe でマルチプロセスを正しく動かすため（子プロセスはここで処理されて終わる）
    multiprocessing.freeze_support()

    # ポートの変更とブラウザの自動起動をオプションで切り替える
    parser = argparse.ArgumentParser(description="robot-calibration")
    parser.add_argument("--backend-port", type=int, default=8000)
    parser.add_argument("--analysis-port", type=int, default=8001)
    parser.add_argument("--frontend-port", type=int, default=8080)
    parser.add_argument("--no-browser", action="store_true", help="ブラウザを自動で開かない")
    args = parser.parse_args()

    # frontend が import 時に接続先を読むため、先に設定してからページを登録する
    os.environ["DRAW_URL"] = f"http://127.0.0.1:{args.backend_port}"
    os.environ["ANALYSIS_URL"] = f"http://127.0.0.1:{args.analysis_port}"
    import frontend.main  # noqa: F401
    from backend.analysis import serve as serve_analysis
    from backend.draw import app

    # 解析 API は学習が重く GIL を占有するため別プロセスで動かし、描画を止めないようにする
    multiprocessing.Process(target=serve_analysis, args=(args.analysis_port,), daemon=True).start()

    # 描画 API はバックグラウンドで動かし、frontend の終了と一緒に止まるようにする
    threading.Thread(target=uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=args.backend_port)).run, daemon=True).start()
    ui.run(host="127.0.0.1", port=args.frontend_port, title="robot-calibration", reload=False, show=not args.no_browser)
