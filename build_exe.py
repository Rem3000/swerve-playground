"""把模擬器打包成單一 exe，給沒裝 Python 的電腦用。

    pip install pyinstaller
    python build_exe.py

產物：dist/SwerveSim.exe（單檔，約 10 MB，雙擊即可執行）
"""

import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        print("找不到 pyinstaller，先執行：pip install pyinstaller")
        return 1

    args = [
        sys.executable, "-m", "PyInstaller",
        "--onefile",              # 打成單一檔案
        "--windowed",             # GUI 程式，不要開主控台視窗
        "--name", "SwerveSim",
        "--clean",
        "--noconfirm",
        # swerve_model 是同目錄的模組，PyInstaller 會自動跟著打包，
        # 但明講一次比較保險（改成 package 結構時才不會漏）
        "--hidden-import", "swerve_model",
        "--paths", HERE,
        os.path.join(HERE, "swerve_sim.py"),
    ]
    print(" ".join(args) + "\n")
    rc = subprocess.call(args, cwd=HERE)
    if rc != 0:
        print("\n打包失敗")
        return rc

    exe = os.path.join(HERE, "dist", "SwerveSim.exe")
    if os.path.exists(exe):
        print(f"\n完成：{exe}  ({os.path.getsize(exe) / 1e6:.1f} MB)")
        print("這一個檔案就能單獨帶走，目標電腦不需要裝 Python。")
    else:
        print("\n打包指令跑完了，但找不到 dist/SwerveSim.exe")
        return 1

    # build/ 和 spec 是中間產物，留著只會佔空間
    shutil.rmtree(os.path.join(HERE, "build"), ignore_errors=True)
    spec = os.path.join(HERE, "SwerveSim.spec")
    if os.path.exists(spec):
        os.remove(spec)
    return 0


if __name__ == "__main__":
    sys.exit(main())
