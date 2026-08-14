"""參數同步測試：散在三個檔案的同一組物理參數有沒有走鐘？

同一組物理參數散在 swerve_config.h（主控）、steer_config.h（轉向板）、
swerve_model.py（模擬器），改一處就要改另外兩處，否則模擬失去意義、
或者兩塊板對行程的認知不一致直接打架。CLAUDE.md 有一整張表在講這件事，
這支就是把那張表自動化。

這是唯一一支會去讀韌體原始碼的測試（純文字解析，不需要編譯器）。

    python test_params.py
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

FW_FILES = {
    "swerve_config.h": ROOT / "Esp32_wheel/include/swerve_config.h",
    "steer_config.h": ROOT / "esp32_rad/include/steer_config.h",
    "LimitBank.h": ROOT / "Esp32_wheel/lib/LimitBank/LimitBank.h",
    "SteerModule.h": ROOT / "esp32_rad/lib/SteerModule/SteerModule.h",
    "SwerveCan.h": ROOT / "shared/SwerveCan/SwerveCan.h",
}
SIM_FILE = ROOT / "sim/swerve_model.py"

# 韌體名稱 -> sim 名稱（只列名字不一樣的）
ALIAS = {
    "STEER_PID_KP_DEF": "STEER_PID_KP",
    "STEER_PID_KI_DEF": "STEER_PID_KI",
    "STEER_PID_KD_DEF": "STEER_PID_KD",
    "STEER_TICKS_PER_DEG": "STEER_TICKS_PER_DEG_NOMINAL",
}

fails = []


def check(cond, msg):
    print(("  [ OK ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def parse_defines(path):
    """抓 #define NAME <純數字>，帶 f/u/l 後綴與行尾註解都吃得下"""
    out = {}
    pat = re.compile(r"^\s*#define\s+([A-Z_][A-Z0-9_]*)\s+"
                     r"\(?\s*([-+]?\d+(?:\.\d+)?)\s*[fFuUlL]?\s*\)?\s*(?://.*|/\*.*)?$")
    for line in path.read_text(encoding="utf-8").splitlines():
        m = pat.match(line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


def parse_python(path):
    """抓模組層級的 NAME = 數字 / True / False / (數字, ...)"""
    out = {}
    num = re.compile(r"^([A-Z_][A-Z0-9_]*)\s*=\s*([-+]?\d+(?:\.\d+)?)\s*(?:#.*)?$")
    boo = re.compile(r"^([A-Z_][A-Z0-9_]*)\s*=\s*(True|False)\s*(?:#.*)?$")
    tup = re.compile(r"^([A-Z_][A-Z0-9_]*)\s*=\s*\(([^)]*)\)\s*(?:#.*)?$")
    for line in path.read_text(encoding="utf-8").splitlines():
        m = num.match(line)
        if m:
            out[m.group(1)] = float(m.group(2))
            continue
        m = boo.match(line)
        if m:
            out[m.group(1)] = 1.0 if m.group(2) == "True" else 0.0
            continue
        m = tup.match(line)
        if m:
            try:
                out[m.group(1)] = tuple(float(v) for v in m.group(2).split(","))
            except ValueError:
                pass
    return out


fw = {}
for fname, path in FW_FILES.items():
    if not path.exists():
        print(f"  [FAIL] 找不到 {path}")
        sys.exit(1)
    for k, v in parse_defines(path).items():
        fw.setdefault(k, []).append((fname, v))
sim = parse_python(SIM_FILE)

# ---------------------------------------------------------------- 1
print("\n[1] 韌體與 sim 的同名常數")
mismatch = []
n_checked = 0
for k in sorted(fw):
    sim_key = ALIAS.get(k, k)
    if sim_key not in sim or isinstance(sim[sim_key], tuple):
        continue
    n_checked += 1
    for fname, v in fw[k]:
        if abs(v - sim[sim_key]) > 1e-9:
            mismatch.append(f"{k}：{fname}={v} vs sim.{sim_key}={sim[sim_key]}")
for m in mismatch:
    print("         " + m)
check(not mismatch, f"{n_checked} 個同名常數全部一致")

# ---------------------------------------------------------------- 2
print("\n[2] 兩塊板都有的常數（不一致 = 兩塊板對行程的認知不同，會打架）")
cross = []
for k in sorted(fw):
    if len(fw[k]) < 2:
        continue
    detail = "、".join(f"{n}={v}" for n, v in fw[k])
    cross.append((k, {v for _, v in fw[k]}, detail))
for k, vals, detail in cross:
    print(f"         {k}：{detail}")
bad = [k for k, vals, _ in cross if len(vals) > 1]
check(not bad, f"{len(cross)} 個跨檔案常數全部一致")

# ---------------------------------------------------------------- 3
print("\n[3] 安裝偏移角（韌體四個 #define vs sim 的 tuple）")
fw_off = []
for suffix in ("FL", "FR", "RL", "RR"):
    key = f"STEER_MOUNT_OFFSET_{suffix}"
    fw_off.append(fw[key][0][1] if key in fw else None)
sim_off = sim.get("MOUNT_OFFSET_DEG")
print(f"         韌體 {fw_off}")
print(f"         sim  {list(sim_off) if sim_off else None}")
check(sim_off is not None and len(sim_off) == 4 and
      all(a is not None and abs(a - b) < 1e-9 for a, b in zip(fw_off, sim_off)),
      "MOUNT_OFFSET 一致")

# ---------------------------------------------------------------- 4
print("\n[4] keep-out 沒有超過行程寬度給的上限")
span = fw["STEER_ANGLE_MAX_DEG"][0][1] - fw["STEER_ANGLE_MIN_DEG"][0][1]
margin = fw["STEER_ANGLE_MARGIN_DEG"][0][1]
usable = span - 2 * margin
ko_max = (usable - 180.0) / 2.0
ko = fw["STEER_KEEPOUT_DEG"][0][1]
print(f"         可用行程 {usable:.0f}°，收緊後要 >= 180°，所以 keep-out <= {ko_max:.0f}°")
check(ko <= ko_max, f"STEER_KEEPOUT_DEG = {ko:.0f}° <= 上限 {ko_max:.0f}°"
                    "（超過的話某些方向兩個解都會被擋掉）")

# ---------------------------------------------------------------- 5
print("\n[5] 頻率相依鏈：回授不可以慢於積分")
odom_hz = fw["LOOP_ODOM_HZ"][0][1]
report_hz = fw.get("STEER_REPORT_HZ", [(None, None)])[0][1]
print(f"         LOOP_ODOM_HZ={odom_hz:.0f}  STEER_REPORT_HZ={report_hz}")
check(report_hz is not None and report_hz >= odom_hz,
      f"STEER_REPORT_HZ ({report_hz}) >= LOOP_ODOM_HZ ({odom_hz:.0f})"
      "（慢了只是拿同一筆資料重複積分）")

# ---------------------------------------------------------------- 6
print("\n[6] 慢速逼近的 PWM 要推得動馬達")
slow = fw["STEER_HOMING_SLOW_PWM"][0][1]
minmove = fw["STEER_MIN_MOVE_PWM"][0][1]
homing = fw["STEER_HOMING_PWM"][0][1]
print(f"         HOMING_PWM={homing:.0f}  SLOW_PWM={slow:.0f}  MIN_MOVE_PWM={minmove:.0f}")
check(slow == 0 or slow > minmove,
      f"SLOW_PWM ({slow:.0f}) > MIN_MOVE_PWM ({minmove:.0f})，否則二次逼近會卡住不動")
check(homing > minmove, f"HOMING_PWM ({homing:.0f}) > MIN_MOVE_PWM ({minmove:.0f})")

# ---------------------------------------------------------------- 總結
print("\n" + "=" * 64)
if fails:
    print(f"失敗 {len(fails)} 項：")
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("全部通過")
