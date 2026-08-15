#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""掃 DRIVE_REPOINT_ANGLE_DEG：「變向多大才值得把車停下來」該設幾度。

這不是喜好問題，是一個有兩端代價的取捨：

  門檻太小 -> 稍微修個方向就整台車停下來重來（減速 + 轉向 + 重新加速 約一秒），
              手動操作變成一路停停走走
  門檻太大 -> 大幅換向時輪子邊滾邊轉，輪胎橫著刮地，車體被側向力推歪

所以要同時量兩件事，再看拐點在哪：

  側向滑移 (m/s)  輪胎真的在地上被橫拖的速度。用**實際指向**和**車體實際速度**
                  算，不是用指令 —— 輪胎不是被指令刮的，是被車體的動量刮的。
  停滯時間 (s)    過程中車速低於 0.05 m/s 的總時間，也就是「被迫停下來」多久。

掃的是幾種不同大小的變向，因為門檻的意義就是「多大的變向要特別對待」。

    python3 tune_repoint.py
"""

import math
import sys

import swerve_model as M
from swerve_model import SwerveRobot

DT = 1.0 / 200.0
CRUISE = 0.4          # 巡航速度 (m/s)，手動操作的常用速度
STOPPED = 0.05        # 低於這個速度算「停滯」


def run(bot, seconds):
    for _ in range(int(seconds / DT)):
        bot.step(DT)


def module_scrub(bot, i):
    """模組 i 的側向滑移速度：車體在該模組位置的實際速度，投影到垂直於
    輪子**實際指向**的方向。"""
    vx, vy, wz = bot.true_vel
    mx, my = bot.kin.module_x[i], bot.kin.module_y[i]
    vxi = vx - wz * my
    vyi = vy + wz * mx
    a = math.radians(bot.reported_deg[i] + bot.physical_offset_deg[i])
    return abs(vxi * math.sin(a) - vyi * math.cos(a))


def body_speed(bot):
    return math.hypot(bot.true_vel[0], bot.true_vel[1])


def trial(threshold_deg, turn_deg):
    """巡航中把行進方向轉 turn_deg 度，回傳 (最大側向滑移, 停滯秒數)"""
    saved = M.DRIVE_REPOINT_ANGLE_DEG
    M.DRIVE_REPOINT_ANGLE_DEG = threshold_deg
    try:
        bot = SwerveRobot()
        bot.set_cmd_vel(0.0, 0.0, 0.0)
        run(bot, 2.0)
        bot.set_cmd_vel(CRUISE, 0.0, 0.0)
        run(bot, 4.0)                      # 先穩定巡航

        r = math.radians(turn_deg)
        bot.set_cmd_vel(CRUISE * math.cos(r), CRUISE * math.sin(r), 0.0)

        worst = 0.0
        stalled = 0.0
        for _ in range(int(4.0 / DT)):
            bot.step(DT)
            worst = max(worst, max(module_scrub(bot, i) for i in range(M.N)))
            if body_speed(bot) < STOPPED:
                stalled += DT
        return worst, stalled
    finally:
        M.DRIVE_REPOINT_ANGLE_DEG = saved


TURNS = [15, 30, 45, 60, 90]
THRESHOLDS = [8, 20, 30, 45, 60, 90]

print(f"\n巡航 {CRUISE} m/s，在行進中改變方向。")
print("每格 = 最大側向滑移 (m/s) / 被迫停滯時間 (s)\n")

header = "  門檻\\變向 " + "".join(f"{t:>16}°" for t in TURNS)
print(header)
print("  " + "-" * (len(header) - 2))

rows = {}
for th in THRESHOLDS:
    cells = []
    rows[th] = []
    for turn in TURNS:
        scrub, stall = trial(th, turn)
        rows[th].append((scrub, stall))
        cells.append(f"{scrub:>8.3f} /{stall:>6.2f}")
    label = f"{th:>3}°"
    if th >= 90:
        label += "*"
    print(f"  {label:>8}  " + "".join(f"{c:>17}" for c in cells))

print("\n  * 90° 等於關閉停車行為（靠 θ/θ+180，需要的轉角永遠 <= 90°）")

# ------------------------------------------------------------------ 判讀
print("\n---- 判讀 ----")
for th in THRESHOLDS:
    worst_scrub = max(s for s, _ in rows[th])
    total_stall = sum(t for _, t in rows[th])
    # 小變向(<=30°)的停滯時間才是「手感卡不卡」的關鍵
    small_stall = sum(t for (_, t), turn in zip(rows[th], TURNS) if turn <= 30)
    print(f"  門檻 {th:>3}°：最大滑移 {worst_scrub:.3f} m/s   "
          f"總停滯 {total_stall:5.2f} s   小變向(<=30°)停滯 {small_stall:5.2f} s")

print("""
  怎麼選：小變向的停滯時間要接近 0（那是手動操作最常做的事，停下來完全
  沒必要），同時大變向的滑移要壓在可接受範圍。看上面哪個門檻是拐點。
""")


# ==========================================================================
#  第二輪：到位閘門 (DRIVE_ALIGN_GATE_DEG) 在**連續操作**時會不會一直誤觸發
#
#  上面那張表顯示大變向的停滯其實是閘門造成的，不是停車邏輯。所以真正決定
#  手感的是「手在連續轉方向的時候，閘門會不會一直關」。
#  模擬手動操作：讓行進方向以固定角速度持續旋轉，量閘門關著的時間比例。
# ==========================================================================
def sweep_scenario(gate_deg, release_deg, rate_dps, seconds=6.0):
    """行進方向以 rate_dps 持續旋轉，回傳 (閘門關閉時間比例, 最大滑移, 平均車速)"""
    g0, r0 = M.DRIVE_ALIGN_GATE_DEG, M.DRIVE_ALIGN_RELEASE_DEG
    M.DRIVE_ALIGN_GATE_DEG, M.DRIVE_ALIGN_RELEASE_DEG = gate_deg, release_deg
    try:
        bot = SwerveRobot()
        bot.set_cmd_vel(0.0, 0.0, 0.0)
        run(bot, 2.0)
        bot.set_cmd_vel(CRUISE, 0.0, 0.0)
        run(bot, 4.0)

        closed = 0.0
        worst = 0.0
        speed_sum = 0.0
        n = int(seconds / DT)
        for k in range(n):
            ang = math.radians(rate_dps * k * DT)
            bot.set_cmd_vel(CRUISE * math.cos(ang), CRUISE * math.sin(ang), 0.0)
            bot.step(DT)
            if bot.gate_stop:
                closed += DT
            worst = max(worst, max(module_scrub(bot, i) for i in range(M.N)))
            speed_sum += body_speed(bot)
        return closed / seconds, worst, speed_sum / n
    finally:
        M.DRIVE_ALIGN_GATE_DEG, M.DRIVE_ALIGN_RELEASE_DEG = g0, r0


print("\n" + "=" * 78)
print("到位閘門：手在連續轉方向時，閘門關著的時間比例")
print(f"（巡航 {CRUISE} m/s，行進方向以固定角速度持續旋轉）\n")
RATES = [20, 45, 90, 180]
GATES = [(8.0, 4.0), (15.0, 8.0), (25.0, 12.0), (40.0, 20.0)]

hdr = "  閘門/放行    " + "".join(f"{r:>17}°/s" for r in RATES)
print(hdr)
print("  " + "-" * (len(hdr) - 2))
for g, r in GATES:
    cells = []
    for rate in RATES:
        frac, scrub, spd = sweep_scenario(g, r, rate)
        cells.append(f"{frac * 100:>5.0f}% {scrub:>6.3f} {spd:>5.2f}")
    print(f"  {g:>4.0f}°/{r:<4.0f}°  " + "".join(f"{c:>20}" for c in cells))

print("\n  每格 = 閘門關閉時間比例 / 最大側向滑移(m/s) / 平均車速(m/s)")
print("""
  判讀：閘門關閉比例高 = 一直在斷油 = 手感卡頓，而且平均車速會掉下來。
  但放太寬滑移就會上升。要挑「關閉比例低、滑移還沒明顯變壞」的那一組。

  ⚠ 上面這個場景是讓方向連續轉好幾圈，會反覆跨過 θ / θ+180 的等價解邊界，
    每跨一次就是一次 180 度翻面。人操作搖桿不會這樣轉，所以下面用比較真實的
    場景（在有限範圍內來回擺）再測一次，並且把翻面次數單獨數出來。
""")


# ==========================================================================
#  第三輪：比較貼近真人操作 —— 方向在有限範圍內來回擺，並數 180 度翻面
# ==========================================================================
def realistic(amp_deg, period_s, seconds=12.0):
    """行進方向在 ±amp_deg 之間以 period_s 的週期來回擺（正弦）。

    回傳 (閘門關閉比例, 翻面次數, 平均車速, 最大滑移)
    """
    bot = SwerveRobot()
    bot.set_cmd_vel(0.0, 0.0, 0.0)
    run(bot, 2.0)
    bot.set_cmd_vel(CRUISE, 0.0, 0.0)
    run(bot, 4.0)

    closed = 0.0
    speed_sum = 0.0
    worst = 0.0
    flips = 0
    prev = list(bot.target_deg)
    n = int(seconds / DT)
    for k in range(n):
        ang = math.radians(amp_deg * math.sin(2 * math.pi * k * DT / period_s))
        bot.set_cmd_vel(CRUISE * math.cos(ang), CRUISE * math.sin(ang), 0.0)
        bot.step(DT)
        if bot.gate_stop:
            closed += DT
        speed_sum += body_speed(bot)
        worst = max(worst, max(module_scrub(bot, i) for i in range(M.N)))
        # 目標角一拍之內跳超過 90 度。
        # ⚠ 這不一定是「換到另一組等價解」——「先停車再轉」的凍結解除時，
        #   目標角也會一次跳到位。實測 keep-out 與換解遲滯都完全改變不了這個
        #   數字，所以在這個場景裡它數到的其實是 repoint 的提交次數。
        if max(abs(a - b) for a, b in zip(bot.target_deg, prev)) > 90.0:
            flips += 1
        prev = list(bot.target_deg)
    return closed / seconds, flips, speed_sum / n, worst


print("\n" + "=" * 78)
print("貼近真人操作：方向在 ±振幅 之間來回擺")
print(f"（巡航 {CRUISE} m/s，目前設定 GATE={M.DRIVE_ALIGN_GATE_DEG}° "
      f"REPOINT={M.DRIVE_REPOINT_ANGLE_DEG}°）\n")
print("   振幅   週期    閘門關閉   大跳次數   平均車速   最大滑移")
print("  " + "-" * 62)
for amp, period in ((20, 4.0), (20, 2.0), (45, 4.0), (45, 2.0), (90, 4.0), (90, 2.0)):
    frac, flips, spd, scrub = realistic(amp, period)
    print(f"  ±{amp:>3}°  {period:>4.1f}s   {frac * 100:>7.0f}%   {flips:>7}   "
          f"{spd:>7.2f}   {scrub:>7.3f}")
print(f"\n  （指令車速是 {CRUISE} m/s，平均車速離它越近代表越不卡）")


# ==========================================================================
#  第四輪：±90 度大幅掃動的元凶是 180 度翻面，而翻面次數由「可用窗口寬度」
#  決定。窗口 = 264 - 2*keepout，所以 keep-out 是可以拿來換平順度的旋鈕。
#  代價是輪子會更靠近微動開關。
# ==========================================================================
def keepout_trial(keepout_deg, amp_deg=90, period_s=4.0, seconds=12.0):
    saved = M.STEER_KEEPOUT_DEG
    M.STEER_KEEPOUT_DEG = keepout_deg
    try:
        return realistic(amp_deg, period_s, seconds)
    finally:
        M.STEER_KEEPOUT_DEG = saved


print("\n" + "=" * 78)
print("keep-out 對 180° 翻面的影響（±90° 來回掃，週期 4 s）")
print("  窗口寬度 = 264 - 2*keepout；窗口越寬，解越不容易被擠出去而被迫翻面\n")
print("  keepout   窗口寬度   閘門關閉   大跳次數   平均車速   離開關最近")
print("  " + "-" * 66)
for ko in (0, 10, 20, 30, 42):
    frac, flips, spd, scrub = keepout_trial(ko)
    width = 264 - 2 * ko
    print(f"  {ko:>5}°   {width:>6.0f}°   {frac * 100:>7.0f}%   {flips:>7}   "
          f"{spd:>7.2f}   {ko:>7}°")
print("""
  ⚠ keep-out 是三層限位防護的第一層，調小 = 輪子正常運轉時會更靠近微動開關。
    撞壞開關要拆車，所以這個旋鈕不要為了手感隨便動 —— 先確認 ±90° 那種
    大幅掃動是不是真的常用動作。
""")


# ==========================================================================
#  第五輪：翻面不是窗口寬度造成的（keep-out 完全沒影響），是 resolve_angle
#  「挑最近的解」在兩解等距處就翻 —— 即使不翻也還留在行程內。
#  遲滯就是用來壓這個的，量一下要多大才壓得住，以及有沒有副作用。
# ==========================================================================
def hyst_trial(h, amp_deg, period_s=4.0, seconds=12.0):
    saved = M.STEER_FLIP_HYSTERESIS_DEG
    M.STEER_FLIP_HYSTERESIS_DEG = h
    try:
        return realistic(amp_deg, period_s, seconds)
    finally:
        M.STEER_FLIP_HYSTERESIS_DEG = saved


print("\n" + "=" * 78)
print("換解遲滯 STEER_FLIP_HYSTERESIS_DEG 對 180° 翻面的影響")
print("  遲滯 = 「另一組解要近上這麼多度才值得換」。夠大就只在被行程逼著時才翻。\n")
for amp in (90, 120):
    print(f"  --- 方向 ±{amp}° 來回掃（週期 4 s）---")
    print("   遲滯   閘門關閉   大跳次數   平均車速   最大滑移")
    print("  " + "-" * 52)
    for h in (0, 30, 60, 90, 120):
        frac, flips, spd, scrub = hyst_trial(float(h), amp)
        print(f"  {h:>4}°   {frac * 100:>7.0f}%   {flips:>7}   {spd:>7.2f}   {scrub:>7.3f}")
    print()
print("""  判讀：翻面次數掉下來、平均車速回升，就是遲滯壓住了「不必要的翻面」。
  若某個遲滯值反而讓滑移變大，代表它把該翻的也擋掉了，過頭了。""")
