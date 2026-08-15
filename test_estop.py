"""急停測試：暴衝的時候按下去，車子要停，而且不可以自己再跑起來。

急停開關接在主控板的 GPIO 上，觸發後主控用 CAN 命令四顆 VESC 煞車
（對應 bot.cpp 的 loop_bot_estop / estop_brake_vescs）。

這支測的是**閂鎖與重新致能的邏輯**，那是最容易寫錯、而且寫錯就會讓車子
自己跑掉的部分。煞車物理只用一個較短的時間常數帶過，這裡不是在驗證煞車距離。

轉向板不在測試範圍內：它是直流馬達，有自己獨立的急停迴路，急停期間只是
因為速度指令被壓成零而凍結在原地。

    python test_estop.py
"""

import math
import sys

import swerve_model as M
from swerve_model import SwerveRobot

fails = []


def check(cond, msg):
    print(("  [ OK ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def run(bot, seconds, dt=1.0 / 200.0):
    for _ in range(int(seconds / dt)):
        bot.step(dt)


def make_running_bot(speed=0.8):
    """做一台已經歸零完成、正在全速前進的車"""
    bot = SwerveRobot()
    bot.set_cmd_vel(speed, 0.0, 0.0)
    run(bot, 3.0)  # 等它加速到穩態
    return bot


# ---------------------------------------------------------------- 1
print("\n[1] 全速前進時按下急停，車子要停住")
bot = make_running_bot()
v_before = max(abs(v) for v in bot.wheel_act_mps)
print(f"        按下前輪速 {v_before:.3f} m/s")
check(v_before > 0.5, "前置條件：車子確實在跑")

bot.set_estop(True)
run(bot, 0.5)
v_after = max(abs(v) for v in bot.wheel_act_mps)
print(f"        按下 0.5 秒後輪速 {v_after:.4f} m/s")
check(bot.estop_latched, "急停已閂鎖")
check(v_after < 0.01, f"輪速歸零（{v_after:.4f} < 0.01 m/s）")

# ---------------------------------------------------------------- 2
print("\n[2] 急停期間 /cmd_vel 繼續催，也不可以動")
bot.set_cmd_vel(1.0, 0.0, 2.0)  # 上層導航完全不知道有人按了急停
run(bot, 2.0)
v = max(abs(v) for v in bot.wheel_act_mps)
print(f"        催了 2 秒，輪速還是 {v:.4f} m/s")
check(v < 0.01, "急停期間 /cmd_vel 完全無效")
check(bot.estop_latched, "急停仍然閂鎖著")

# ---------------------------------------------------------------- 3
print("\n[3] 放開開關 -> 馬上解除，但殘留的 /cmd_vel 不算數")
# ESTOP_REARM_REQUIRE_ZERO=False 的行為。危險點沒有消失，只是換人擋：
# 解除本身不再等 /cmd_vel 歸零（等零速在手動操作時會變成「怎麼按都不動」），
# 改由「解除瞬間把急停前那筆指令作廢」來防止暴衝。
bot.set_estop(False)
run(bot, 1.0)
v = max(abs(v) for v in bot.wheel_act_mps)
print(f"        放開 1 秒（急停前是全速前進），輪速 {v:.4f} m/s")
check(not bot.estop_latched, "放開就解除，不用等 /cmd_vel 歸零")
check(v < 0.01, "殘留指令被作廢，車子沒有自己跑掉")

# ---------------------------------------------------------------- 4
print("\n[4] 解除後要有**新的** /cmd_vel 才會動")
bot.set_cmd_vel(0.6, 0.0, 0.0)
run(bot, 2.0)
v = max(abs(v) for v in bot.wheel_act_mps)
print(f"        重新下 0.6 m/s，輪速 {v:.3f} m/s")
check(v > 0.3, "解除後恢復正常驅動")

# ---------------------------------------------------------------- 5
print("\n[5] 切回 ESTOP_REARM_REQUIRE_ZERO=True：要等零速才解除")
# 這條路徑跑 nav2 時會用（上層不會因為有人按急停就停發 /cmd_vel），
# config 隨時可以切回去，所以兩條路徑都要有測試守著
_saved = M.ESTOP_REARM_REQUIRE_ZERO
M.ESTOP_REARM_REQUIRE_ZERO = True
try:
    bot5 = make_running_bot()
    bot5.set_estop(True)
    run(bot5, 0.5)
    check(bot5.estop_latched, "前置條件：已閂鎖")

    bot5.set_estop(False)
    bot5.set_cmd_vel(1.0, 0.0, 2.0)   # 上層還在催全速
    run(bot5, 3.0)                    # 遠超過 ESTOP_REARM_CMD_ZERO_MS
    v = max(abs(v) for v in bot5.wheel_act_mps)
    print(f"        放開 3 秒但 cmd_vel 仍為全速，輪速 {v:.4f} m/s")
    check(bot5.estop_latched, "cmd_vel 沒歸零就不解除閂鎖")
    check(v < 0.01, "車子沒有自己跑掉")

    bot5.set_cmd_vel(0.0, 0.0, 0.0)
    half = M.ESTOP_REARM_CMD_ZERO_MS / 1000.0 * 0.5
    run(bot5, half)
    print(f"        零速維持 {half:.2f} 秒（不足 "
          f"{M.ESTOP_REARM_CMD_ZERO_MS / 1000.0:.2f} 秒）")
    check(bot5.estop_latched, "時間不夠，還不解除")

    run(bot5, M.ESTOP_REARM_CMD_ZERO_MS / 1000.0)
    print(f"        再等到超過 {M.ESTOP_REARM_CMD_ZERO_MS / 1000.0:.2f} 秒")
    check(not bot5.estop_latched, "條件滿足，解除閂鎖")
finally:
    M.ESTOP_REARM_REQUIRE_ZERO = _saved

# ---------------------------------------------------------------- 6
print("\n[6] 急停期間轉向角度要凍結，不可以亂甩")
bot2 = make_running_bot()
bot2.set_estop(True)
run(bot2, 0.3)
ang_at_stop = [m.angle_deg for m in bot2.steer]
# 按著急停的同時給一個會讓輪子大轉向的指令
bot2.set_cmd_vel(0.0, 1.0, 0.0)
run(bot2, 2.0)
ang_later = [m.angle_deg for m in bot2.steer]
drift = max(abs(a - b) for a, b in zip(ang_at_stop, ang_later))
print(f"        急停中下橫移指令 2 秒，轉向角最大變化 {drift:.3f}°")
check(drift < 2.0, f"轉向角凍結（變化 {drift:.3f}° < 2°）")

# ---------------------------------------------------------------- 7
print("\n[7] 開機時就按著急停 -> 車子完全不會動")
bot3 = SwerveRobot()
bot3.set_estop(True)
run(bot3, 0.1)          # 讓去彈跳跑完
bot3.set_cmd_vel(1.0, 0.0, 0.0)
run(bot3, 3.0)
moved = math.hypot(*bot3.true_pose[:2])
print(f"        一開機就催全速，車體位移 {moved * 1000:.2f} mm")
check(bot3.estop_latched, "開機即閂鎖")
check(moved < 0.001, "車子完全沒動")

# ---------------------------------------------------------------- 8
print("\n[8] 解除方向的去彈跳要比觸發方向長（抗接觸不良）")
print(f"        觸發 {M.ESTOP_TRIP_DEBOUNCE_MS:.0f} ms / "
      f"解除 {M.ESTOP_CLEAR_DEBOUNCE_MS:.0f} ms")
check(M.ESTOP_CLEAR_DEBOUNCE_MS > M.ESTOP_TRIP_DEBOUNCE_MS,
      "解除去彈跳比觸發長，雜訊不會讓車子自己跑掉")

# 開關抖一下（短於解除去彈跳）不可以被當成「放開了」
bot4 = make_running_bot()
bot4.set_estop(True)
run(bot4, 0.2)
bounce_s = M.ESTOP_CLEAR_DEBOUNCE_MS / 1000.0 * 0.5
bot4.set_cmd_vel(0.0, 0.0, 0.0)
run(bot4, 2.0)          # 先讓 cmd_vel 歸零計時跑滿，把其他條件排除掉
bot4.set_estop(False)   # 假裝接觸不良，短暫斷開
run(bot4, bounce_s)
bot4.set_estop(True)
run(bot4, 0.05)
print(f"        急停線抖動 {bounce_s * 1000:.0f} ms（短於解除去彈跳）")
check(bot4.estop_latched, "抖動沒有被誤判成放開")

# ---------------------------------------------------------------- 總結
print("\n" + "=" * 64)
if fails:
    print(f"失敗 {len(fails)} 項：")
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("全部通過")
