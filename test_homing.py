"""開機歸零測試：跑完之後韌體以為的角度，和真實角度差多少？

歸零是唯一會「故意」把模組往微動開關頂的時候，也是決定「直線走不走得直」的
根源，所以單獨拉出來測。模擬器裡刻意讓硬體和韌體的認知不一致：

    HW_TICKS_PER_DEG    真實刻度（比韌體估算值大 4%）
    HW_BACKLASH_DEG     齒隙 1.2 度
    HW_HARD_STOP_...    開關之後還有 2.5 度才頂到硬擋塊
    LimitLink           開關接在主控板，要經過 5 ms 去彈跳 + CAN + 200 Hz 取樣
                        才到轉向板；逼近越快，這段延遲換算的誤差越大

    python test_homing.py
"""

import math
import sys

import swerve_model as M
from swerve_model import N, NAMES, SwerveRobot

fails = []


def check(cond, msg):
    print(("  [ OK ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def run_homing(bot, limit_s=90.0, dt=1.0 / 200.0):
    """跑到歸零結束（或故障 / 逾時），回傳花了幾秒"""
    t = 0.0
    while t < limit_s:
        bot.step(dt)
        t += dt
        if bot.homing_done or bot.homing_fault:
            break
    return t


# 四個很不一樣的起點，第 3、4 個是開機就壓在開關上（最容易出事）
START = [200.0, 60.0, 269.5, 0.2]

# ---------------------------------------------------------------- 1
print("\n[1] 從各種起點開機都要能歸零完成")
bot = SwerveRobot()
bot.start_homing(START)
t = run_homing(bot)
print(f"        起點 {START}，花了 {t:.1f} 秒")
check(bot.homing_done, "四顆都完成歸零")
check(not bot.homing_fault, "沒有任何模組判故障")
check(all(m.hard_stop_hits == 0 for m in bot.steer),
      f"沒有頂到硬擋塊（{[m.hard_stop_hits for m in bot.steer]}）")

# ---------------------------------------------------------------- 2
print("\n[2] 歸零後：韌體以為的角度 vs 真實角度")
errs = [m.home_error_deg for m in bot.steer]
for i in range(N):
    m = bot.steer[i]
    print(f"        {NAMES[i]}  韌體 {m.angle_deg:7.2f}°  真實 {m.true_deg:7.2f}°  "
          f"誤差 {m.home_error_deg:+6.3f}°")
worst = max(abs(e) for e in errs)
check(worst < 1.0, f"最大歸零誤差 {worst:.3f}° < 1°")
spread = max(errs) - min(errs)
check(spread < 0.5,
      f"四顆之間的誤差差異 {spread:.3f}° < 0.5°（差異才會讓輪子互相拉扯）")

# ---------------------------------------------------------------- 3
print("\n[3] 刻度校正：碰第二顆開關把 ticks/度 修回來")
nom = M.STEER_TICKS_PER_DEG_NOMINAL
hw = M.HW_TICKS_PER_DEG
got = bot.steer[0].ticks_per_deg
print(f"        估算 {nom:.3f}  ->  實測 {got:.3f}  （真實 {hw:.3f}）")
print(f"        校正前偏差 {(nom / hw - 1) * 100:+.2f}%  ->  "
      f"校正後 {(got / hw - 1) * 100:+.2f}%")
check(abs(got / hw - 1) < abs(nom / hw - 1) / 4,
      "刻度誤差至少改善 4 倍")
check(all(m.measured_span_deg == 270.0 for m in bot.steer),
      "四顆都完成了行程量測")

# ---------------------------------------------------------------- 4
print("\n[4] 二次逼近有沒有用（關掉來對照）")
saved = M.STEER_HOMING_SLOW_PWM
M.STEER_HOMING_SLOW_PWM = 0          # 只快速撞一次
bot_fast = SwerveRobot()
bot_fast.start_homing(START)
run_homing(bot_fast)
M.STEER_HOMING_SLOW_PWM = saved
fast_worst = max(abs(m.home_error_deg) for m in bot_fast.steer)
print(f"        只快速撞一次：最大誤差 {fast_worst:.3f}°")
print(f"        二次逼近    ：最大誤差 {worst:.3f}°")
check(bot_fast.homing_done, "關掉二次逼近也還是能完成")

# ⚠ 這一項故意只驗「不會更糟」，不驗「一定更準」。
#
# 二次逼近在實車上的主要價值是**重複性**：快速撞上去的瞬間列印件會被壓變形、
# 齒隙被吃掉，讀到的位置每次都不一樣（steer_config.h 的註解就是這樣寫的）。
# 那個效應模擬器沒有建 —— 這裡只有齒隙與硬擋塊，沒有「撞擊力道 -> 變形」。
#
# 模擬器唯一能反映的好處是「速度慢 -> 訊號延遲換算的角度小」，而那一段已經被
# 延遲補償扣掉了（見第 5 項）。減速比從猜的 4.5 改成實測的 9.0 之後，快速逼近
# 本身也只剩 26 度/秒，兩者都落在補償能力之內，差異就掉進齒隙的雜訊裡了。
#
# 所以這裡驗的是「開著它不會讓事情變糟」。真要評估它值不值得，得上實車量
# 重複性（同一顆模組連續歸零十次，看零點散佈），模擬器給不出那個答案。
check(worst <= fast_worst + 0.05,
      f"二次逼近沒有讓精度變糟（{worst:.3f}° vs 只撞一次 {fast_worst:.3f}°，"
      f"差異在齒隙雜訊內）")

# ---------------------------------------------------------------- 5
print("\n[5] 開關走 CAN 的延遲補償有沒有用（關掉來對照）")
# 微動開關接在主控板，狀態經 CAN 才到轉向板，比直接接在轉向板多 5~10 ms。
# 慢速逼近 27.7 度/秒，每 1 ms 就是 0.028 度。訊框帶 age、轉向板用當下角速度
# 回推來補掉（SteerModule::contact_tick）。
#
# ⚠ 不要拿「park 角度的誤差」來評估這件事，那裡量不到東西：
#   兩顆開關的接觸點都被修正 d 度，zero_tick 往一邊移、實測刻度往另一邊改，
#   而 park 剛好在行程正中間(135° = 270/2)，兩者在那裡**恰好抵銷**。
#   推導：angle₁(p) ≈ angle₀(p) − d + p·(2d/270)，代 p=135 得 angle₁ = angle₀。
#
# 所以這是一個「刻度」修正而不是「零點」修正，效果要在遠離 park 的地方看。
# 行程兩端才是輪子實際會去的位置，那裡的誤差 ≈ 刻度誤差 x 270。
M.STEER_LIMIT_AGE_COMPENSATE = False
bot_nocomp = SwerveRobot()
bot_nocomp.start_homing(START)
run_homing(bot_nocomp)
M.STEER_LIMIT_AGE_COMPENSATE = True

hw = M.HW_TICKS_PER_DEG
comp_scale = sum(m.ticks_per_deg / hw - 1 for m in bot.steer) / N * 100
noc_scale = sum(m.ticks_per_deg / hw - 1 for m in bot_nocomp.steer) / N * 100
span = M.STEER_ANGLE_MAX_DEG - M.STEER_ANGLE_MIN_DEG

print(f"        不補償：刻度誤差 {noc_scale:+.3f}%  -> 行程末端角度誤差 "
      f"{noc_scale / 100 * span:+.3f}°")
print(f"        有補償：刻度誤差 {comp_scale:+.3f}%  -> 行程末端角度誤差 "
      f"{comp_scale / 100 * span:+.3f}°")
print(f"        （park 135° 的誤差兩者相同是預期的：{worst:.3f}° vs "
      f"{max(abs(m.home_error_deg) for m in bot_nocomp.steer):.3f}°）")
check(bot_nocomp.homing_done, "關掉補償也還是能完成（只是刻度比較不準）")
check(abs(comp_scale) < abs(noc_scale),
      f"補償之後刻度比較準（{abs(comp_scale):.3f}% < {abs(noc_scale):.3f}%）")
# 剩下的殘差是齒隙造成的：min 端量到 true-半齒隙、max 端量到 true+半齒隙，
# 行程就多算了一整個齒隙。這是機構問題，軟體補不掉。
backlash_scale = M.HW_BACKLASH_DEG / span * 100
print(f"        殘差 {comp_scale:+.3f}% 幾乎就是齒隙貢獻的 {backlash_scale:+.3f}%"
      f"（{M.HW_BACKLASH_DEG}° 齒隙 / {span:.0f}° 行程），軟體補不掉")

# ---------------------------------------------------------------- 6
print("\n[6] 減速比填錯時，行程合理性檢查要攔下來")
saved_hw = M.HW_TICKS_PER_DEG
M.HW_TICKS_PER_DEG = M.STEER_TICKS_PER_DEG_NOMINAL * 2.0  # 實際是估算的兩倍
bot_bad = SwerveRobot()
bot_bad.start_homing(START)
run_homing(bot_bad)
measured = [round(m.measured_tpd, 2) if hasattr(m, 'measured_tpd') else None
            for m in bot_bad.steer]
M.HW_TICKS_PER_DEG = saved_hw
check(bot_bad.homing_fault,
      "實際刻度是估算值的 2 倍 -> 判故障，不會帶著錯誤刻度去跑")
check(not bot_bad.ready, "故障時 ready=False，主控不會給驅動輪出力")

# ---------------------------------------------------------------- 7
print("\n[7] 歸零期間主控不可以驅動輪子")
bot2 = SwerveRobot()
bot2.start_homing(START)
bot2.set_cmd_vel(M.MAX_LINEAR_SPEED_MPS, 0.0, 0.0)  # 一開機就狂催油門
moved = 0.0
t = 0.0
while t < 90.0 and not (bot2.homing_done or bot2.homing_fault):
    bot2.step(1 / 200)
    t += 1 / 200
    moved = max(moved, math.hypot(*bot2.true_pose[:2]))
print(f"        歸零過程中車體位移 {moved * 1000:.2f} mm")
check(moved < 0.001, "歸零沒完成前車子完全沒動")

# ---------------------------------------------------------------- 8
print("\n[8] 歸零之後跑直線，里程計要跟得上")
bot2.set_cmd_vel(0.8, 0.0, 0.0)
for _ in range(int(6.0 * 200)):
    bot2.step(1 / 200)
dist = math.hypot(*bot2.true_pose[:2])
err = bot2.odom_error()
print(f"        走了 {dist:.3f} m，里程計誤差 {err * 100:.2f} cm "
      f"({err / max(dist, 1e-9) * 100:.2f}%)")
check(bot2.ready, "歸零完成後進入 READY")
check(err / max(dist, 1e-9) < 0.05, "里程計誤差 < 行走距離的 5%")
lateral = abs(bot2.true_pose[1])
print(f"        橫向偏移 {lateral * 100:.2f} cm（歸零誤差造成的『走不直』）")

# ---------------------------------------------------------------- 9
print("\n[9] 歸零完成後沒人下指令，輪子要留在 park 不可以自己跑掉")
# 迴歸測試。曾經發生過：inverse() 內部的 seed 只在第一次呼叫時做，而那一刻
# （開機 +20 ms）轉向板還沒開始歸零、回報角度接近 0，last_angle_deg_ 就被
# seed 成 constrain(0, 3, 267) = 3 度。零速時 inverse() 走「沿用上一次角度」
# 的捷徑、不經過 resolve_angle()，keep-out 完全沒有機會介入，於是歸零完成
# 停在 park 的輪子會被一路拉回開機當下的角度，最靠近的那顆只離開關 3.8 度。
# 修法是在 READY 的邊沿重新 seed（bot.cpp loop_bot_control() 的 1.5 步）。
bot3 = SwerveRobot()
bot3.start_homing(START)          # 起點含 269.5 與 0.2 兩個貼著開關的
run_homing(bot3)
check(bot3.ready, "先確認有進 READY（不然下面測不到東西）")
for _ in range(int(5.0 * 200)):   # /cmd_vel 完全不下，維持零速 5 秒
    bot3.step(1 / 200)
park_err = max(abs(m.angle_deg - M.STEER_HOMING_PARK_DEG) for m in bot3.steer)
clear = min(m.limit_clearance_deg for m in bot3.steer)
print(f"        零速 5 秒後：{[f'{m.angle_deg:.1f}' for m in bot3.steer]}"
      f"（park {M.STEER_HOMING_PARK_DEG:.0f}°）")
print(f"        最小 clearance {clear:.1f}°")
check(park_err < 2.0, f"四顆都還在 park 附近，最大偏離 {park_err:.2f}° < 2°")
check(clear >= M.STEER_KEEPOUT_DEG,
      f"最小 clearance {clear:.1f}° >= keep-out {M.STEER_KEEPOUT_DEG:.0f}°")

# ---------------------------------------------------------------- 總結
print("\n" + "=" * 64)
if fails:
    print(f"失敗 {len(fails)} 項：")
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("全部通過")
