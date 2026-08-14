"""限位安全測試：確認實車不會把微動開關撞壞。

這支是三個問題的答案：
  1. 微動開關畫在對的位置嗎（對照實車俯視圖量到的角度）
  2. 逆運動學會不會把輪子逼到貼著開關
  3. 全方向操作跑一輪，開關會不會被壓下

    python test_limits.py
"""

import math
import sys

from swerve_model import (
    N, NAMES, SwerveRobot, SwerveKinematics, MOUNT_OFFSET_DEG, STRIKER_OFFSET_DEG,
    STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG, STEER_ANGLE_MARGIN_DEG,
    STEER_KEEPOUT_DEG,
    CHASSIS_WHEELBASE_M, CHASSIS_TRACK_M, MAX_WHEEL_SPEED_MPS,
    MAX_LINEAR_SPEED_MPS, MAX_ANGULAR_SPEED_RPS,
    switch_body_deg, striker_body_deg,
)

fails = []


def check(cond, msg):
    print(("  [ OK ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def norm180(d):
    return (d + 180.0) % 360.0 - 180.0


def new_kin(keepout=True):
    k = SwerveKinematics()
    k.set_chassis_param(CHASSIS_WHEELBASE_M, CHASSIS_TRACK_M, MAX_WHEEL_SPEED_MPS)
    k.set_steer_range(STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG, STEER_ANGLE_MARGIN_DEG)
    k.set_mount_offsets(MOUNT_OFFSET_DEG)
    if keepout:
        k.set_limit_keepout(STEER_KEEPOUT_DEG)
    return k


# ---------------------------------------------------------------- 1
print("\n[1] 微動開關位置：對照實車俯視圖量到的角度")
# 從實車俯視圖量出來的兩顆開關位置（車體座標，度）。這是外部事實，
# MOUNT_OFFSET 是拿它配上 CAD 的撞塊偏移 +90 反推出來的，所以這一項驗的是
# 「那組 MOUNT_OFFSET 填對了沒有」——對不上就是安裝偏移角錯了，實車會走歪。
MEASURED = {"FL": (-90.0, 180.0), "FR": (90.0, 180.0),
            "RL": (-90.0, 0.0), "RR": (90.0, 0.0)}
for i in range(N):
    lo, hi = switch_body_deg(i)
    got = sorted(round(norm180(v), 6) for v in (lo, hi))
    want = sorted(round(norm180(v), 6) for v in MEASURED[NAMES[i]])
    check(got == want, f"{NAMES[i]} 開關在車體 {got}，圖上量到 {want}")

print("\n[2] 單撞塊機構自洽性")
for i in range(N):
    s_min, s_max = switch_body_deg(i)
    # 撞塊在機械 0 度時落在 min 開關、270 度時落在 max 開關
    check(abs(norm180(striker_body_deg(i, STEER_ANGLE_MIN_DEG) - s_min)) < 1e-6
          and abs(norm180(striker_body_deg(i, STEER_ANGLE_MAX_DEG) - s_max)) < 1e-6,
          f"{NAMES[i]} 撞塊@機械0 落在 min 開關、@機械270 落在 max 開關")
    # 撞塊繞「另外那一邊」走 270 度，所以兩顆開關在車體座標上看起來只差 90 度
    check(abs(abs(norm180(s_max - s_min)) - 90.0) < 1e-6,
          f"{NAMES[i]} 兩顆開關看起來相差 {abs(norm180(s_max - s_min)):.0f} 度"
          f"（撞塊走另一邊的 {STEER_ANGLE_MAX_DEG:.0f} 度）")

print("\n[3] 死角朝向：開關裝在內側的話，90 度死角應該朝車體中心")
CENTER_DIR = {"FL": -135.0, "FR": 135.0, "RL": -45.0, "RR": 45.0}
for i in range(N):
    # 死角 = 撞塊到不了的那 90 度，中心在機械 315 度的位置
    dead_mid = norm180(striker_body_deg(i, 315.0))
    want = CENTER_DIR[NAMES[i]]
    check(abs(norm180(dead_mid - want)) < 1e-6,
          f"{NAMES[i]} 死角中心朝 {dead_mid:+.0f}°，車體中心方向 {want:+.0f}°")

# ---------------------------------------------------------------- 3
print("\n[4] 數學保證：每個行進方向都存在 clearance 夠大的解")
k = new_kin()
worst_best = 999.0
for deg10 in range(-1800, 1800):
    g = deg10 / 10.0
    for i in range(N):
        local = k.global_to_local_deg(i, g)
        best = -999.0
        for flip in (0, 1):
            for kk in range(-2, 3):
                a = local + (180.0 if flip else 0.0) + 360.0 * kk
                if k.steer_min_deg <= a <= k.steer_max_deg:
                    best = max(best, k.limit_clearance_deg(a))
        worst_best = min(worst_best, best)
check(worst_best > 41.0,
      f"最差情況下，最好的解仍有 {worst_best:.1f}° clearance（理論值 42°）")

# ---------------------------------------------------------------- 4
print("\n[5] keep-out 真的有在起作用（開 vs 關 的對照）")
for label, use_ko in (("關閉 keep-out", False), ("開啟 keep-out", True)):
    kk = new_kin(keepout=use_ko)
    worst = 999.0
    cur = [135.0] * N
    for deg2 in range(0, 720):          # 平移方向連續繞兩圈
        g = deg2 / 2.0
        vx = math.cos(math.radians(g))
        vy = math.sin(math.radians(g))
        st = kk.inverse(vx, vy, 0.0, cur)
        cur = [s[0] for s in st]
        worst = min(worst, min(kk.limit_clearance_deg(a) for a in cur))
    print(f"        {label:<16} 繞兩圈過程中最小 clearance = {worst:6.2f}°")
    if use_ko:
        check(worst > 20.0, f"開啟後 clearance 保持在 {worst:.1f}° > 20°")
    else:
        no_ko_worst = worst
check(no_ko_worst < 5.0,
      f"關閉時會貼到 {no_ko_worst:.2f}°，證明這個機制不是裝飾用的")

# ---------------------------------------------------------------- 5
print("\n[6] 全系統：全方向操作跑一輪，微動開關不可以被壓下")
bot = SwerveRobot()
dt = 1.0 / 200.0
v = MAX_LINEAR_SPEED_MPS * 0.7
w = MAX_ANGULAR_SPEED_RPS * 0.6


def sweep(t):
    if t < 12.0:
        a = math.radians(t / 12.0 * 720.0)
        return v * math.cos(a), v * math.sin(a), 0.0
    if t < 20.0:
        a = math.radians((t - 12.0) / 8.0 * 360.0)
        return v * math.cos(a), v * math.sin(a), w * math.sin(a * 2.0)
    if t < 23.0:
        return 0.0, 0.0, w
    if t < 26.0:
        return 0.0, 0.0, -w
    step = int((t - 26.0) / 0.5)
    a = math.radians(step * 137.0)
    return v * math.cos(a), v * math.sin(a), 0.0


t = 0.0
while t < 30.0:
    bot.set_cmd_vel(*sweep(t))
    bot.step(dt)
    t += dt

print(f"        30 秒掃描：最小 clearance {bot.worst_clearance():.2f}°，"
      f"開關觸發 {bot.total_switch_hits()} 次")
check(bot.total_switch_hits() == 0, "全程沒有壓到任何一顆微動開關")
check(bot.worst_clearance() > STEER_ANGLE_MARGIN_DEG,
      f"全程最小 clearance {bot.worst_clearance():.2f}° > 安全邊界 "
      f"{STEER_ANGLE_MARGIN_DEG}°")
check(not bot.out_of_range(), "沒有模組跑出 0~270 度實體行程")

# ---------------------------------------------------------------- 6
print("\n[7] 極端操作：亂送指令也不可以撞開關")
bot2 = SwerveRobot()
# 用固定種子的偽亂數，每次跑結果一樣，出事才能重現
seed = 12345
for step in range(int(20.0 / dt)):
    if step % 40 == 0:  # 每 0.2 秒換一次方向，逼出最多的換解
        seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
        a = (seed % 3600) / 10.0
        seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
        rot = ((seed % 2000) / 1000.0 - 1.0) * w
        bot2.set_cmd_vel(v * math.cos(math.radians(a)),
                         v * math.sin(math.radians(a)), rot)
    bot2.step(dt)
print(f"        20 秒亂數操作：最小 clearance {bot2.worst_clearance():.2f}°，"
      f"開關觸發 {bot2.total_switch_hits()} 次")
check(bot2.total_switch_hits() == 0, "亂數操作全程沒有壓到微動開關")

# ---------------------------------------------------------------- 7
print("\n[8] 軟限位：直接下達貼著開關的目標角度，也要煞得住")
bot3 = SwerveRobot()
for m in bot3.steer:
    m.angle_deg = 40.0
    m.target_deg = 40.0
for _ in range(int(3.0 / dt)):
    for m in bot3.steer:
        m.set_target_deg(STEER_ANGLE_MIN_DEG)  # 故意叫它去撞 0 度開關
        m.update(dt)
stop = [m.angle_deg for m in bot3.steer]
print(f"        叫模組直接衝 0 度開關，最後停在 {stop[0]:.2f}°")
check(all(m.switch_hits == 0 for m in bot3.steer),
      f"軟限位把它擋在 {stop[0]:.2f}°，沒有壓到開關")

# ---------------------------------------------------------------- 總結
print("\n" + "=" * 64)
if fails:
    print(f"失敗 {len(fails)} 項：")
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("全部通過")
