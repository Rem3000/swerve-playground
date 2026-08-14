"""一致性測試：swerve_model.py 的移植和韌體 C++ 真的是同一套公式嗎？

其他五支測試都建立在「sim 是韌體的 1:1 移植」這個前提上。這支就是驗那個前提：
拿 ref_kinematics.py（從 SwerveKinematics.cpp 獨立重譯、刻意不看 swerve_model.py）
去對 swerve_model.py，隨機輸入大量比對。

不一致的時候要先判斷是哪一邊偏了：
  - 對照 C++ 原始碼，錯的通常是 swerve_model.py（移植時漏抄）
  - ref_kinematics.py 只有在 C++ 真的改了的時候才該跟著改

    python test_parity.py
"""

import random
import sys

import swerve_model as M
from ref_kinematics import RefKinematics

TOL = 1e-9
fails = []


def check(cond, msg):
    print(("  [ OK ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def make_pair():
    """同一組參數餵給兩邊"""
    a = RefKinematics()
    a.set_chassis_param(M.CHASSIS_WHEELBASE_M, M.CHASSIS_TRACK_M, M.MAX_WHEEL_SPEED_MPS)
    a.set_steer_range(M.STEER_ANGLE_MIN_DEG, M.STEER_ANGLE_MAX_DEG, M.STEER_ANGLE_MARGIN_DEG)
    a.set_mount_offsets(M.MOUNT_OFFSET_DEG)
    a.set_limit_keepout(M.STEER_KEEPOUT_DEG)

    b = M.SwerveKinematics()
    b.set_chassis_param(M.CHASSIS_WHEELBASE_M, M.CHASSIS_TRACK_M, M.MAX_WHEEL_SPEED_MPS)
    b.set_steer_range(M.STEER_ANGLE_MIN_DEG, M.STEER_ANGLE_MAX_DEG, M.STEER_ANGLE_MARGIN_DEG)
    b.set_mount_offsets(M.MOUNT_OFFSET_DEG)
    b.set_limit_keepout(M.STEER_KEEPOUT_DEG)
    return a, b


# ---------------------------------------------------------------- 1
print("\n[1] 設定值：行程窗口 / keep-out 夾限 / 底盤幾何")
a, b = make_pair()
worst = max(abs(a.steer_min_deg - b.steer_min_deg),
            abs(a.steer_max_deg - b.steer_max_deg),
            abs(a.keepout_deg - b.keepout_deg),
            abs(a.radius_sum_sq - b.radius_sum_sq),
            abs(a.max_wheel_speed - b.max_wheel_speed),
            max(abs(a.module_x[i] - b.module_x[i]) for i in range(M.N)),
            max(abs(a.module_y[i] - b.module_y[i]) for i in range(M.N)))
check(worst <= TOL, f"set_chassis_param / set_steer_range / set_limit_keepout，差 {worst:.1e}")

# keep-out 上限夾限：故意填過頭，兩邊要夾到同一個值
ko_worst = 0.0
for k in (0.0, 10.0, 30.0, 42.0, 60.0, 999.0):
    a2, b2 = make_pair()
    a2.set_limit_keepout(k)
    b2.set_limit_keepout(k)
    ko_worst = max(ko_worst, abs(a2.keepout_deg - b2.keepout_deg))
check(ko_worst <= TOL, f"keep-out 夾限，6 種填法（含填過頭的 60 / 999），差 {ko_worst:.1e}")

# ---------------------------------------------------------------- 2
print("\n[2] resolve_angle：隨機目標角 x 隨機目前角")
random.seed(20260814)
a, b = make_pair()
worst_deg = worst_sign = 0.0
found_mismatch = 0
N_CASE = 20000
for _ in range(N_CASE):
    desired = random.uniform(-720.0, 720.0)
    current = random.uniform(-50.0, 320.0)
    ra = a.resolve_angle(desired, current)      # (found, deg, sign)
    rb = b.resolve_angle(desired, current)      # (deg, sign) 或 None
    if ra[0] != (rb is not None):
        found_mismatch += 1
        continue
    if ra[0]:
        worst_deg = max(worst_deg, abs(ra[1] - rb[0]))
        worst_sign = max(worst_sign, abs(ra[2] - rb[1]))
check(found_mismatch == 0, f"「找不找得到解」一致（{N_CASE} 組，不一致 {found_mismatch} 組）")
check(worst_deg <= TOL, f"解出來的機械角，差 {worst_deg:.1e}")
check(worst_sign <= TOL, f"解出來的輪速正負，差 {worst_sign:.1e}")

# ---------------------------------------------------------------- 3
print("\n[3] inverse：隨機 cmd_vel x 隨機目前角（含零速走的那條捷徑）")
a, b = make_pair()
worst_ang = worst_spd = 0.0
for _ in range(N_CASE):
    vx = random.uniform(-1.2, 1.2)
    vy = random.uniform(-1.2, 1.2)
    wz = random.uniform(-2.5, 2.5)
    if random.random() < 0.15:
        vx = vy = wz = 0.0      # 零速沿用上次角度，那條路徑也要對
    cur = [random.uniform(3.0, 267.0) for _ in range(M.N)]
    sa = a.inverse(vx, vy, wz, cur)
    sb = b.inverse(vx, vy, wz, cur)
    for i in range(M.N):
        worst_ang = max(worst_ang, abs(sa[i][0] - sb[i][0]))
        worst_spd = max(worst_spd, abs(sa[i][1] - sb[i][1]))
check(worst_ang <= TOL, f"目標機械角（{N_CASE} 組），差 {worst_ang:.1e}")
check(worst_spd <= TOL, f"輪速（含反向解的正負），差 {worst_spd:.1e}")
last_worst = max(abs(a.last_angle_deg[i] - b.last_angle_deg[i]) for i in range(M.N))
check(last_worst <= TOL, f"內部狀態 last_angle 也要一致，差 {last_worst:.1e}")

# ---------------------------------------------------------------- 4
print("\n[4] forward + update_odom：隨機角度與輪速，連續積分")
a, b = make_pair()
worst_pose = 0.0
for _ in range(N_CASE):
    ang = [random.uniform(0.0, 270.0) for _ in range(M.N)]
    spd = [random.uniform(-1.1, 1.1) for _ in range(M.N)]
    dt = random.choice([0.01, 0.005, 0.02, 0.0, -0.1, 0.6])   # 含不合理的 dt
    a.update_odom(ang, spd, dt)
    b.update_odom(ang, spd, dt)
    oa, ob = a.odom, b.odom
    worst_pose = max(worst_pose,
                     abs(oa['x'] - ob['x']), abs(oa['y'] - ob['y']),
                     abs(oa['yaw'] - ob['yaw']),
                     abs(oa['vx'] - ob['vx']), abs(oa['vy'] - ob['vy']),
                     abs(oa['wz'] - ob['wz']))
check(worst_pose <= TOL,
      f"位姿與速度（{N_CASE} 組，含 dt 不合理時要跳過積分），差 {worst_pose:.1e}")

# ---------------------------------------------------------------- 總結
print("\n" + "=" * 64)
if fails:
    print(f"失敗 {len(fails)} 項：")
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("全部通過")
