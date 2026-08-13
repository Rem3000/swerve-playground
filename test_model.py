"""不開視窗的邏輯測試：把 swerve_model 跑過一輪，確認偏移角轉換是對的。

    python test_model.py
"""

import math
import sys

import swerve_model as M

from swerve_model import (
    N, NAMES, SwerveRobot, SwerveKinematics, MOUNT_OFFSET_DEG,
    STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG, STEER_ANGLE_MARGIN_DEG,
    CHASSIS_WHEELBASE_M, CHASSIS_TRACK_M, MAX_WHEEL_SPEED_MPS,
)

fails = []


def check(cond, msg):
    print(("  [ OK ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def new_kin():
    k = SwerveKinematics()
    k.set_chassis_param(CHASSIS_WHEELBASE_M, CHASSIS_TRACK_M, MAX_WHEEL_SPEED_MPS)
    k.set_steer_range(STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG, STEER_ANGLE_MARGIN_DEG)
    k.set_mount_offsets(MOUNT_OFFSET_DEG)
    return k


# ---------------------------------------------------------------- 1
print("\n[1] 全方向可達性：270 度行程 + 反向解，加上偏移後仍要涵蓋 360 度")
k = new_kin()
unreach = []
for deg10 in range(-1800, 1801):
    g = deg10 / 10.0
    for i in range(N):
        if k.resolve_angle(k.global_to_local_deg(i, g), 135.0) is None:
            unreach.append((NAMES[i], g))
check(not unreach, f"3601 個方向 x 4 模組全部可達（不可達 {len(unreach)} 筆）")

# ---------------------------------------------------------------- 2
print("\n[2] 逆運動學 -> 正運動學閉環（含偏移轉換）")
k = new_kin()
cases = [
    ("前進", 1.0, 0.0, 0.0), ("後退", -1.0, 0.0, 0.0),
    ("左平移", 0.0, 1.0, 0.0), ("右平移", 0.0, -1.0, 0.0),
    ("斜走45", 0.7, 0.7, 0.0), ("逆時針自轉", 0.0, 0.0, 2.0),
    ("順時針自轉", 0.0, 0.0, -2.0), ("前進轉彎", 0.8, 0.0, 1.5),
    ("全向混合", 0.5, -0.3, 1.0), ("小速度", 0.02, 0.01, 0.05),
]
worst = 0.0
for name, vx, vy, wz in cases:
    st = k.inverse(vx, vy, wz, [135.0] * N)
    loc = [s[0] for s in st]
    spd = [s[1] for s in st]
    rvx, rvy, rwz = k.forward(loc, spd)
    e = max(abs(rvx - vx), abs(rvy - vy), abs(rwz - wz))
    worst = max(worst, e)
    in_range = all(STEER_ANGLE_MIN_DEG + STEER_ANGLE_MARGIN_DEG - 1e-6 <= a
                   <= STEER_ANGLE_MAX_DEG - STEER_ANGLE_MARGIN_DEG + 1e-6 for a in loc)
    print(f"        {name:<12} " + "  ".join(
        f"{NAMES[i]} 機械{loc[i]:6.1f}° 車體{loc[i]+MOUNT_OFFSET_DEG[i]:+7.1f}° v{spd[i]:+.2f}"
        for i in range(N)))
    check(e < 1e-9 and in_range, f"{name}：殘差 {e:.2e}，機械角在行程內 {in_range}")
check(worst < 1e-9, f"所有案例最大殘差 {worst:.2e} < 1e-9")

# ---------------------------------------------------------------- 3
print("\n[3] 已知姿態驗證（手算對照）")
# 注意：同一個行進方向有兩個等價解（θ 正轉 與 θ+180 反轉），程式挑離現在最近的，
# 平手時挑到哪一個都對。所以這裡比對「輪子實際推車的方向」，不是比對機械角本身。
k = new_kin()
st = k.inverse(1.0, 0.0, 0.0, [135.0] * N)   # 純前進


def travel_dir(i, local_deg, speed):
    """輪子實際把車往哪個方向推（車體座標，度）"""
    g = local_deg + MOUNT_OFFSET_DEG[i]
    if speed < 0:
        g += 180.0
    return round((g + 180.0) % 360.0 - 180.0, 6)


dirs = [travel_dir(i, st[i][0], st[i][1]) for i in range(N)]
check(all(abs(d) < 1e-6 for d in dirs),
      f"純前進時四顆輪子的推進方向都是車體 0 度，實得 {dirs}")
check(all(abs(abs(s[1]) - 1.0) < 1e-9 for s in st),
      "純前進時四顆輪速大小都是 1.0 m/s")
# 機械角必為 45/135/225 這幾個等價值之一
ok_mech = all(any(abs(st[i][0] - v) < 1e-6 for v in (45.0, 135.0, 225.0)) for i in range(N))
check(ok_mech, f"機械角落在等價解集合上，實得 {[round(s[0], 1) for s in st]}")

k2 = new_kin()
st = k2.inverse(0.0, 0.0, 2.0, [180.0] * N)  # 原地逆時針
got = [round(s[0], 3) for s in st]
check(got == [180.0] * N,
      f"原地自轉時四顆機械角相同（對稱安裝的必然結果），實得 {got}")

# ---------------------------------------------------------------- 4
print("\n[4] 偏移角填 0（改動前的行為）不應該被破壞")
k0 = SwerveKinematics()
k0.set_chassis_param(CHASSIS_WHEELBASE_M, CHASSIS_TRACK_M, MAX_WHEEL_SPEED_MPS)
k0.set_steer_range(STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG, STEER_ANGLE_MARGIN_DEG)
st = k0.inverse(1.0, 0.0, 0.0, [135.0] * N)
check(all(abs(s[0] - 0.0) < 1e-6 or abs(s[0] - 180.0) < 1e-6 for s in st),
      "沒設偏移時純前進 = 機械 0 度（或 180 度反向解），與舊行為一致")

# ---------------------------------------------------------------- 5
SCRIPT = [((0.8, 0.0, 0.0), 2.0), ((0.0, 0.8, 0.0), 2.0), ((0.0, 0.0, 1.5), 2.0),
          ((0.6, -0.4, 0.8), 3.0), ((-0.7, 0.0, 0.0), 2.0), ((0.0, 0.0, 0.0), 1.0)]


def run(bot, hz):
    dt = 1.0 / hz
    worst = 0.0
    for cmd, dur in SCRIPT:
        bot.set_cmd_vel(*cmd)
        for _ in range(int(round(dur / dt))):
            bot.step(dt)
            worst = max(worst, bot.odom_error())
    return worst


print("\n[5] 全系統模擬：座標轉換本身的正確性")
# 把三個迴圈頻率拉到跟物理積分同步消掉取樣延遲，並且把齒隙關掉，
# 只留下「轉換式對不對」這一件事。齒隙會讓韌體讀到的角度（馬達側）
# 和真實輪向（輸出側）差一個固定量，那是另一回事，下面 [5c] 單獨量。
_saved_backlash = M.HW_BACKLASH_DEG
M.HW_BACKLASH_DEG = 0.0
for hz in (2000, 8000):
    bot = SwerveRobot(control_hz=hz, report_hz=hz, steer_hz=hz, odom_hz=hz)
    worst = run(bot, hz)
    check(worst < 0.002,
          f"{hz} Hz 同步取樣 + 無齒隙：里程計最大誤差 {worst*1000:.4f} mm（應趨近 0）")
M.HW_BACKLASH_DEG = _saved_backlash

print("\n[5b] 全系統模擬：實際韌體頻率（50 Hz 回報）下的取樣誤差")
bot = SwerveRobot()  # 50/50/200 Hz，跟實車一樣
worst = run(bot, 200.0)
o = bot.kin.odom
dist = sum(math.hypot(bot.true_trail[i + 1][0] - bot.true_trail[i][0],
                      bot.true_trail[i + 1][1] - bot.true_trail[i][1])
           for i in range(len(bot.true_trail) - 1))
print(f"        真實 x={bot.true_pose[0]:+.4f} y={bot.true_pose[1]:+.4f} "
      f"yaw={math.degrees(bot.true_pose[2]):+.2f}°")
print(f"        里程 x={o['x']:+.4f} y={o['y']:+.4f} yaw={math.degrees(o['yaw']):+.2f}°")
print(f"        行走距離 {dist:.2f} m，最終誤差 {bot.odom_error()*100:.1f} cm "
      f"({bot.odom_error()/max(dist,1e-9)*100:.1f}%)")
check(bot.odom_error() / max(dist, 1e-9) < 0.06,
      "50 Hz 回報造成的里程計飄移 < 行走距離的 6%（純取樣延遲，實車也會有）")

print("\n[5c] 齒隙對里程計的影響（韌體讀馬達側，輪子在輸出側）")
# 編碼器裝在馬達軸上，中間隔著減速齒輪。轉向換方向的瞬間輸出會空走一段，
# 這段期間韌體以為輪子已經轉過去了，實際上還沒。這是實車一定會有的誤差。
res = []
for bl in (0.0, M.HW_BACKLASH_DEG, M.HW_BACKLASH_DEG * 2):
    M.HW_BACKLASH_DEG = bl
    b2 = SwerveRobot(control_hz=4000, report_hz=4000, steer_hz=4000, odom_hz=4000)
    run(b2, 4000)
    res.append((bl, b2.odom_error()))
    print(f"        齒隙 {bl:4.2f}° -> 里程計誤差 {b2.odom_error()*1000:6.1f} mm")
M.HW_BACKLASH_DEG = _saved_backlash
check(res[0][1] < res[1][1] < res[2][1],
      "齒隙越大里程計誤差越大（證明模擬有把這件事算進去）")
check(res[1][1] < 0.10,
      f"目前設定的 {_saved_backlash}° 齒隙造成 {res[1][1]*1000:.0f} mm 誤差，"
      f"實車要靠機構把齒隙壓下來")
check(all(STEER_ANGLE_MIN_DEG <= m.angle_deg <= STEER_ANGLE_MAX_DEG for m in bot.steer),
      "全程四顆機械角都沒有超出 0~270 度行程")

# ---------------------------------------------------------------- 6
print("\n[6] 反例：故意把 FL 的物理偏移弄錯 15 度，里程計應該要飄掉")
bot2 = SwerveRobot(control_hz=2000, report_hz=2000, steer_hz=2000, odom_hz=2000)
bot2.physical_offset_deg[0] += 15.0
bot2.set_cmd_vel(0.8, 0.0, 0.0)
for _ in range(int(4.0 * 2000)):
    bot2.step(1.0 / 2000)
check(bot2.odom_error() > 0.02,
      f"偏移填錯時里程計誤差 {bot2.odom_error()*100:.2f} cm（證明這個量真的有在影響結果）")

# ---------------------------------------------------------------- 總結
print("\n" + "=" * 64)
if fails:
    print(f"失敗 {len(fails)} 項：")
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("全部通過")
