"""從韌體 C++ 獨立重新翻譯的參考實作，只給 test_parity.py 用。

刻意**不看** swerve_model.py 的 Python 版本，只照著
Esp32_wheel/lib/SwerveKinematics/SwerveKinematics.cpp 逐行抄下來。

用途：跟 swerve_model.py 的移植做數值比對。兩邊不一致就代表其中一方偏了 ——
sim 是這個專案唯一的測試，它跟韌體不一致的話，所有測試結論都不算數。

所以這支檔案有一條規矩：**改它的唯一理由是 C++ 改了**。不要為了讓
test_parity.py 過而把這裡改成跟 swerve_model.py 一樣，那就失去意義了。
"""

import math

SWERVE_N = 4
PI = math.pi
RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0


def constrain(x, a, b):
    """Arduino 的 constrain()"""
    return a if x < a else (b if x > b else x)


class RefKinematics:
    """對應 class SwerveKinematics"""

    def __init__(self):
        self.module_x = [0.0] * SWERVE_N
        self.module_y = [0.0] * SWERVE_N
        self.radius_sum_sq = 1.0
        self.max_wheel_speed = 2.0
        self.steer_min_deg = 0.0
        self.steer_max_deg = 270.0
        self.mount_offset_deg = [0.0] * SWERVE_N
        self.keepout_deg = 0.0
        self.flip_hysteresis_deg = 0.0
        self.last_flip = [0] * SWERVE_N
        self.last_angle_deg = [0.0] * SWERVE_N
        self.seeded = False
        self.odom = dict(x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0)

    def set_chassis_param(self, wheelbase_m, track_m, max_wheel_speed_mps):
        hx = wheelbase_m * 0.5
        hy = track_m * 0.5
        self.module_x[0] = +hx; self.module_y[0] = +hy
        self.module_x[1] = +hx; self.module_y[1] = -hy
        self.module_x[2] = -hx; self.module_y[2] = +hy
        self.module_x[3] = -hx; self.module_y[3] = -hy
        s = 0.0
        for i in range(SWERVE_N):
            s += self.module_x[i] ** 2 + self.module_y[i] ** 2
        self.radius_sum_sq = s if s >= 1e-6 else 1e-6
        self.max_wheel_speed = max_wheel_speed_mps if max_wheel_speed_mps > 0.01 else 0.01

    def set_steer_range(self, min_deg, max_deg, margin_deg):
        self.steer_min_deg = min_deg + margin_deg
        self.steer_max_deg = max_deg - margin_deg
        if self.steer_max_deg < self.steer_min_deg:
            self.steer_max_deg = self.steer_min_deg

    def set_mount_offsets(self, offs):
        self.mount_offset_deg = list(offs)

    def set_limit_keepout(self, keepout_deg):
        self.keepout_deg = keepout_deg if keepout_deg > 0.0 else 0.0
        max_keepout = (self.steer_max_deg - self.steer_min_deg - 180.0) * 0.5
        if self.keepout_deg > max_keepout:
            self.keepout_deg = max_keepout if max_keepout > 0.0 else 0.0

    def limit_clearance_deg(self, local_deg):
        to_min = local_deg - self.steer_min_deg
        to_max = self.steer_max_deg - local_deg
        return to_min if to_min < to_max else to_max

    @staticmethod
    def normalize_angle(rad):
        while rad > PI:
            rad -= 2.0 * PI
        while rad <= -PI:
            rad += 2.0 * PI
        return rad

    def local_to_global_deg(self, i, local_deg):
        return local_deg + self.mount_offset_deg[i]

    def global_to_local_deg(self, i, global_deg):
        return global_deg - self.mount_offset_deg[i]

    def set_flip_hysteresis(self, deg):
        self.flip_hysteresis_deg = deg if deg > 0.0 else 0.0

    def resolve_angle(self, i, desired_local_deg, current_local_deg):
        """回傳 (found, deg, sign)，對應 C++ 的 bool + 兩個 out 參數"""
        for p in range(2):
            strict = (p == 0) and (self.keepout_deg > 0.0)
            best_deg = 0.0
            best_sign = 1.0
            best_cost = 1e9
            best_flip = 0
            found = False
            for flip in range(2):
                base = desired_local_deg + (180.0 if flip else 0.0)
                sign = -1.0 if flip else 1.0
                for k in range(-2, 3):
                    a = base + 360.0 * k
                    if a < self.steer_min_deg or a > self.steer_max_deg:
                        continue
                    if strict and self.limit_clearance_deg(a) < self.keepout_deg:
                        continue
                    cost = abs(a - current_local_deg)
                    # 換解遲滯：加在「另一組」的成本上，兩組都找不到解時行為不變
                    if self.flip_hysteresis_deg > 0.0 and flip != self.last_flip[i]:
                        cost += self.flip_hysteresis_deg
                    if cost < best_cost:
                        best_cost = cost
                        best_deg = a
                        best_sign = sign
                        best_flip = flip
                        found = True
            if found:
                self.last_flip[i] = best_flip
                return True, best_deg, best_sign
            if not strict:
                break
        return False, 0.0, 1.0

    def seed_angles(self, current):
        for i in range(SWERVE_N):
            self.last_angle_deg[i] = constrain(current[i], self.steer_min_deg, self.steer_max_deg)
        self.seeded = True

    def inverse(self, vx, vy, wz, current):
        if not self.seeded:
            self.seed_angles(current)

        mag = [0.0] * SWERVE_N
        dirs = [0.0] * SWERVE_N
        for i in range(SWERVE_N):
            vxi = vx - wz * self.module_y[i]
            vyi = vy + wz * self.module_x[i]
            mag[i] = math.sqrt(vxi * vxi + vyi * vyi)
            dirs[i] = math.atan2(vyi, vxi)

        peak = 0.0
        for i in range(SWERVE_N):
            if mag[i] > peak:
                peak = mag[i]
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            for i in range(SWERVE_N):
                mag[i] *= scale

        out = []
        for i in range(SWERVE_N):
            if mag[i] < 1e-4:
                out.append((self.last_angle_deg[i], 0.0))
                continue
            desired_local = self.global_to_local_deg(i, dirs[i] * RAD_TO_DEG)
            ok, angle_deg, sign = self.resolve_angle(i, desired_local, current[i])
            if not ok:
                out.append((self.last_angle_deg[i], 0.0))
                continue
            out.append((angle_deg, mag[i] * sign))
            self.last_angle_deg[i] = angle_deg
        return out

    def module_speeds(self, vx, vy, wz):
        out = []
        for i in range(SWERVE_N):
            vxi = vx - wz * self.module_y[i]
            vyi = vy + wz * self.module_x[i]
            out.append(math.sqrt(vxi * vxi + vyi * vyi))
        return out

    def project_wheel_speeds(self, vx, vy, wz, angle_local_deg):
        out = []
        peak = 0.0
        for i in range(SWERVE_N):
            vxi = vx - wz * self.module_y[i]
            vyi = vy + wz * self.module_x[i]
            a = self.local_to_global_deg(i, angle_local_deg[i]) * DEG_TO_RAD
            s = vxi * math.cos(a) + vyi * math.sin(a)
            out.append(s)
            if abs(s) > peak:
                peak = abs(s)
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            out = [x * scale for x in out]
        return out

    def forward(self, angle_deg, speed_mps):
        sum_vx = sum_vy = sum_moment = 0.0
        for i in range(SWERVE_N):
            a = self.local_to_global_deg(i, angle_deg[i]) * DEG_TO_RAD
            vxi = speed_mps[i] * math.cos(a)
            vyi = speed_mps[i] * math.sin(a)
            sum_vx += vxi
            sum_vy += vyi
            sum_moment += self.module_x[i] * vyi - self.module_y[i] * vxi
        return (sum_vx / SWERVE_N, sum_vy / SWERVE_N, sum_moment / self.radius_sum_sq)

    def update_odom(self, angle_deg, speed_mps, dt_s):
        if dt_s <= 0.0 or dt_s > 0.5:
            return
        vx, vy, wz = self.forward(angle_deg, speed_mps)
        self.odom['vx'] = vx
        self.odom['vy'] = vy
        self.odom['wz'] = wz
        yaw_mid = self.odom['yaw'] + wz * dt_s * 0.5
        self.odom['x'] += (vx * math.cos(yaw_mid) - vy * math.sin(yaw_mid)) * dt_s
        self.odom['y'] += (vx * math.sin(yaw_mid) + vy * math.cos(yaw_mid)) * dt_s
        self.odom['yaw'] = self.normalize_angle(self.odom['yaw'] + wz * dt_s)
