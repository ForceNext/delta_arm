#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Xbox 手柄遥操作节点

摇杆 → 末端目标速度 → 积分成 xyz → 发布 geometry_msgs/Point 到 /delta_arm/target_xyz
（由 ros2_ws 里的逆解节点 Kinematics_node 订阅后算出关节角并下发给底层）

映射（按实测方向）：
  左摇杆 X（右=+1）→ 末端 x 正方向
  左摇杆 Y（上=-1）→ 末端 y 正方向（所以取反：上推 → +y）
  右摇杆 Y（上=-1）→ 末端 z 负方向（z 向下为正，所以上推 → 平台抬升）
  松开摇杆 → 速度为 0，目标保持不动
  A → 回 home；Y → 打印当前目标；START → 退出节点；LB 精细(×0.25)；RB 加速(×2)

运行：
  ros2 run Kinematics xbox_teleop
  ros2 run Kinematics xbox_teleop --ros-args -p dry_run:=true      # 只打印不发布
  ros2 run Kinematics xbox_teleop --ros-args -p max_speed_xy:=40.0 -p max_speed_z:=25.0
  python3 xbox_teleop.py --selftest                                # 只验证映射方向，不需要手柄/ROS
"""

import math
import os
import sys

# —— 映射参数（方向约定）——
# 轴索引默认按你的 Xbox 360：0=左X 1=左Y 2=LT 3=右X 4=右Y 5=RT
AXIS_LX_DEFAULT = 0
AXIS_LY_DEFAULT = 1
AXIS_RY_DEFAULT = 4
BTN_A, BTN_Y, BTN_LB, BTN_RB, BTN_START = 0, 3, 4, 5, 7

# z 向下为正；末端坐标单位 mm
DEF_Z_MIN, DEF_Z_MAX, DEF_R_MAX = 138.0, 300.0, 150.0
DEF_HOME = (0.0, 0.0, 250.0)


def apply_deadzone_square(value, deadzone):
    """死区 + 平方曲线：|a|<dead 归零，其余按平方映射，低速段更细腻"""
    if abs(value) < deadzone:
        return 0.0
    s = (abs(value) - deadzone) / (1.0 - deadzone)
    s = max(0.0, min(1.0, s))
    return math.copysign(s * s, value)


def stick_to_velocity(lx, ly, ry, deadzone, v_xy, v_z):
    """摇杆原始值 → 末端速度 (vx, vy, vz)，单位 mm/s

    方向约定（实测）：左摇杆 右=+1 / 上=-1；右摇杆 上=-1 / 下=+1。
      x：右推 → +x
      y：上推 → +y（取反）
      z：上推 → 抬升（z 变小，取 ry 本身）
    """
    ax = apply_deadzone_square(lx, deadzone)
    ay = apply_deadzone_square(ly, deadzone)
    az = apply_deadzone_square(ry, deadzone)
    return (ax * v_xy, -ay * v_xy, az * v_z)


def clamp_to_workspace(x, y, z, z_min, z_max, r_max):
    """把目标夹到工作空间：z 范围 + xy 圆形边界（沿半径方向收）"""
    z = max(z_min, min(z_max, z))
    if r_max > 0.0:
        r = math.hypot(x, y)
        if r > r_max and r > 1e-9:
            k = r_max / r
            x *= k
            y *= k
    return x, y, z


def selftest():
    """验证映射方向（不需要手柄、不需要 ROS）"""
    ok = True

    def chk(cond, msg):
        nonlocal ok
        print(("  [PASS] " if cond else "  [FAIL] ") + msg)
        ok = ok and cond

    dz, vxy, vz = 0.08, 30.0, 20.0
    print("映射方向自检：")
    vx, vy, zz = stick_to_velocity(1.0, 0.0, 0.0, dz, vxy, vz)
    chk(vx > 0 and abs(vy) < 1e-9, f"左摇杆右推 → +x（vx={vx:.1f}）")
    vx, vy, _ = stick_to_velocity(-1.0, 0.0, 0.0, dz, vxy, vz)
    chk(vx < 0, f"左摇杆左推 → -x（vx={vx:.1f}）")
    vx, vy, _ = stick_to_velocity(0.0, -1.0, 0.0, dz, vxy, vz)
    chk(vy > 0 and abs(vx) < 1e-9, f"左摇杆上推 → +y（vy={vy:.1f}）")
    vx, vy, _ = stick_to_velocity(0.0, 1.0, 0.0, dz, vxy, vz)
    chk(vy < 0, f"左摇杆下推 → -y（vy={vy:.1f}）")
    _, _, zz = stick_to_velocity(0.0, 0.0, -1.0, dz, vxy, vz)
    chk(zz < 0, f"右摇杆上推 → z 减小=抬升（vz={zz:.1f}）")
    _, _, zz = stick_to_velocity(0.0, 0.0, 1.0, dz, vxy, vz)
    chk(zz > 0, f"右摇杆下推 → z 增大=下探（vz={zz:.1f}）")
    vx, vy, zz = stick_to_velocity(0.01, 0.01, -0.01, dz, vxy, vz)
    chk(vx == 0 and vy == 0 and zz == 0, "死区内输出为 0")

    x, y, z = clamp_to_workspace(0, 0, 500, DEF_Z_MIN, DEF_Z_MAX, DEF_R_MAX)
    chk(z == DEF_Z_MAX, f"z 超上限被夹到 {z:.1f}")
    x, y, z = clamp_to_workspace(0, 0, 0, DEF_Z_MIN, DEF_Z_MAX, DEF_R_MAX)
    chk(z == DEF_Z_MIN, f"z 超下限被夹到 {z:.1f}")
    x, y, z = clamp_to_workspace(300, 0, 250, DEF_Z_MIN, DEF_Z_MAX, DEF_R_MAX)
    chk(abs(math.hypot(x, y) - DEF_R_MAX) < 1e-6, f"半径被夹到 {math.hypot(x, y):.1f}")

    print("\n全部通过 ✓" if ok else "\n存在失败 ✗")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point

    # 手柄读取不需要窗口
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame

    class XboxTeleop(Node):
        def __init__(self):
            super().__init__("xbox_teleop")
            gp = self.declare_parameter
            self.topic = gp("xyz_topic", "/delta_arm/target_xyz").value
            self.rate_hz = gp("rate_hz", 30.0).value
            self.v_xy = gp("max_speed_xy", 30.0).value      # mm/s
            self.v_z = gp("max_speed_z", 20.0).value        # mm/s
            self.deadzone = gp("deadzone", 0.08).value
            self.z_min = gp("z_min", DEF_Z_MIN).value
            self.z_max = gp("z_max", DEF_Z_MAX).value
            self.r_max = gp("radius_max", DEF_R_MAX).value
            self.home = (gp("home_x", DEF_HOME[0]).value,
                         gp("home_y", DEF_HOME[1]).value,
                         gp("home_z", DEF_HOME[2]).value)
            self.axis_lx = gp("axis_lx", AXIS_LX_DEFAULT).value
            self.axis_ly = gp("axis_ly", AXIS_LY_DEFAULT).value
            self.axis_ry = gp("axis_ry", AXIS_RY_DEFAULT).value
            self.joy_index = gp("joy_index", 0).value
            self.dry_run = gp("dry_run", False).value
            self.fine_scale = gp("fine_scale", 0.25).value
            self.boost_scale = gp("boost_scale", 2.0).value
            if self.rate_hz < 1.0:
                self.rate_hz = 30.0

            self.pub = self.create_publisher(Point, self.topic, 10)
            self.cur = [self.home[0], self.home[1], self.home[2]]
            self.joy = None
            self.prev_btn = {}
            self.last_t = self.get_clock().now()

            pygame.init()
            pygame.joystick.init()
            self._open_joy()

            self.get_logger().info(
                f"发布 {self.topic} @ {self.rate_hz:.0f} Hz | 左摇杆→xy "
                f"({self.v_xy:.0f} mm/s) | 右摇杆上下→z ({self.v_z:.0f} mm/s)")
            self.get_logger().info(
                f"工作空间: z∈[{self.z_min:.0f}, {self.z_max:.0f}] mm, r≤{self.r_max:.0f} mm | "
                f"home=({self.home[0]:.1f}, {self.home[1]:.1f}, {self.home[2]:.1f})"
                + ("  [dry-run]" if self.dry_run else ""))
            self.create_timer(1.0 / self.rate_hz, self.on_tick)

        # ---------- 手柄 ----------
        def _open_joy(self):
            n = pygame.joystick.get_count()
            if n == 0:
                self.joy = None
                return
            idx = min(self.joy_index, n - 1)
            j = pygame.joystick.Joystick(idx)
            j.init()
            self.joy = j
            self.get_logger().info(
                f"手柄 {idx + 1}/{n}: {j.get_name()}  轴={j.get_numaxes()} 键={j.get_numbuttons()} "
                f"hat={j.get_numhats()}")

        def _btn(self, i):
            if self.joy is None or i >= self.joy.get_numbuttons():
                return False
            return bool(self.joy.get_button(i))

        def _edge(self, i):
            """按键按下的那一刻返回 True（避免长按重复触发）"""
            now = self._btn(i)
            was = self.prev_btn.get(i, False)
            self.prev_btn[i] = now
            return now and not was

        # ---------- 定时器 ----------
        def on_tick(self):
            now = self.get_clock().now()
            dt = (now - self.last_t).nanoseconds * 1e-9
            self.last_t = now
            if dt <= 0.0 or dt > 0.5:
                dt = 1.0 / self.rate_hz

            if self.joy is None:
                self._open_joy()
                if self.joy is None:
                    self.get_logger().warn(
                        "没有检测到手柄（pygame 0 个）——保持当前目标不动；"
                        "检查 /dev/input 权限或 usb 连接", throttle_duration_sec=5.0)
                    self._publish()
                    return

            pygame.event.pump()
            for e in pygame.event.get():
                if e.type == pygame.JOYDEVICEREMOVED:
                    self.get_logger().warn("手柄被拔出，保持当前目标")
                    self.joy = None

            if self.joy is None:
                self._publish()
                return

            # 按键
            if self._edge(BTN_A):
                self.cur = [self.home[0], self.home[1], self.home[2]]
                self.get_logger().info(
                    f"A: 回 home ({self.cur[0]:.1f}, {self.cur[1]:.1f}, {self.cur[2]:.1f})")
            if self._edge(BTN_Y):
                self.get_logger().info(
                    f"Y: 当前目标 ({self.cur[0]:.2f}, {self.cur[1]:.2f}, {self.cur[2]:.2f})")
            if self._edge(BTN_START):
                self.get_logger().warn("START: 退出节点（目标保持在最后一次值）")
                rclpy.shutdown()
                return

            # 速度档位
            scale = 1.0
            if self._btn(BTN_LB):
                scale *= self.fine_scale
            if self._btn(BTN_RB):
                scale *= self.boost_scale

            lx = self.joy.get_axis(self.axis_lx) if self.joy.get_numaxes() > self.axis_lx else 0.0
            ly = self.joy.get_axis(self.axis_ly) if self.joy.get_numaxes() > self.axis_ly else 0.0
            ry = self.joy.get_axis(self.axis_ry) if self.joy.get_numaxes() > self.axis_ry else 0.0

            vx, vy, vz = stick_to_velocity(lx, ly, ry, self.deadzone, self.v_xy, self.v_z)
            if scale != 1.0:
                vx, vy, vz = vx * scale, vy * scale, vz * scale

            # 积分（+ 限幅）
            self.cur[0] += vx * dt
            self.cur[1] += vy * dt
            self.cur[2] += vz * dt
            self.cur[0], self.cur[1], self.cur[2] = clamp_to_workspace(
                self.cur[0], self.cur[1], self.cur[2], self.z_min, self.z_max, self.r_max)

            if self.dry_run:
                self.get_logger().info(
                    f"[dry-run] stick=({lx:+.2f},{ly:+.2f},{ry:+.2f}) v=({vx:+.1f},{vy:+.1f},{vz:+.1f}) "
                    f"→ xyz=({self.cur[0]:.2f}, {self.cur[1]:.2f}, {self.cur[2]:.2f})",
                    throttle_duration_sec=0.5)
                return
            self._publish()

        def _publish(self):
            msg = Point()
            msg.x, msg.y, msg.z = self.cur[0], self.cur[1], self.cur[2]
            self.pub.publish(msg)

    rclpy.init()
    node = XboxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
