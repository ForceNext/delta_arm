#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Xbox 手柄测试（pygame / SDL2）

图形模式（默认）：每个摇杆/扳机一根进度条，按键方块高亮，十字键九宫格指示。
终端模式（--text）：不需要显示器，适合 SSH；值变化时才打印一行。

用法：
  python3 xbox_test.py                     # 图形窗口（ESC / q 退出）
  python3 xbox_test.py --text              # 纯终端输出
  python3 xbox_test.py --text --duration 5 # 跑 5 秒自动退出（自检用）
  python3 xbox_test.py --rumble            # 启动时发一次震动测试
  python3 xbox_test.py --index 1           # 多个手柄时选第 2 个

Linux 权限提示：
  需要能读 /dev/input/eventXX。图形界面登录时 systemd-logind 通常已经给当前用户加了 ACL；
  若提示“没有检测到手柄”，检查：
      lsusb | grep -i xbox
      ls -l /dev/input/event*        # 你的用户是否可读
      sudo usermod -aG input $USER   # 加组后需重新登录
      # 或者给这台设备加一条 udev 规则（推荐）：
      # /etc/udev/rules.d/99-xbox.rules
      #   KERNEL=="event*", ATTRS{name}=="Microsoft X-Box 360 pad*", MODE="0660", GROUP="input"
"""

import argparse
import math
import os
import sys
import time

try:
    import pygame
except ImportError:
    print("没有 pygame。安装方式（任选其一）：\n"
          "  python3 -m pip install --user --break-system-packages pygame\n"
          "  sudo apt install python3-pygame", file=sys.stderr)
    sys.exit(1)

# 轴名不是硬编码索引：静止值接近 -1 的是扳机，其余前 4 个依次是 LX/LY/RX/RY
# （不同驱动/平台索引不同，实际以 build_axis_labels() 的推断结果为准）
STICK_NAMES = ["左摇杆 X", "左摇杆 Y", "右摇杆 X", "右摇杆 Y"]
TRIG_NAMES = ["左扳机 LT", "右扳机 RT"]
BUTTON_NAMES = ["A", "B", "X", "Y", "LB", "RB", "BACK", "START", "GUIDE", "LS", "RS"]

BG = (22, 24, 28)
FG = (232, 234, 238)
DIM = (110, 116, 128)
ACC = (86, 200, 138)
WARN = (240, 170, 70)


def build_axis_labels(j):
    """按"静止值"推断每个轴的用途：<-0.9 视为扳机（松开时为 -1），其余按顺序是摇杆"""
    n = j.get_numaxes()
    rest = [j.get_axis(i) for i in range(n)]
    trig = [i for i in range(n) if rest[i] < -0.9]
    sticks = [i for i in range(n) if i not in trig]
    labels = [f"轴{i}" for i in range(n)]
    for k, i in enumerate(sticks[:4]):
        labels[i] = STICK_NAMES[k]
    for k, i in enumerate(trig[:2]):
        labels[i] = TRIG_NAMES[k]
    return labels, rest


def load_fonts(size_big=22, size_small=18):
    """找一个带中文字形的字体；找不到就退回默认字体（会显示方块，但不崩）"""
    for name in ("notosanscjksc", "notosanscjk", "wqyzenhei", "droidsansfallback",
                 "arplumingcn", "notoserifcjksc"):
        path = pygame.font.match_font(name)
        if path:
            try:
                return pygame.font.Font(path, size_big), pygame.font.Font(path, size_small)
            except Exception:
                pass
    return pygame.font.Font(None, size_big), pygame.font.Font(None, size_small)


def axis_name(i):
    return AXIS_LABELS[i] if i < len(AXIS_LABELS) else f"轴 {i}"


def button_name(i):
    return BUTTON_NAMES[i] if i < len(BUTTON_NAMES) else f"btn {i}"


AXIS_LABELS = []
REST_VALUES = []


def pick_joystick(index):
    """返回 (joystick, 说明)；没有可用手柄返回 (None, 诊断信息)"""
    n = pygame.joystick.get_count()
    if n == 0:
        info = [
            "没有检测到手柄（pygame 看到 0 个）。检查：",
            "  1) lsusb | grep -i xbox            # 设备是否插着",
            "  2) ls -l /dev/input/event*         # 你的用户是否可读（需要 ACL 或 input 组）",
            "  3) sudo usermod -aG input $USER    # 加组后重新登录",
            "  4) udev 规则（推荐）: /etc/udev/rules.d/99-xbox.rules",
            '     KERNEL=="event*", ATTRS{name}=="Microsoft X-Box 360 pad*", MODE="0660", GROUP="input"',
        ]
        return None, "\n".join(info)
    if index >= n:
        index = 0
    j = pygame.joystick.Joystick(index)
    j.init()
    info = (f"手柄 {index + 1}/{n}: {j.get_name()}\n"
            f"  GUID={j.get_guid()}  轴={j.get_numaxes()}  按键={j.get_numbuttons()}  "
            f"十字键={j.get_numhats()}  轨迹球={j.get_numballs()}")
    global AXIS_LABELS, REST_VALUES
    AXIS_LABELS, REST_VALUES = build_axis_labels(j)
    layout = "  ".join(f"{i}={AXIS_LABELS[i]}({REST_VALUES[i]:+.2f})"
                       for i in range(len(AXIS_LABELS)))
    info += "\n  轴布局（按静止值推断）: " + layout
    return j, info


def read_state(j):
    axes = [j.get_axis(i) for i in range(j.get_numaxes())]
    btns = [j.get_button(i) for i in range(j.get_numbuttons())]
    hats = [j.get_hat(i) for i in range(j.get_numhats())]
    return axes, btns, hats


def hat_text(hats):
    if not hats:
        return "—"
    out = []
    for hx, hy in hats:
        dx = {-1: "左", 1: "右", 0: ""}[hx]
        dy = {1: "上", -1: "下", 0: ""}[hy]
        out.append((dy + dx) if (dx or dy) else "·")
    return " ".join(out)


def run_text(j, args):
    print("按 Ctrl-C 退出" if args.duration <= 0 else f"运行 {args.duration:.1f} 秒…")
    t0 = time.monotonic()
    last_print = 0.0
    prev = None
    clock = pygame.time.Clock()
    try:
        while True:
            pygame.event.pump()
            for e in pygame.event.get():
                if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q)):
                    return
            now = time.monotonic()
            if args.duration > 0 and now - t0 >= args.duration:
                break
            axes, btns, hats = read_state(j)
            pressed = [button_name(i) for i, v in enumerate(btns) if v]
            # 变化才打印（轴 0.02、按键、十字键）
            changed = prev is None
            if not changed:
                changed = any(abs(a - b) > 0.02 for a, b in zip(axes, prev[0])) or \
                          pressed != prev[1] or hats != prev[2]
            if changed and (now - last_print) > 0.05:
                last_print = now
                prev = (axes, pressed, hats)
                ax = " ".join(f"{a:+.2f}" for a in axes)
                print(f"\r轴 [{ax}]  按下[{','.join(pressed) if pressed else '-'}]  "
                      f"十字[{hat_text(hats)}]   ", end="", flush=True)
            clock.tick(60)
    except KeyboardInterrupt:
        pass
    print()


def render(screen, j, args, axes, btns, hats, font, small):
    W, H = screen.get_size()
    screen.fill(BG)
    screen.blit(font.render(f"{j.get_name()}   {j.get_guid()}", True, FG), (16, 12))
    screen.blit(small.render("ESC / q 退出   轴=摇杆与扳机   方块=按键   九宫格=十字键", True, DIM),
                (16, 38))

    # —— 轴进度条 ——
    #   扳机（静止 -1）从左端往右填；摇杆（静止 0）从中心往两边填
    y = 74
    for i, v in enumerate(axes):
        is_trig = (i < len(REST_VALUES)) and (REST_VALUES[i] < -0.9)
        screen.blit(small.render(f"{axis_name(i):<10} {v:+.3f}", True, FG), (16, y - 2))
        bx, bw, bh = 210, W - 260, 16
        pygame.draw.rect(screen, (44, 48, 56), (bx, y, bw, bh), border_radius=4)
        vv = max(-1.0, min(1.0, v))
        if is_trig:
            frac = (vv + 1.0) / 2.0
            pygame.draw.rect(screen, ACC, (bx, y, int(bw * frac), bh), border_radius=4)
        else:
            cx = bx + bw // 2
            pygame.draw.line(screen, DIM, (cx, y), (cx, y + bh), 1)
            if vv >= 0:
                pygame.draw.rect(screen, ACC, (cx, y, int(bw / 2 * vv), bh), border_radius=4)
            else:
                pygame.draw.rect(screen, ACC, (cx + int(bw / 2 * vv), y, int(-bw / 2 * vv), bh),
                                 border_radius=4)
            if abs(vv) < args.dead:
                pygame.draw.rect(screen, (60, 66, 78), (cx - 2, y, 4, bh))
        y += 34

    # —— 按键方块 ——
    y += 12
    screen.blit(font.render("按键", True, FG), (16, y))
    y += 28
    per_row = 8
    bw2, bh2 = 96, 34
    for i, v in enumerate(btns):
        col, row = i % per_row, i // per_row
        x = 16 + col * (bw2 + 8)
        yy = y + row * (bh2 + 8)
        pygame.draw.rect(screen, ACC if v else (44, 48, 56), (x, yy, bw2, bh2), border_radius=6)
        screen.blit(small.render(f"{i}:{button_name(i)}", True, BG if v else FG), (x + 8, yy + 9))

    # —— 十字键九宫格 ——
    if hats:
        hx, hy = hats[0]
        gx, gy = W - 170, H - 170
        screen.blit(font.render("十字键", True, FG), (gx, gy - 30))
        for r in range(3):
            for c in range(3):
                on = (hx or hy) and (c - 1 == hx) and (1 - r == hy)
                pygame.draw.rect(screen, ACC if on else (44, 48, 56),
                                 (gx + c * 40, gy + r * 40, 34, 34), border_radius=6)


def run_window(j, args):
    W, H = 920, 620
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption(f"手柄测试 - {j.get_name()}")
    font, small = load_fonts()
    clock = pygame.time.Clock()

    t0 = time.monotonic()
    running = True
    while running:
        if args.duration > 0 and time.monotonic() - t0 >= args.duration:
            break
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False
            elif e.type == pygame.JOYDEVICEREMOVED:
                print("手柄被拔出")
                running = False

        axes, btns, hats = read_state(j)
        render(screen, j, args, axes, btns, hats, font, small)
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()


def main():
    ap = argparse.ArgumentParser(description="Xbox 手柄测试（pygame）")
    ap.add_argument("--text", action="store_true", help="纯终端输出（不需要显示器）")
    ap.add_argument("--index", type=int, default=0, help="选第几个手柄（从 0 开始）")
    ap.add_argument("--dead", type=float, default=0.08, help="死区（图形模式下画灰块），默认 0.08")
    ap.add_argument("--duration", type=float, default=0.0, help="运行多少秒后自动退出（0=一直跑）")
    ap.add_argument("--rumble", action="store_true", help="启动时发一次震动测试")
    ap.add_argument("--shot", metavar="PNG", help="渲染一帧存成图片后退出（不需要显示器）")
    args = ap.parse_args()

    # 截图模式：用 dummy 视频驱动渲染一帧
    if args.shot:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
    # 终端模式下不需要窗口；没显示器时也自动走终端
    if args.text or not os.environ.get("DISPLAY"):
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()
    pygame.joystick.init()

    j, info = pick_joystick(args.index)
    print(info)
    if j is None:
        pygame.quit()
        return 1

    if args.rumble:
        if hasattr(j, "rumble"):
            ok = j.rumble(1.0, 1.0, 400)
            print("震动测试:", "已发送" if ok else "设备不支持/失败")
        else:
            print("震动测试: 这个 pygame 版本没有 rumble() 接口")

    if args.shot:
        pygame.event.pump()
        for _ in range(3):                      # 先泵几帧让状态稳定
            pygame.event.pump()
        axes, btns, hats = read_state(j)
        screen = pygame.display.set_mode((920, 620))
        screen.fill(BG)
        f_big, f_small = load_fonts()
        render(screen, j, args, axes, btns, hats, f_big, f_small)
        pygame.image.save(screen, args.shot)
        print("已保存截图:", args.shot)
        return 0

    if args.text or not os.environ.get("DISPLAY"):
        run_text(j, args)
    else:
        run_window(j, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
