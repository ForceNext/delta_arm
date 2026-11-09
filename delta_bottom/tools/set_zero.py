#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
以「当前位置」为零点：把三台驱动器的位置计数器清零（0xF8，06H 写 0x00F8 = 0x0001）。

直接运行即可，不需要任何参数：

    python3 set_zero.py            # 读当前计数 → 清零三台 → 复核

常用参数：
    --dry-run       只读当前计数并打印要发的帧，不发任何写命令
    --axis 1        只清 1 号（默认三台都清；也可 --axis 1,3）
    --ask           清零前询问一次

注意：
  * 清零**不会让电机转动**，只是把「此刻的位置」定义为 0；
  * 清零后共享内存里的 theta（关节角）就以这个位形为基准；
  * 驱动器不是掉电记忆的，重新上电后计数会回到 0（需要重新标定或做 0x91/0x92 回零）；
  * 运行前请先停掉 delta_bottom —— 一根 RS485 只能有一个主站。
"""

import argparse
import os
import select
import sys
import termios
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import probe_bus as pb          # noqa: E402  建帧 / CRC / 波特率
import read_angles as ra        # noqa: E402  读位置（带 CRC 校验与重试）
import set_angle as sa          # noqa: E402  共享内存（判断底层是否在跑）

DEV = "/dev/ttyACM0"
BAUD = 256000
DEG2CNT = 51200.0 / 360.0


def zero_frame(addr):
    """06H 写 0x00F8 = 0x0001（位置清零）"""
    body = bytes([addr, 0x06, 0x00, 0xF8, 0x00, 0x01])
    c = pb.crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def bottom_alive():
    """底层在跑吗（看共享内存 status.seq 是否在变化）——在跑就别清零"""
    try:
        semid = sa.sem_open(sa.SEM_KEY)
        mm = sa.shm_map()
        data = sa.ArmSharedData.from_buffer(mm)
        sa.sem_p(semid)
        a = data.status.seq
        sa.sem_v(semid)
        time.sleep(0.15)
        sa.sem_p(semid)
        b = data.status.seq
        sa.sem_v(semid)
        return b != a
    except Exception:
        return False


def open_port(dev=None):
    fd = os.open(dev or DEV, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, isp, osp, cc = a
    cflag = (cflag & ~termios.CSIZE) | termios.CS8
    cflag &= ~(termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
    cflag |= termios.CREAD | termios.CLOCAL
    lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE |
               termios.ECHONL | termios.ISIG)
    iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY | termios.IGNBRK |
               termios.BRKINT | termios.PARMRK | termios.ISTRIP | termios.INLCR |
               termios.IGNCR | termios.ICRNL)
    oflag &= ~(termios.OPOST | termios.ONLCR)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, termios.B9600, termios.B9600, cc])
    pb.set_baud(fd, BAUD)
    return fd


def send_expect_echo(fd, frame, timeout=0.08):
    """发送并等 8 字节回显（06H 会原样回显），校验 CRC"""
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, frame)
    termios.tcdrain(fd)
    buf, t0 = b"", time.monotonic()
    while time.monotonic() - t0 < timeout:
        r, _, _ = select.select([fd], [], [], 0.002)
        if r:
            d = os.read(fd, 4096)
            if d:
                buf += d
    if len(buf) >= 8 and buf[0] == frame[0] and buf[1] == 0x06:
        if pb.crc16(buf[:6]).to_bytes(2, "little") == buf[6:8]:
            return True, buf[:8].hex(" ").upper()
    return False, buf.hex(" ").upper() if buf else "(超时/无应答)"


def main():
    ap = argparse.ArgumentParser(description="以当前位置为零点（驱动器 0xF8 计数器清零）")
    ap.add_argument("--axis", default="1,2,3", help="要清哪些轴，如 1 或 1,3（默认全部）")
    ap.add_argument("--dry-run", action="store_true", help="只读位置并打印要发的帧，不写入")
    ap.add_argument("--ask", action="store_true", help="清零前询问一次")
    ap.add_argument("--yes", action="store_true", help=argparse.SUPPRESS)  # 兼容旧用法，无作用
    ap.add_argument("--dev", default=None, help="串口设备，默认 /dev/ttyACM0")
    args = ap.parse_args()

    try:
        axes = sorted({int(x) for x in args.axis.replace(" ", "").split(",") if x})
        assert axes and all(1 <= a <= 3 for a in axes)
    except Exception:
        print("--axis 只能是 1/2/3 的组合，如 1 或 1,3", file=sys.stderr)
        return 1

    if bottom_alive():
        print("[拒绝] delta_bottom 正在运行：先停掉它（一根 RS485 只能有一个主站），\n"
              "       否则清零会与它的绝对位置下发互相打架。", file=sys.stderr)
        return 1

    try:
        counts, online, _ = ra.read_from_serial(dev=args.dev)
    except Exception as e:
        print(f"[错误] 打开串口失败（{e}）：先停掉 delta_bottom。", file=sys.stderr)
        return 1

    print("清零前：")
    for i in range(3):
        tag = "" if online[i] else "   ← 读失败，数值不可信"
        print(f"  轴{i+1}: {counts[i]:8d} 计数  {counts[i]/DEG2CNT:9.3f}° 电机角  "
              f"{counts[i]/DEG2CNT/3.0:9.3f}° 关节角{tag}")

    frames = {a: zero_frame(a) for a in axes}
    if args.dry_run:
        print("\n将要发送（06H 写 0x00F8 = 1，位置清零）：")
        for a in axes:
            print(f"  轴{a}: {frames[a].hex(' ').upper()}")
        print("\n--dry-run：未发送任何写命令")
        return 0

    if args.ask:
        if input("\n以当前位置为零点，输入 yes 继续（其它取消）: ").strip().lower() != "yes":
            print("已取消")
            return 0

    fd = open_port(args.dev)
    ok_all = True
    try:
        for a in axes:
            ok, raw = send_expect_echo(fd, frames[a])
            print(f"  轴{a}: {'✓ 已清零' if ok else '✗ 清零失败'}  应答 {raw}")
            ok_all &= ok
            time.sleep(0.05)
    finally:
        os.close(fd)

    time.sleep(0.15)
    after, online2, _ = ra.read_from_serial(dev=args.dev)
    print("\n清零后（应以当前位置为零点）：")
    for i in range(3):
        tag = "" if online2[i] else "   ← 读失败"
        bad = ""
        if i + 1 in axes and online2[i] and abs(after[i]) > 50:
            bad = "   ← 没清干净？"
        print(f"  轴{i+1}: {after[i]:8d} 计数  {after[i]/DEG2CNT:9.3f}° 电机角  "
              f"{after[i]/DEG2CNT/3.0:9.3f}° 关节角{tag}{bad}")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
