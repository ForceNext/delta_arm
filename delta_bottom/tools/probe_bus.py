#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
总线探测工具（只读诊断，不会让电机动作）。

用途：定位「电机一直不在线 / 时而在线时而不在线」这类通信故障，回答三个问题：
  1. 串口波特率到底设成了多少？（回读 termios2 的真实 c_ispeed/c_ospeed）
  2. 在某个波特率下，哪些从站地址真的会应答？
  3. 失败到底是「没应答（超时）」「地址/功能码不对」还是「CRC 错」？

只发送 Modbus 读命令（04H 读输入寄存器）：
  - 0x2A 读实时位置（2 个寄存器，与程序里的在线探测完全一致）
  - 0x20 读软硬件版本（1 个寄存器，辅助确认）
绝不会发送写命令，因此不会改变驱动器状态或让电机运动。

用法：
  python3 probe_bus.py                      # 默认 115200，地址 1~3
  python3 probe_bus.py --baud 256000        # 指定波特率
  python3 probe_bus.py --baud 115200 --addr 1-8
  python3 probe_bus.py --baud 115200,256000,921600 --addr 1-3   # 一次跑多个波特率
  python3 probe_bus.py --port /dev/ttyACM0 --timeout 100
"""

import argparse
import ctypes
import fcntl
import os
import select
import struct
import sys
import time

# ---------------------------------------------------------------------------
# 任意波特率：termios2 + BOTHER（与 drives/src/serial_port.cpp 中同一套做法）
# ---------------------------------------------------------------------------
BOTHER = 0x1000
CBAUD = 0x100F
NCCS = 19


class termios2(ctypes.Structure):
    _fields_ = [
        ("c_iflag", ctypes.c_uint),
        ("c_oflag", ctypes.c_uint),
        ("c_cflag", ctypes.c_uint),
        ("c_lflag", ctypes.c_uint),
        ("c_line", ctypes.c_ubyte),
        ("c_cc", ctypes.c_ubyte * NCCS),
        ("c_ispeed", ctypes.c_uint),
        ("c_ospeed", ctypes.c_uint),
    ]


def _IOC(direction, typ, nr, size):
    return (direction << 30) | (size << 16) | (ord(typ) << 8) | nr


TCGETS2 = _IOC(2, 'T', 0x2A, ctypes.sizeof(termios2))
TCSETS2 = _IOC(1, 'T', 0x2B, ctypes.sizeof(termios2))


def set_baud(fd, baud):
    """用 termios2 + BOTHER 设置任意波特率，返回实际生效的 (ispeed, ospeed)。"""
    buf = bytearray(ctypes.sizeof(termios2))
    fcntl.ioctl(fd, TCGETS2, buf, True)
    tio = termios2.from_buffer(buf)
    tio.c_cflag &= ~CBAUD
    tio.c_cflag |= BOTHER
    tio.c_ispeed = baud
    tio.c_ospeed = baud
    fcntl.ioctl(fd, TCSETS2, buf, True)
    # 回读确认驱动是否真的接受了这个速率
    fcntl.ioctl(fd, TCGETS2, buf, True)
    tio = termios2.from_buffer(buf)
    return tio.c_ispeed, tio.c_ospeed


# ---------------------------------------------------------------------------
# Modbus RTU
# ---------------------------------------------------------------------------
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_read(addr: int, start: int, count: int) -> bytes:
    body = bytes([addr, 0x04, start >> 8, start & 0xFF, count >> 8, count & 0xFF])
    c = crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def recv_frame(fd, timeout_s):
    """收一帧：先等首字节（timeout_s），再按帧内间隔收完，返回 (raw, 首字节耗时 ms)。"""
    t0 = time.monotonic()
    buf = b""
    first_ms = None
    deadline = t0 + timeout_s
    while True:
        remain = deadline - time.monotonic()
        if remain <= 0:
            break
        r, _, _ = select.select([fd], [], [], remain)
        if not r:
            break
        chunk = os.read(fd, 256)
        if not chunk:
            break
        if not buf:
            first_ms = (time.monotonic() - t0) * 1000.0
            deadline = time.monotonic() + 0.02      # 首字节后，帧内间隔 20ms
        buf += chunk
        if len(buf) >= 260:
            break
    return buf, first_ms


def classify(raw: bytes, addr: int, start: int, count: int):
    if not raw:
        return "TIMEOUT(无任何字节)"
    if len(raw) < 5:
        return f"SHORT(只收到 {len(raw)} 字节)"
    if raw[0] != addr:
        return f"ADDR_MISMATCH(期望 {addr}，实际 {raw[0]})"
    if raw[1] == 0x84:
        return f"EXCEPTION(异常码 0x{raw[2]:02X})"
    if raw[1] != 0x04:
        return f"FUNC_MISMATCH(期望 04，实际 {raw[1]:02X})"
    exp_len = 5 + count * 2
    if len(raw) < exp_len:
        return f"SHORT(期望 {exp_len} 字节，实际 {len(raw)})"
    body, rx = raw[:exp_len - 2], raw[exp_len - 2] | (raw[exp_len - 1] << 8)
    if crc16(body) != rx:
        return f"BAD_CRC(计算 0x{crc16(body):04X}，收到 0x{rx:04X})"
    n = raw[2]
    data = raw[3:3 + n]
    return f"OK  数据={data.hex(' ').upper()}"


def probe(fd, addr, start, count, timeout_s, strict=False, byte_timeout_s=0.005):
    req = build_read(addr, start, count)
    os.write(fd, req)
    # tcdrain：等数据真正发出（与 C++ SerialPort::write 一致）
    try:
        import termios as _t
        _t.tcdrain(fd)
    except Exception:
        pass
    if strict:
        raw, err = recv_frame_strict(fd, timeout_s, byte_timeout_s, 5 + count * 2)
        if err:
            return raw, None, f"FAIL({err})"
        return raw, None, classify(raw, addr, start, count)
    raw, first_ms = recv_frame(fd, timeout_s)
    return raw, first_ms, classify(raw, addr, start, count)


def listen(fd, seconds, label):
    """只监听不发送：用于判断总线上是否有持续信号（噪声 or 另一个主站）。"""
    import termios
    termios.tcflush(fd, termios.TCIFLUSH)
    t0 = time.monotonic()
    total = 0
    chunks = []
    while time.monotonic() - t0 < seconds:
        r, _, _ = select.select([fd], [], [], 0.2)
        if not r:
            continue
        data = os.read(fd, 4096)
        if not data:
            continue
        total += len(data)
        chunks.append((time.monotonic() - t0, data))
    rate = total / seconds
    print(f"  [只听不发 {label}] {seconds:.1f}s 内收到 {total} 字节 "
          f"({rate:.0f} B/s ≈ {rate*10:.0f} bit/s)")
    for t, d in chunks[:6]:
        print(f"      t={t*1000:7.1f}ms  {len(d):4d}B  {d[:24].hex(' ').upper()}")
    if len(chunks) > 6:
        print(f"      ... 共 {len(chunks)} 个数据块")
    return total


def recv_frame_strict(fd, timeout_s, byte_timeout_s, expected_len=9):
    """完全复刻 ModbusMaster::transact_ 的收帧时序：
       ① 前 3 字节（addr+func+第 3 字节）首字节等 timeout_ms，其余各等 byte_timeout_ms；
       ② 数据区（expected_len-3 字节）的第一个字节也只等 byte_timeout_ms。
       expected_len = 完整应答帧长度 = 5 + 寄存器数*2（异常应答为 5）。
       返回 (raw, 失败点)。"""
    import termios
    raw = b""
    # 第 1 段：3 字节（addr + func + 第3字节）
    for i in range(3):
        t = timeout_s if i == 0 else byte_timeout_s
        r, _, _ = select.select([fd], [], [], t)
        if not r:
            return raw, ("等待第 %d 字节超时(%.0fms)" % (i + 1, t * 1000))
        chunk = os.read(fd, 1)
        if not chunk:
            return raw, "读到 EOF"
        raw += chunk
    # 第 2 段：剩余字节，第一个字节也只等 byte_timeout_ms
    for i in range(max(0, expected_len - 3)):
        r, _, _ = select.select([fd], [], [], byte_timeout_s)
        if not r:
            return raw, ("等待数据区第 %d 字节超时(%.0fms)" % (i + 1, byte_timeout_s * 1000))
        chunk = os.read(fd, 1)
        if not chunk:
            return raw, "读到 EOF"
        raw += chunk
    return raw, None


def parse_addrs(s):
    out = []
    for part in s.split(','):
        if '-' in part:
            a, b = part.split('-')
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def main():
    ap = argparse.ArgumentParser(description="Modbus 从站总线探测（只读）")
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', default='115200', help='逗号分隔，可给多个波特率')
    ap.add_argument('--addr', default='1-3', help='地址，如 1-8 或 1,2,3')
    ap.add_argument('--timeout', type=float, default=0.08, help='等首字节超时（秒）')
    ap.add_argument('--trials', type=int, default=1,
                    help='每个地址重复探测次数（统计成功率）')
    ap.add_argument('--burst', type=int, default=0,
                    help='模拟程序行为：每轮按顺序把 --addr 全部连续探一遍（背靠背），'
                         '重复 N 轮，分别统计各地址成功率')
    ap.add_argument('--byte-timeout', type=float, default=0.005,
                    help='--strict 模式下帧内字节间隔超时（秒），默认 5ms = 程序里的值')
    ap.add_argument('--strict', action='store_true',
                    help='严格复刻程序的收帧时序（首字节 20ms，帧内及数据区首字节 5ms）')
    ap.add_argument('--listen', type=float, default=0.0,
                    help='只监听不发送的秒数（判断总线噪声/另一主站）')
    ap.add_argument('--raw', default=None,
                    help='发送任意十六进制字节并抓取回复（诊断回环/echo 用），'
                         '如 --raw 0104002A0002C92B 或 --raw 0104（半截帧）')
    ap.add_argument('--rts', choices=['in', 'out', 'keep'], default='keep',
                    help='设置 RTS 电平（RS485 方向控制常见由 RTS 决定）')
    ap.add_argument('--dtr', choices=['in', 'out', 'keep'], default='keep',
                    help='设置 DTR 电平')
    args = ap.parse_args()

    TIOCMBIS, TIOCMBIC, TIOCM_RTS, TIOCM_DTR = 0x5418, 0x5419, 0x004, 0x002

    bauds = [int(b) for b in args.baud.split(',')]
    addrs = parse_addrs(args.addr)

    if not os.path.exists(args.port):
        print(f"[错误] 串口不存在: {args.port}")
        return 1

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        # 原始模式 + 8N1（与 SerialPort::setInterfaceAttrs 等价）
        import termios
        attrs = termios.tcgetattr(fd)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
        cflag = (cflag & ~termios.CSIZE) | termios.CS8
        cflag &= ~(termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        cflag |= termios.CREAD | termios.CLOCAL
        lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE |
                   termios.ECHONL | termios.ISIG)
        iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY |
                   termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                   termios.ISTRIP | termios.INLCR | termios.IGNCR | termios.ICRNL)
        oflag &= ~(termios.OPOST | termios.ONLCR)
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, termios.B9600,
                           termios.B9600, cc])

        for baud in bauds:
            isp, osp = set_baud(fd, baud)
            # 手工控制 RTS/DTR（诊断 RS485 方向控制）
            mask = 0
            if args.rts == 'in':
                mask |= TIOCM_RTS
            if args.dtr == 'in':
                mask |= TIOCM_DTR
            if args.rts == 'out':
                fcntl.ioctl(fd, TIOCMBIC, struct.pack('I', TIOCM_RTS))
            if args.dtr == 'out':
                fcntl.ioctl(fd, TIOCMBIC, struct.pack('I', TIOCM_DTR))
            if mask:
                fcntl.ioctl(fd, TIOCMBIS, struct.pack('I', mask))
            ok_rate = (isp == baud and osp == baud)
            print(f"\n{'='*72}")
            print(f"波特率请求 {baud}  →  回读 ispeed={isp} ospeed={osp} "
                  f"{'✓ 生效' if ok_rate else '✗ 未生效(驱动改写/拒绝)'}")
            print(f"{'='*72}")
            if args.listen > 0:
                listen(fd, args.listen, f"@{baud}")
            if args.raw is not None:
                import termios
                payload = bytes.fromhex(args.raw.replace(' ', ''))
                for trial in range(3):
                    termios.tcflush(fd, termios.TCIFLUSH)
                    os.write(fd, payload)
                    try:
                        termios.tcdrain(fd)
                    except Exception:
                        pass
                    raw, first_ms = recv_frame(fd, args.timeout)
                    t = f"{first_ms:6.2f}ms" if first_ms is not None else "   --   "
                    print(f"  发送 {payload.hex(' ').upper():<28} 首字节 {t} "
                          f"收 {len(raw):3d}B  {raw.hex(' ').upper()}")
                continue
            if args.burst:
                # 复刻 ArmDriveTask::run() 的探测方式：同一拍内按 1,2,3 顺序背靠背探测
                okcnt = {a: 0 for a in addrs}
                samples = {a: [] for a in addrs}
                for _ in range(args.burst):
                    for a in addrs:
                        termios.tcflush(fd, termios.TCIFLUSH)
                        raw, _, verdict = probe(fd, a, 0x2A, 2, args.timeout, args.strict, args.byte_timeout)
                        if verdict.startswith("OK"):
                            okcnt[a] += 1
                        elif len(samples[a]) < 2:
                            samples[a].append(raw)
                print(f"  [背靠背突发探测，{args.burst} 轮]")
                for a in addrs:
                    print(f"    地址 {a:3d}: 成功 {okcnt[a]:3d}/{args.burst} "
                          f"({100.0*okcnt[a]/args.burst:5.1f}%)")
                for a in addrs:
                    for raw in samples[a][:1]:
                        print(f"      地址 {a} 失败样本 {len(raw):3d}B "
                              f"{raw.hex(' ').upper()[:80]}")
            for addr in addrs:
                if args.burst:
                    break
                good = []
                seen = []
                for _ in range(args.trials):
                    termios.tcflush(fd, termios.TCIFLUSH)
                    # 与程序在线探测完全相同：0x2A 读实时位置 2 个寄存器
                    raw, first_ms, verdict = probe(fd, addr, 0x2A, 2, args.timeout, args.strict, args.byte_timeout)
                    t = f"{first_ms:6.2f}ms" if first_ms is not None else "   --   "
                    seen.append(raw)
                    if verdict.startswith("OK"):
                        good.append((t, raw, verdict))
                    if args.trials == 1:
                        print(f"  地址 {addr:3d}  0x2A位置  首字节 {t}  "
                              f"收 {len(raw):2d}B  {raw.hex(' ').upper():<28} → {verdict}")
                if args.trials > 1:
                    rate = 100.0 * len(good) / args.trials
                    print(f"  地址 {addr:3d}  0x2A位置  {args.trials} 次中成功 "
                          f"{len(good)} 次 ({rate:.1f}%)")
                    for t, raw, verdict in good[:3]:
                        print(f"        ✓ 成功样本 首字节 {t}  {raw.hex(' ').upper()} → {verdict}")
                    bad = [r for r in seen if r not in [g[1] for g in good]]
                    for raw in bad[:3]:
                        print(f"        ✗ 失败样本 {len(raw):2d}B "
                              f"{raw.hex(' ').upper()}")
                # 辅助：0x20 读版本 1 个寄存器
                if args.trials == 1:
                    termios.tcflush(fd, termios.TCIFLUSH)
                    raw, first_ms, verdict = probe(fd, addr, 0x20, 1, args.timeout, args.strict, args.byte_timeout)
                    t = f"{first_ms:6.2f}ms" if first_ms is not None else "   --   "
                    print(f"  地址 {addr:3d}  0x20版本  首字节 {t}  "
                          f"收 {len(raw):2d}B  {raw.hex(' ').upper():<28} → {verdict}")
    finally:
        os.close(fd)
    return 0


if __name__ == '__main__':
    sys.exit(main())
