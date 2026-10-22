#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取三台电机的角度（**只读**，不会下发任何指令）。

关于"相对位置"：
  驱动器**没有**"读相对位置"的寄存器；`0xF3` 只是"相对位置模式控制"这条**写**命令
  （每次相对当前位置偏移）。可读的绝对量是 `0x2A 读电机实时位置`（计数，51200 = 一圈）。
  因此"相对角度"一律由参考点算出来：
      相对计数 = 当前计数 − 参考计数
      相对电机角 = 相对计数 ÷ (51200/360)
      相对关节角 = 相对电机角 ÷ 减速比(joint.gear_ratio)
  参考点三种来源（优先级从高到低）：
      --ref c1,c2,c3   命令行直接给三个计数
      已保存的参考点   见 --capture / --clear（默认存 tools/.angle_ref.json）
      （都没有 → 只打印绝对值）

数据来源**自动选择**，避免两个主站抢同一根 RS485：
  * 底层 delta_bottom 在跑（共享内存 status.seq 在变化）→ 读共享内存；
  * 底层没跑 → 直连串口读 0x2A。

用法：
  python3 read_angles.py                          # 打印绝对角度（+已保存参考点的相对角度）
  python3 read_angles.py --capture                # 把当前计数记为参考点并保存
  python3 read_angles.py --ref 33717,17682,12272  # 临时指定参考点（不保存）
  python3 read_angles.py --clear                  # 清除已保存的参考点
  python3 read_angles.py --json                   # 机器可读输出
  python3 read_angles.py --ratio 3                # 覆盖减速比（默认从 hardware_config.yaml 读）
"""

import argparse
import json
import os
import re
import select
import struct
import sys
import termios
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import probe_bus as pb                      # noqa: E402  （复用建帧/波特率设置）
import set_angle as sa                      # noqa: E402  （复用共享内存/信号量封装）

DEG2CNT = 51200.0 / 360.0                   # 51200 计数 = 一圈
REF_FILE = os.path.join(HERE, ".angle_ref.json")
DEV = "/dev/ttyACM0"
BAUD = 256000


# ---------------------------------------------------------------------------
# 减速比：从 hardware_config.yaml 的 joint.gear_ratio 读（读不到则 3.0 并告警）
# ---------------------------------------------------------------------------
def load_ratio():
    path = os.path.join(HERE, "..", "configs", "hardware_config.yaml")
    try:
        txt = open(path, encoding="utf-8").read()
        m = re.search(r"gear_ratio\s*:\s*\n\s*-\s*\[([^\]]+)\]", txt)
        if m:
            v = [float(x) for x in m.group(1).split(",")]
            if len(v) == 3 and all(x > 0 for x in v):
                return v
    except Exception as e:
        print(f"[警告] 读取 gear_ratio 失败({e})，按 3.0 处理", file=sys.stderr)
    return [3.0, 3.0, 3.0]


# ---------------------------------------------------------------------------
# 数据来源 1：底层在跑 → 读共享内存（不碰串口）
# ---------------------------------------------------------------------------
def bottom_alive(data):
    a = data.status.seq
    time.sleep(0.15)
    return data.status.seq != a


def read_from_shm():
    semid = sa.sem_open(sa.SEM_KEY)
    mm = sa.shm_map()
    data = sa.ArmSharedData.from_buffer(mm)
    if not bottom_alive(data):
        return None
    sa.sem_p(semid)
    counts = [int(data.status.motor_pos[i]) for i in range(3)]
    online = [bool(data.status.online[i]) for i in range(3)]
    seq = data.status.seq
    measured = bool(data.status.state & 0x02)
    sa.sem_v(semid)
    if not measured:
        print("[提示] 底层在运行，但它还没有实现回读实测值（state bit1=0），"
              "共享内存里的角度/计数都是 0、不可用。\n"
              "       要看真实角度：先停掉底层，本脚本会改为直连串口读取。", file=sys.stderr)
    return counts, online, f"共享内存（底层运行中，status.seq={seq}）"


# ---------------------------------------------------------------------------
# 数据来源 2：底层没跑 → 直连串口读 0x2A
# ---------------------------------------------------------------------------
def read_from_serial(trials=5):
    fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
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

        counts, online, notes = [], [], []
        for addr in (1, 2, 3):
            got, raw, bad = None, "", 0
            for _ in range(trials):
                termios.tcflush(fd, termios.TCIFLUSH)
                os.write(fd, pb.build_read(addr, 0x2A, 2))
                termios.tcdrain(fd)
                buf, t0 = b"", time.monotonic()
                while time.monotonic() - t0 < 0.08:
                    r, _, _ = select.select([fd], [], [], 0.005)
                    if r:
                        d = os.read(fd, 4096)
                        if d:
                            buf += d
                raw = buf.hex(" ").upper() if buf else "(超时/无应答)"
                # 必须校验 CRC：这几台驱动器偶发坏帧，不校验会读出天量离谱数值
                if (len(buf) >= 9 and buf[0] == addr and buf[1] == 0x04
                        and pb.crc16(buf[:7]).to_bytes(2, "little") == buf[7:9]):
                    got = struct.unpack(">i", buf[3:7])[0]
                    break
                if buf:
                    bad += 1
                time.sleep(0.03)
            counts.append(got if got is not None else 0)
            online.append(got is not None)
            notes.append(raw)
            if got is None:
                print(f"[警告] 地址 {addr} 读取失败（{bad} 次应答全部 CRC 校验失败或超时）：{raw}",
                      file=sys.stderr)
        return counts, online, f"直连串口 {DEV} @ {BAUD}"
    finally:
        os.close(fd)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="读取三台电机角度（只读）")
    ap.add_argument("--capture", action="store_true", help="把当前计数记为参考点并保存")
    ap.add_argument("--clear", action="store_true", help="清除已保存的参考点")
    ap.add_argument("--ref", help="临时参考点计数，格式 c1,c2,c3")
    ap.add_argument("--ratio", type=float, nargs=3, metavar=("G1", "G2", "G3"),
                    help="覆盖减速比（默认读 hardware_config.yaml）")
    ap.add_argument("--json", action="store_true", help="以 JSON 输出")
    args = ap.parse_args()

    if args.clear:
        if os.path.exists(REF_FILE):
            os.remove(REF_FILE)
            print(f"已清除参考点：{REF_FILE}")
        else:
            print("没有已保存的参考点")
        return

    ratio = list(args.ratio) if args.ratio else load_ratio()

    res = read_from_shm()
    if res is None:
        res = read_from_serial()
    counts, online, source = res

    # 参考点
    ref, ref_src = None, None
    if args.ref:
        try:
            v = [int(float(x)) for x in args.ref.split(",")]
            assert len(v) == 3
            ref, ref_src = v, "--ref"
        except Exception:
            print("--ref 格式应为 c1,c2,c3（三个整数计数）", file=sys.stderr)
            return 1
    elif os.path.exists(REF_FILE):
        try:
            j = json.load(open(REF_FILE))
            ref, ref_src = [int(x) for x in j["counts"]], f"已保存参考点({j.get('time', '?')})"
        except Exception as e:
            print(f"[警告] 参考点文件损坏({e})，忽略", file=sys.stderr)

    if args.capture:
        json.dump({"counts": counts, "time": time.strftime("%Y-%m-%d %H:%M:%S")},
                  open(REF_FILE, "w"), indent=2)
        ref, ref_src = list(counts), "刚保存（本次=参考点）"
        print(f"已把当前计数记为参考点：{counts}  → {REF_FILE}")
        print()

    out = {"source": source, "ratio": ratio, "ref_source": ref_src,
           "ref_counts": ref, "axes": []}
    print(f"数据来源：{source}")
    print(f"减速比  ：{ratio[0]}, {ratio[1]}, {ratio[2]}    (51200 计数 = 一圈 = 360°)")
    if ref:
        print(f"参考点  ：{ref}   来源：{ref_src}")
    else:
        print("参考点  ：无（只显示绝对角度；用 --capture 或 --ref 指定后可显示相对角度）")
    print()

    hdr = "  轴  在线   绝对计数    电机角(°)   关节角(°)  "
    if ref:
        hdr += "│  相对计数   相对电机角(°)  相对关节角(°)"
    print(hdr)
    for i in range(3):
        c = counts[i]
        if online[i]:
            line = (f"  {i+1}   是   {c:9d}   {c / DEG2CNT:10.3f}   {c / DEG2CNT / ratio[i]:9.3f}  ")
        else:                                  # 读失败：显示 -- 而不是 0，避免误解
            line = f"  {i+1}   否         --          --           --  "
        ax = {"axis": i + 1, "online": online[i], "counts": c,
              "motor_deg": round(c / DEG2CNT, 4),
              "joint_deg": round(c / DEG2CNT / ratio[i], 4)}
        if ref:
            rc = c - ref[i]
            line += (f"│  {rc:9d}   {rc / DEG2CNT:12.3f}   "
                     f"{rc / DEG2CNT / ratio[i]:12.3f}")
            ax.update({"rel_counts": rc, "rel_motor_deg": round(rc / DEG2CNT, 4),
                       "rel_joint_deg": round(rc / DEG2CNT / ratio[i], 4)})
        if not online[i]:
            line += "   ← 离线/读取失败，数值不可信"
        print(line)
        out["axes"].append(ax)

    if args.json:
        print()
        print(json.dumps(out, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    sys.exit(main() or 0)
