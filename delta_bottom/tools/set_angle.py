#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
向共享内存发布三个【角度】（单位：度）。

角度含义由底层开关 APPLY_GEAR_RATIO_ON_SEND（user/task/src/arm_drive_task.cpp）决定：
    false（当前）：theta 直接作为【电机角】 → 计数 = theta × 51200/360
    true         ：theta 作为【关节角】   → 计数 = theta × 减速比 × 51200/360

注意：共享内存里不再是末端 x/y/z 坐标（坐标正逆解已移到上位机）。

共享内存 / 信号量参数与底层一致：
  - 共享内存名：/delta_arm      （POSIX shm_open，对应 /dev/shm/delta_arm）
  - 信号量 key ：0x5A11         （System V 信号量，= ARM_SEM_KEY）

用法：
  python3 set_angle.py 30 45 60         # 三个角度（度），按电机 1/2/3 顺序
  python3 set_angle.py -10 5 20         # 也可给负角度（需已做过位置计数器零点标定）
  python3 set_angle.py                  # 不带参数则交互式输入
  python3 set_angle.py --idle           # 只写 mode=0（空闲），底层停止下发

护栏（只剩"不拿垃圾当基准"这一条；**跳变大小不限幅、不拒绝**）：
  * 底层没在运行（status.seq 不变）/ 还没回传实测值 → 拒绝写入（否则等于以 0 为基准乱动）；
  * 跳变超过 --hint-jump（默认 20°，单位同 --unit）→ **只打印提示，照常下发**；
  * 用 --force 可跳过上面的拒绝。
  --unit 必须与底层开关 APPLY_GEAR_RATIO_ON_SEND 一致：false→motor（当前），true→joint。

提示：脚本被 kill -9 / 异常退出时，共享内存里可能残留 mode=1 + 上次的角度；
     接电机重启底层前先运行一次 --idle，避免底层一进入运行就按旧角度下发。
"""

import ctypes
import ctypes.util
import mmap
import os
import sys
import time

# ---------------------------------------------------------------------------
# 与底层保持一致
# ---------------------------------------------------------------------------
SHM_NAME = "/delta_arm"     # 共享内存名（= ARM_SHARED_MEMORY_NAME）
SEM_KEY  = 0x5A11           # System V 信号量 key（= ARM_SEM_KEY）


# ---------------------------------------------------------------------------
# C 结构体布局（必须与 SharedMemory.h 的 ArmCommand / ArmStatus 完全一致）
# ---------------------------------------------------------------------------
ARM_SHM_MAGIC = 0x444C5431   # 布局版本魔数（= ARM_SHM_MAGIC，布局一变就改）


class ArmCommand(ctypes.Structure):
    _fields_ = [
        ("seq",      ctypes.c_uint64),     # 写入序号
        ("mode",     ctypes.c_uint8),      # 0=空闲 1=角度模式
        ("reserved", ctypes.c_uint8 * 3),  # 对齐填充
        ("magic",    ctypes.c_uint32),     # 布局版本魔数
        ("theta",    ctypes.c_double * 3), # 三个【关节角】（度）
    ]


class ArmStatus(ctypes.Structure):
    _fields_ = [
        ("seq",         ctypes.c_uint64),
        ("state",       ctypes.c_uint8),       # bit0=运行 bit1=实测
        ("online",      ctypes.c_uint8 * 3),
        ("theta",       ctypes.c_double * 3),  # 电机角（度，已乘减速比）
        ("motor_pos",   ctypes.c_double * 3),  # 电机位置（计数）
        ("joint_theta", ctypes.c_double * 3),  # 实测关节角（度）
    ]


class ArmSharedData(ctypes.Structure):
    _fields_ = [
        ("command", ArmCommand),
        ("status",  ArmStatus),
    ]


DATA_SIZE = ctypes.sizeof(ArmSharedData)
assert DATA_SIZE == 128, f"结构体大小应为 128，实际 {DATA_SIZE}（与 C++ 不一致！）"
assert (ArmCommand.mode.offset, ArmCommand.magic.offset, ArmCommand.theta.offset) == (8, 12, 16), "ArmCommand 字段偏移与 C++ 不一致！"
assert (ArmStatus.state.offset, ArmStatus.theta.offset, ArmStatus.motor_pos.offset,
        ArmStatus.joint_theta.offset) == (8, 16, 40, 64), "ArmStatus 字段偏移与 C++ 不一致！"


# ---------------------------------------------------------------------------
# System V 信号量（与 sem_com.h 的 sem_p / sem_v 等价）
# ---------------------------------------------------------------------------
IPC_CREAT = 0o1000
IPC_EXCL  = 0o2000
SEM_UNDO  = 0o10000
IPC_SETVAL = 16


class sembuf(ctypes.Structure):
    _fields_ = [
        ("sem_num", ctypes.c_ushort),
        ("sem_op",  ctypes.c_short),
        ("sem_flg", ctypes.c_short),
    ]


_libc = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6", use_errno=True)
_libc.semget.restype = ctypes.c_int
_libc.semget.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
_libc.semop.restype = ctypes.c_int
_libc.semop.argtypes = [ctypes.c_int, ctypes.POINTER(sembuf), ctypes.c_size_t]
_libc.semctl.restype = ctypes.c_int


def sem_open(key):
    """打开（或首次创建）互斥信号量，返回 semid。首次创建者初始化为 1。"""
    semid = _libc.semget(key, 1, IPC_CREAT | IPC_EXCL | 0o666)
    if semid < 0:
        semid = _libc.semget(key, 1, 0o666)
        if semid < 0:
            raise OSError(f"semget 打开失败 (key=0x{key:X})")
    else:
        if _libc.semctl(semid, 0, IPC_SETVAL, 1) < 0:
            raise OSError("semctl SETVAL 失败")
    return semid


def sem_p(semid):
    buf = sembuf(0, -1, SEM_UNDO)
    if _libc.semop(semid, ctypes.byref(buf), 1) < 0:
        raise OSError("semop P 失败")


def sem_v(semid):
    buf = sembuf(0, 1, SEM_UNDO)
    if _libc.semop(semid, ctypes.byref(buf), 1) < 0:
        raise OSError("semop V 失败")


def shm_map():
    """打开（或创建）共享内存并 mmap，返回可写的映射对象。"""
    path = "/dev/shm" + SHM_NAME
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
    try:
        if os.fstat(fd).st_size < DATA_SIZE:
            os.ftruncate(fd, DATA_SIZE)
        return mmap.mmap(fd, DATA_SIZE, mmap.MAP_SHARED,
                         mmap.PROT_READ | mmap.PROT_WRITE)
    finally:
        os.close(fd)


def parse_args():
    """返回 (三个角度, unit, hint_jump, force)；--idle 时角度为 None。"""
    import argparse
    ap = argparse.ArgumentParser(description="向共享内存发布三个角度（度）")
    ap.add_argument("theta", nargs="*", type=float, help="三个角度（度），不给则交互式输入")
    ap.add_argument("--idle", action="store_true", help="只写 mode=0（空闲）")
    ap.add_argument("--unit", choices=["motor", "joint"], default="motor",
                    help="角度含义，必须与底层 APPLY_GEAR_RATIO_ON_SEND 一致（默认 motor）")
    ap.add_argument("--hint-jump", type=float, default=20.0,
                    help="跳变超过该值（度）只打印提示，不阻止下发，默认 20")
    ap.add_argument("--force", action="store_true", help="跳过护栏")
    a = ap.parse_args()
    if a.idle:
        return None, a.unit, a.hint_jump, a.force
    if len(a.theta) == 3:
        return tuple(a.theta), a.unit, a.hint_jump, a.force
    if len(a.theta) != 0:
        ap.error("请给 3 个角度，或用 --idle")
    print("输入三个【角度】（度，按电机 1/2/3；当前底层按电机角直接下发）")
    return (tuple(float(input(f"  角度 {i + 1} (deg): ").strip()) for i in range(3)),
            a.unit, a.hint_jump, a.force)


def bottom_alive(data, wait=0.15):
    """判断底层是否在运行：它的 status.seq 每拍都在递增。

    共享内存里的 status 在底层退出后会**保持最后一次的值**（含 state bit1=1），
    若只看 state 会把陈旧数据当成"实测值"来当基准 —— 所以用 seq 是否变化来判断。

    注意：这里**故意不加锁**（只为判活，读到撕裂值也无所谓），
    而且会 sleep，所以必须在"没有持锁"的时候调用，否则会与底层抢锁、拖慢实时循环。
    """
    a = data.status.seq
    time.sleep(wait)
    return data.status.seq != a


def safety_check(data, targets, unit, hint_jump, force):
    """返回 True 表示允许写入（调用时须已持锁）。

    只拦"没有可信基准"这一种情况（底层没在跑 / 还没回传实测值）；
    **不做跳变限幅**：跳变再大也照常下发，只在超过 hint_jump 时打印提示。
    """
    state = data.status.state
    if force:
        return True
    if not (state & 0x02):
        print("\033[1;31m[拒绝] 底层还没回传实测值（state bit1=0）：此刻写角度会以\"0\"为基准，\n"
              "       属于拿垃圾当基准，请等底层回传实测值后再试，或用 --force 跳过。\033[0m")
        return False
    ref = [data.status.theta[i] if unit == "motor" else data.status.joint_theta[i]
           for i in range(3)]
    worst = max(abs(targets[i] - ref[i]) for i in range(3))
    if worst > hint_jump:
        print(f"\033[1;33m[提示] 本次最大跳变 {worst:.2f}°（单位={unit}，阈值 {hint_jump}°）：\n"
              f"       实测 = ({ref[0]:.3f}, {ref[1]:.3f}, {ref[2]:.3f})\n"
              f"       目标 = ({targets[0]:.3f}, {targets[1]:.3f}, {targets[2]:.3f})\n"
              f"       按绝对位置直接下发（不限幅）；请自行确认机械臂行程与安全。\033[0m")
    return True


def main():
    args, unit, hint_jump, force = parse_args()

    semid = sem_open(SEM_KEY)
    mm = shm_map()
    data = ArmSharedData.from_buffer(mm)

    if args is None:                      # --idle：只置空闲，不写角度
        sem_p(semid)
        data.command.magic = ARM_SHM_MAGIC
        data.command.mode = 0
        state = data.status.state
        bottom_seq = data.status.seq
        sem_v(semid)
        print(f"已写 mode=0（空闲） 底层 state=0x{state:02X}  回读 seq={bottom_seq}")
        return

    t1, t2, t3 = args

    # ① 存活检查（必须在取锁之前：它内部会 sleep）
    if not force and not bottom_alive(data):
        print("\033[1;31m[拒绝] 底层似乎没有在运行（status.seq 不变化）："
              "共享内存里可能是上次留下的陈旧状态，不能当基准。\n"
              "       请先启动底层；确认无误可用 --force 跳过。\033[0m")
        return

    # ② 护栏：写入前先看底层实测值（只在没有可信基准时拒绝；跳变不限幅）
    sem_p(semid)
    ok = safety_check(data, (t1, t2, t3), unit, hint_jump, force)
    sem_v(semid)
    if not ok:
        return

    # 加锁后写入三个角度；seq +1，mode 置 1
    sem_p(semid)
    data.command.magic = ARM_SHM_MAGIC
    data.command.mode = 1
    data.command.seq += 1
    data.command.theta[0] = t1
    data.command.theta[1] = t2
    data.command.theta[2] = t3
    state = data.status.state
    online = list(data.status.online)
    jt = [data.status.joint_theta[i] for i in range(3)]
    mp = [data.status.motor_pos[i] for i in range(3)]
    bottom_seq = data.status.seq
    sem_v(semid)

    print(f"已发布角度 (°) = ({t1:.3f}, {t2:.3f}, {t3:.3f})   mode=1")
    print(f"  底层 state=0x{state:02X} (运行={state & 1}, 实测={1 if state & 2 else 0})  "
          f"online={online}  回读 seq={bottom_seq}")
    print(f"  实测关节角(°) = ({jt[0]:.3f}, {jt[1]:.3f}, {jt[2]:.3f})")
    print(f"  实测计数      = ({mp[0]:.0f}, {mp[1]:.0f}, {mp[2]:.0f})")


if __name__ == "__main__":
    main()
