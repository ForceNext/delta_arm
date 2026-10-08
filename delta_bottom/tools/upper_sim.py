#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上位机模拟脚本：持续向共享内存发布末端坐标 (x,y,z)，让 delta 机械臂走圆。

底层 ArmDriveTask 以 200Hz 读共享内存里的 x/y/z，当作末端坐标（单位 mm，
z 正方向向上，平台在基座下方为负），逆解出关节角、乘传动比后下发到电机。

共享内存 / 信号量参数与底层一致：
  - 共享内存名：/delta_arm      （POSIX shm_open，对应 /dev/shm/delta_arm）
  - 信号量 key ：0x5A11         （System V 信号量，= ARM_SEM_KEY）

用法：
  python3 upper_sim.py                     # 默认：半径 30mm、高度 -250mm、50Hz、0.25 圈/秒
  python3 upper_sim.py 30 -250 50 0.5      # 半径 / 高度 / 频率 / 圈速
Ctrl+C 退出；退出前自动写 mode=0（空闲），底层停止下发。
"""

import os
import sys
import time
import signal
import math
import ctypes
import ctypes.util
import mmap

# ---------------------------------------------------------------------------
# 与底层保持一致
# ---------------------------------------------------------------------------
SHM_NAME = "/delta_arm"     # 共享内存名（= ARM_SHARED_MEMORY_NAME）
SEM_KEY  = 0x5A11           # System V 信号量 key（= ARM_SEM_KEY）


# ---------------------------------------------------------------------------
# C 结构体布局（必须与 SharedMemory.h 的 ArmCommand / ArmStatus 完全一致）
# ---------------------------------------------------------------------------
class ArmCommand(ctypes.Structure):
    _fields_ = [
        ("seq",  ctypes.c_uint64),   # 写入序号
        ("mode", ctypes.c_uint8),    # 0=空闲 1=位置模式
        ("x",    ctypes.c_double),   # 末端 x 坐标（mm）
        ("y",    ctypes.c_double),   # 末端 y 坐标（mm）
        ("z",    ctypes.c_double),   # 末端 z 坐标（mm，向下为负）
    ]


class ArmStatus(ctypes.Structure):
    _fields_ = [
        ("seq",       ctypes.c_uint64),
        ("state",     ctypes.c_uint8),
        ("online",    ctypes.c_uint8 * 3),
        ("theta",     ctypes.c_double * 3),
        ("motor_pos", ctypes.c_double * 3),
        ("x",         ctypes.c_double),
        ("y",         ctypes.c_double),
        ("z",         ctypes.c_double),
    ]


class ArmSharedData(ctypes.Structure):
    _fields_ = [
        ("command", ArmCommand),
        ("status",  ArmStatus),
    ]


DATA_SIZE = ctypes.sizeof(ArmSharedData)
assert DATA_SIZE == 128, f"结构体大小应为 128，实际 {DATA_SIZE}（与 C++ 不一致！）"


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
    """返回 (radius_mm, z_mm, rate_hz, rev_per_sec)。"""
    radius = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    z      = float(sys.argv[2]) if len(sys.argv) > 2 else -250.0
    rate   = float(sys.argv[3]) if len(sys.argv) > 3 else 50.0
    rev    = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25
    return radius, z, rate, rev


def main():
    radius, z, rate, rev = parse_args()

    semid = sem_open(SEM_KEY)
    mm = shm_map()
    data = ArmSharedData.from_buffer(mm)

    running = True

    def on_exit(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)

    period = 1.0 / rate
    deg_per_tick = 360.0 * rev / rate     # 每拍增加的角度

    print(f"上位机模拟启动：半径 {radius}mm、高度 {z}mm、{rate}Hz、{rev} 圈/秒")
    print(f"共享内存 /dev/shm{SHM_NAME}，信号量 key=0x{SEM_KEY:X}，结构体 {DATA_SIZE} 字节")

    seq = 0
    angle = 0.0
    t0 = time.monotonic()
    next_t = t0 + period
    last_report = t0

    try:
        while running:
            seq += 1
            angle += deg_per_tick
            x = radius * math.cos(math.radians(angle))
            y = radius * math.sin(math.radians(angle))

            sem_p(semid)
            data.command.mode = 1
            data.command.seq = seq
            data.command.x = x
            data.command.y = y
            data.command.z = z
            sx, sy, sz = data.status.x, data.status.y, data.status.z
            bottom_seq = data.status.seq    # 底层回读序号（>0 说明底层在消费）
            sem_v(semid)

            now = time.monotonic()
            if now - last_report >= 1.0:
                last_report = now
                print(f"t={now - t0:5.1f}s  目标=({x:6.1f},{y:6.1f},{z:6.1f})  "
                      f"回显=({sx:6.1f},{sy:6.1f},{sz:6.1f})  底层seq={bottom_seq}")

            # 睡到下一个周期边界，避免累积漂移
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic() + period
    finally:
        # 退出前置空闲，底层停止下发
        sem_p(semid)
        data.command.mode = 0
        sem_v(semid)
        print("已写 mode=0（空闲），退出")


if __name__ == "__main__":
    main()
