#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上位机模拟脚本：向共享内存写入 xyz（三个电机的目标角度，单位：度），
模拟上位机发送指令，默认让电机 x（id1）以「每秒一圈」的速度连续旋转。

底层 delta_bottom（user/main.cpp）以 200Hz 读该共享内存，把 x/y/z 直接作为
电机 id1/id2/id3 的目标角度（闭环绝对位置模式 0x00F2）下发。

共享内存 / 信号量参数必须和底层一致（见
delta_bottom/common/include/Utilities/SharedMemory.h）：
  - 共享内存名：/delta_arm      （POSIX shm_open，对应 /dev/shm/delta_arm）
  - 信号量 key ：0x5A11         （System V 信号量，= ARM_SEM_KEY）

用法：
  python3 upper_sim.py            # x 轴每秒 1 圈
  python3 upper_sim.py xyz        # x/y/z 三轴一起，每秒 1 圈
  python3 upper_sim.py xy 2.0     # x/y 两轴，每秒 2 圈
Ctrl+C 退出；退出前会自动写入 mode=0（空闲），让底层停止下发。
"""

import os
import sys
import time
import signal
import ctypes
import ctypes.util
import mmap

# ---------------------------------------------------------------------------
# 与底层保持一致
# ---------------------------------------------------------------------------
SHM_NAME = "/delta_arm"     # 共享内存名（= ARM_SHARED_MEMORY_NAME）
SEM_KEY  = 0x5A11           # System V 信号量 key（= ARM_SEM_KEY）
RATE     = 200              # 写入频率 Hz（与底层控制频率一致）

# ---------------------------------------------------------------------------
# C 结构体布局（必须与 SharedMemory.h 的 ArmCommand / ArmStatus 完全一致）
# ---------------------------------------------------------------------------
class ArmCommand(ctypes.Structure):
    _fields_ = [
        ("seq",  ctypes.c_uint64),   # 写入序号
        ("mode", ctypes.c_uint8),    # 0=空闲 1=位置模式
        ("x",    ctypes.c_double),   # 电机 id1 目标角度（度）
        ("y",    ctypes.c_double),   # 电机 id2 目标角度（度）
        ("z",    ctypes.c_double),   # 电机 id3 目标角度（度）
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
SEM_UNDO  = 0o10000        # sem_com.h 里的 SEM_UNDO
IPC_SETVAL = 16            # semctl 命令码


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
_libc.semctl.restype = ctypes.c_int   # 变参函数，不设 argtypes；SETVAL 直接传 int


def sem_open(key):
    """打开（或首次创建）互斥信号量，返回 semid。首次创建者初始化为 1。"""
    semid = _libc.semget(key, 1, IPC_CREAT | IPC_EXCL | 0o666)
    if semid < 0:
        # 已存在：只打开，不重置（避免覆盖他人持有的锁）
        semid = _libc.semget(key, 1, 0o666)
        if semid < 0:
            raise OSError(f"semget 打开失败 (key=0x{key:X})")
    else:
        # 首次创建：初始化为 1（互斥锁，未上锁）
        if _libc.semctl(semid, 0, IPC_SETVAL, 1) < 0:
            raise OSError("semctl SETVAL 失败")
    return semid


def sem_p(semid):
    """P 操作（加锁）。"""
    buf = sembuf(0, -1, SEM_UNDO)
    if _libc.semop(semid, ctypes.byref(buf), 1) < 0:
        raise OSError("semop P 失败")


def sem_v(semid):
    """V 操作（解锁）。"""
    buf = sembuf(0, 1, SEM_UNDO)
    if _libc.semop(semid, ctypes.byref(buf), 1) < 0:
        raise OSError("semop V 失败")


def shm_map():
    """打开（或创建）共享内存并 mmap，返回可写的映射对象。"""
    path = "/dev/shm" + SHM_NAME
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
    try:
        # 保证大小与 C++ 结构体一致；底层已创建时 ftruncate 到同样大小是无副作用
        if os.fstat(fd).st_size < DATA_SIZE:
            os.ftruncate(fd, DATA_SIZE)
        return mmap.mmap(fd, DATA_SIZE, mmap.MAP_SHARED,
                         mmap.PROT_READ | mmap.PROT_WRITE)
    finally:
        os.close(fd)   # mmap 之后即可关闭 fd


def main():
    axes = sys.argv[1] if len(sys.argv) > 1 else "x"
    rps  = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    for c in axes:
        if c not in "xyz":
            print(f"无效轴 '{c}'（只能是 x/y/z）")
            sys.exit(1)

    semid = sem_open(SEM_KEY)
    mm = shm_map()
    data = ArmSharedData.from_buffer(mm)

    deg_per_tick = 360.0 * rps / RATE
    period = 1.0 / RATE

    print(f"上位机模拟启动：写入频率 {RATE}Hz，每秒 {rps} 圈，驱动轴 '{axes}'")
    print(f"共享内存 /dev/shm{SHM_NAME}，信号量 key=0x{SEM_KEY:X}，"
          f"结构体 {DATA_SIZE} 字节")

    running = True

    def on_exit(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)

    seq = 0
    angle = 0.0
    t0 = time.monotonic()
    next_t = t0 + period
    last_report = t0

    try:
        while running:
            seq += 1
            angle += deg_per_tick

            sem_p(semid)
            data.command.seq = seq
            data.command.mode = 1
            if "x" in axes:
                data.command.x = angle
            if "y" in axes:
                data.command.y = angle
            if "z" in axes:
                data.command.z = angle
            cx, cy, cz = data.command.x, data.command.y, data.command.z
            bottom_seq = data.status.seq    # 底层回读序号（>0 说明底层在消费）
            sem_v(semid)

            now = time.monotonic()
            if now - last_report >= 1.0:
                last_report = now
                print(f"t={now - t0:6.1f}s  x={cx:9.2f}°  y={cy:9.2f}°  "
                      f"z={cz:9.2f}°  底层回读seq={bottom_seq}")

            # 睡到下一个周期边界，避免累积漂移
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic() + period
    finally:
        # 退出前置空闲，底层停止下发（mmap 由内核在进程退出时自动释放，无需手动 close）
        sem_p(semid)
        data.command.mode = 0
        sem_v(semid)
        print("已写 mode=0（空闲），退出")


if __name__ == "__main__":
    main()
