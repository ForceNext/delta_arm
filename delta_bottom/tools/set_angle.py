#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
向共享内存发布一个目标角度，让电机转到对应角度（闭环绝对位置模式）。

底层 delta_bottom 的 ArmDriveTask 以控制频率读共享内存，把 x/y/z 直接作为
电机 id1/id2/id3 的目标角度（单位：度）下发（mode=1 时生效）。

共享内存 / 信号量参数与底层一致（见 upper_sim.py）：
  - 共享内存名：/delta_arm      （POSIX shm_open，对应 /dev/shm/delta_arm）
  - 信号量 key ：0x5A11         （System V 信号量，= ARM_SEM_KEY）

用法：
  python3 set_angle.py 30            # 三轴 x/y/z 都转到 30°
  python3 set_angle.py 30 x          # 只有 x（id1）转到 30°，其余轴保持原值
  python3 set_angle.py 30 xy         # x/y 转到 30°
  python3 set_angle.py               # 不带参数则交互式输入角度
"""

import os
import sys
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
    """返回 (angle, axes)。角度可从命令行或交互式输入。"""
    angle = None
    axes = "xyz"

    if len(sys.argv) >= 2:
        angle = float(sys.argv[1])
    if len(sys.argv) >= 3:
        axes = sys.argv[2].lower()

    if angle is None:
        angle = float(input("请输入目标角度（度）: ").strip())
        a = input("要控制的轴 (x/y/z，默认 xyz): ").strip()
        axes = a.lower() or "xyz"

    for c in axes:
        if c not in "xyz":
            print(f"无效轴 '{c}'（只能是 x/y/z）")
            sys.exit(1)
    return angle, axes


def main():
    angle, axes = parse_args()

    semid = sem_open(SEM_KEY)
    mm = shm_map()
    data = ArmSharedData.from_buffer(mm)

    # 加锁后：仅更新指定轴的角度，其余轴保持原值；seq +1，mode 置 1
    sem_p(semid)
    data.command.mode = 1
    data.command.seq += 1
    if "x" in axes:
        data.command.x = angle
    if "y" in axes:
        data.command.y = angle
    if "z" in axes:
        data.command.z = angle
    x, y, z = data.command.x, data.command.y, data.command.z
    state = data.status.state
    bottom_seq = data.status.seq
    sem_v(semid)

    print(f"已发布角度 {angle:.2f}° 到轴 '{axes}'")
    print(f"  x={x:.2f}°  y={y:.2f}°  z={z:.2f}°  mode=1")
    print(f"  底层状态 state={state}，回读 seq={bottom_seq}")


if __name__ == "__main__":
    main()
