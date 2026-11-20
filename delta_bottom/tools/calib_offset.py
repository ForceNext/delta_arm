#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
关节零位偏置标定：让「当前姿态」对应指定的关节角。

原理（与底层一致）：
    关节角  = dir × 计数 ÷ (51200/360) ÷ 减速比 + offset
    下发    = dir × (关节角 − offset) × 减速比 × (51200/360)
所以把当前姿态标定为 θ_target：
    offset = θ_target − dir × 计数 ÷ (51200/360) ÷ 减速比
dir / 减速比 / offset 都读自 configs/hardware_config.yaml（joint.dir / joint.gear_ratio / joint.offset）。

用法：
  python3 calib_offset.py 35 35 35            # 算出偏置并打印（不写文件）
  python3 calib_offset.py 0 0 0               # 把当前姿态定义为关节 0°
  python3 calib_offset.py 35 35 35 --write    # 写回 hardware_config.yaml（备份 .bak）
  python3 calib_offset.py 35 35 35 --counts=-206,48356,40204  # 不接硬件，直接给计数
                                              （值以负号开头时必须用 --counts=xxx 形式）

注意：标定要求机械臂**静止在当前位形**；底层在跑时串口被占用，请先停掉 delta_bottom。
"""

import argparse
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import read_angles as ra        # noqa: E402  读计数 / 读 ratio+dir / 读当前 offset

CFG = os.path.join(HERE, "..", "configs", "hardware_config.yaml")
DEG2CNT = 51200.0 / 360.0


def yaml_offset_line(offs):
    return "    - [" + ",".join(f"{o:.7f}" for o in offs) + "]"


def write_offsets(offs):
    """把 joint.offset 写回 yaml（先备份 .bak）；没有该字段就插在 joint.dir 后面"""
    txt = open(CFG, encoding="utf-8").read()
    shutil.copyfile(CFG, CFG + ".bak")

    pat = re.compile(r"(offset\s*:\s*\n)(\s*-?\s*\[[^\]]*\])")
    if pat.search(txt):
        new = pat.sub(lambda m: m.group(1) + yaml_offset_line(offs), txt, count=1)
    else:
        # 插在 joint.dir 的列表行之后
        m = re.search(r"(^  dir:\s*\n\s*-?\s*\[[^\]]*\]\s*\n)", txt, re.M)
        if not m:
            print("✗ 配置里找不到 joint.dir，无法定位插入位置", file=sys.stderr)
            return False
        ins = ("\n  offset:\n" + yaml_offset_line(offs) +
               " #每轴关节零位偏置（度）：关节角 = 方向×计数/(51200/360)/减速比 + offset\n\n")
        new = txt[:m.end()] + ins + txt[m.end():]
    open(CFG, "w", encoding="utf-8").write(new)
    print(f"\n已写入 {os.path.normpath(CFG)}（原文件备份为 .bak）")
    return True


def main():
    ap = argparse.ArgumentParser(description="关节零位偏置标定（当前姿态 → 指定关节角）")
    ap.add_argument("target", nargs="*", type=float, help="三个目标关节角（度），如 35 35 35")
    ap.add_argument("--counts", help="直接给三个电机计数 c1,c2,c3（不读硬件）")
    ap.add_argument("--write", action="store_true", help="写回 hardware_config.yaml")
    args = ap.parse_args()

    if len(args.target) == 0:
        args.target = [35.0, 35.0, 35.0]
    if len(args.target) != 3:
        print("请给 3 个目标关节角，例如: calib_offset.py 35 35 35", file=sys.stderr)
        return 1

    ratio = ra.load_ratio()
    jdir = ra.load_dir()
    old_off = ra.load_offset()

    # 当前计数
    if args.counts:
        try:
            counts = [int(float(x)) for x in args.counts.split(",")]
            assert len(counts) == 3
        except Exception:
            print("--counts 格式应为 c1,c2,c3", file=sys.stderr)
            return 1
    else:
        counts, online, src = ra.read_from_serial()
        if not all(online):
            print(f"[拒绝] 有轴读不到计数（{online}），先查连线/供电；或用 --counts 手动指定",
                  file=sys.stderr)
            return 1

    print(f"减速比 {ratio}  方向 {[int(d) for d in jdir]}  当前 offset {old_off}\n")
    print("  轴  当前计数     旧读数(°)    目标(°)     新 offset(°)     校验: θ=目标 → 计数")
    offs = []
    for i in range(3):
        raw_old = jdir[i] * counts[i] / (DEG2CNT * ratio[i]) + old_off[i]   # 现在的读数
        off = args.target[i] - jdir[i] * counts[i] / (DEG2CNT * ratio[i])   # 需要的偏置
        offs.append(off)
        back = jdir[i] * (args.target[i] - off) * ratio[i] * DEG2CNT
        print(f"  {i+1}   {counts[i]:9d}  {raw_old:10.4f}  {args.target[i]:9.3f}  "
              f"{off:14.7f}   {back:11.3f}（当前 {counts[i]}）")

    print("\n配置行：")
    print("  offset:")
    print(yaml_offset_line(offs))
    print("\n提示：偏置是「软件零点」，与 set_zero.py（改驱动器计数零点）是两套机制，不要混用；")
    print("      若用 set_zero.py 清零，之后读数会等于 offset，需要重新跑本脚本把 offset 设成 0 或目标值。")

    if args.write:
        if not write_offsets(offs):
            return 1
        print("改动需要重新编译（./build.sh）并在下次运行 delta_bottom 时生效。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
