#!/usr/bin/env python3
"""把 PCD 的 x/y/z 整体平移（默认原地写回）。

【2026-11-04 更正：默认平移已停用，请勿再对 prior_3d_map.pcd 调用】
FAST-LIO 保存的 PCD 在 odom 系，原点是建图起点的 IMU 位姿。slam_toolbox 建 2D
栅格地图时 map->odom 初始为恒等变换，因此 2D 地图（my_map.pgm）的 map 系原点
**同样是**建图起点的 IMU，而不是 base_link。实测建图刚启动时
map->base_link = (-0.339, -0.028)，正好是 IMU 安装偏移的负值。

所以先验 PCD 保持 odom 系即可；再叠加 (0.3335, 0.0233) 会让重定位输出的
map->odom 多出同样的偏移，导航时 RViz 里的机器人比 Gazebo 中凭空靠近障碍物
0.33 m（实测 map->base_link 为 (0.012, -0.016)，真值为 (-0.305, -0.022)）。
build_3d_map.sh 已不再调用本脚本；保留它只是为了需要时手工平移调试。

用法：
    python3 shift_pcd.py in.pcd -o out.pcd --xy 0.33 0.02
    python3 shift_pcd.py in.pcd --z 0.0
"""
import argparse
import sys

import numpy as np

# 与 fast_lio/config/mid360_sim*.yaml 的 mapping.body_to_base_T 保持一致
# （雷达 30° 斜装：t = (0.346,0,0.93) + Ry(30°)·(0.011,0.02329,-0.04412)
#                       = (0.3334663, 0.02329, 0.886291)）
DEFAULT_XY = (0.3334663, 0.02329)


def read_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        order = []
        while True:
            line = f.readline().decode('ascii', 'replace').strip()
            if not line or line.startswith('#'):
                continue
            key, *vals = line.split()
            header[key] = vals
            order.append(key)
            if key == 'DATA':
                break
        data = f.read()
    return header, order, data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('-o', '--output', help='输出文件（默认原地覆盖）')
    ap.add_argument('--xy', nargs=2, type=float, default=DEFAULT_XY,
                    help='x y 平移（米）')
    ap.add_argument('--z', type=float, default=0.0, help='z 平移（米）')
    args = ap.parse_args()

    header, order, data = read_pcd(args.pcd)
    if header.get('DATA', [''])[0] != 'binary':
        sys.exit('只支持 binary PCD')
    fields = header['FIELDS']
    sizes = [int(s) for s in header['SIZE']]
    counts = [int(c) for c in header['COUNT']]
    n = int(header['POINTS'][0])
    dtype = np.dtype([(f, '<f4') if s == 4 else (f, '<f8')
                      for f, s, c in zip(fields, sizes, counts) if c == 1])
    if len(dtype) != len(fields):
        sys.exit('只支持 COUNT=1 的字段')
    arr = np.frombuffer(data, dtype=dtype, count=n).copy()
    arr['x'] += args.xy[0]
    arr['y'] += args.xy[1]
    arr['z'] += args.z

    out = args.output or args.pcd
    with open(out, 'wb') as f:
        for key in order:
            if key == 'DATA':
                f.write(b'DATA binary\n')
                break
            f.write(('%s %s\n' % (key, ' '.join(header[key]))).encode('ascii'))
        f.write(arr.tobytes())
    print(f'已将 {args.pcd} 平移 ({args.xy[0]:.4f}, {args.xy[1]:.4f}, {args.z:.4f}) m'
          f' -> {out}（{n} 点）')


if __name__ == '__main__':
    main()
