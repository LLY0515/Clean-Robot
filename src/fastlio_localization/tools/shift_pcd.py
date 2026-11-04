#!/usr/bin/env python3
"""把 PCD 的 x/y/z 整体平移（默认原地写回）。

用途：FAST-LIO 保存的 PCD 在 odom 系，原点是建图起点的 **IMU**；而 2D 栅格地图
（my_map.pgm）的原点是建图起点的 **base_link**（世界原点）。两者相差 IMU 的安装
偏移 t = (0.3335, 0.0233, 0.8863) m（见 URDF base_link→livox→livox_imu）。

不平移的话，3D 重定位输出的 map 系与 2D 静态地图相差 0.33 m，Nav2 的 static_layer
和实际障碍物对不上，会出现“RViz 里没撞、Gazebo 里撞了”。

这里只平移 x/y：z 保持 FAST-LIO 的雷达平面为 0，重定位节点的 constrain_planar
会把 map->odom 的 z 约束为 0，平移 z 反而会破坏 ICP 的竖向对齐。

用法：
    python3 shift_pcd.py prior_3d_map.pcd                 # 用默认偏移，原地更新
    python3 shift_pcd.py in.pcd -o out.pcd --xy 0.33 0.02
    python3 shift_pcd.py in.pcd --z 0.0
"""
import argparse
import sys

import numpy as np

# 与 fast_lio/config/mid360_sim*.yaml 的 mapping.body_to_base_T 保持一致
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
