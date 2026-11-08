#!/usr/bin/env python3
"""把 /map（nav_msgs/OccupancyGrid）存成 nav2 可用的 pgm + yaml。

替代 nav2 的 map_saver_cli。后者内部只等 2 秒（save_map_timeout 默认值），
而 slam_toolbox 每 5 秒才发布一次地图、rtabmap 只在栅格变化时发布，
机器人一停就容易错过消息，表现为
    [ERROR] [map_saver]: Failed to spin map subscription
然后地图没写出来。

本工具：先等到第一帧，再持续收 settle 秒取最新的一帧写盘，因此不会因为
"那两秒恰好没有新消息"而失败。

用法：
    save_map.py <输出前缀> [--topic /map] [--timeout 30] [--settle 3]
"""

import argparse
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

UNKNOWN = 205
# nav2 的写法：占用概率 0..100 线性映射到 255..0（越黑越"占用"）
FREE = 254


class Saver(Node):
    def __init__(self, topic):
        super().__init__('save_map')
        self.msg = None
        self.count = 0
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            # 用 VOLATILE：既能接 slam_toolbox 的 transient_local（latch）发布者，
            # 也能接 rtabmap 的普通发布者。反过来则 QoS 不兼容、收不到数据。
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(OccupancyGrid, topic, self.cb, qos)

    def cb(self, m):
        self.msg = m
        self.count += 1


def spin_for(node, seconds):
    end = node.get_clock().now().nanoseconds / 1e9 + seconds
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds / 1e9 >= end:
            break


def to_pgm(grid):
    h, w = grid.info.height, grid.info.width
    data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
    img = np.full((h, w), UNKNOWN, dtype=np.uint8)
    known = data >= 0
    occ = np.zeros((h, w), dtype=np.float32)
    occ[known] = data[known].astype(np.float32) / 100.0
    img[known] = np.round((1.0 - occ[known]) * (FREE - 0)).astype(np.uint8)
    # OccupancyGrid 的行从 y 最小开始，PGM 从 y 最大开始
    return np.flipud(img)


def write_pgm(path, img):
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefix', help='输出前缀，会生成 <prefix>.pgm/.yaml')
    ap.add_argument('--topic', default='/map')
    ap.add_argument('--timeout', type=float, default=30.0,
                    help='等第一帧 /map 的最长时间（秒）')
    ap.add_argument('--settle', type=float, default=3.0,
                    help='拿到第一帧后继续收集的时间（秒），取最新一帧')
    args = ap.parse_args()

    rclpy.init()
    node = Saver(args.topic)

    # 先等第一帧
    end = node.get_clock().now().nanoseconds / 1e9 + args.timeout
    while rclpy.ok() and node.msg is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds / 1e9 >= end:
            break
    if node.msg is None:
        print('✗ %.0f 秒内没收到 %s' % (args.timeout, args.topic), file=sys.stderr)
        rclpy.shutdown()
        return 1

    spin_for(node, args.settle)
    m = node.msg

    img = to_pgm(m)
    pgm_path = args.prefix + '.pgm'
    yaml_path = args.prefix + '.yaml'
    write_pgm(pgm_path, img)

    with open(yaml_path, 'w') as f:
        f.write('image: %s\n' % os.path.basename(pgm_path))
        f.write('mode: trinary\n')
        f.write('resolution: %g\n' % m.info.resolution)
        f.write('origin: [%.4f, %.4f, 0.0]\n'
                % (m.info.origin.position.x, m.info.origin.position.y))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')

    print('✓ 已保存 %s (%dx%d, %d 个占用格)'
          % (pgm_path, img.shape[1], img.shape[0], int((img < 100).sum())))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
