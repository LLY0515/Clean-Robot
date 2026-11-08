#!/usr/bin/env python3
"""Wait until a ROS 2 topic actually delivers messages, then exit 0.

为什么需要这个脚本：
    本机的 `ros2 topic echo` 会因 ros2cli 守护进程异常直接抛出
    "!rclpy.ok()" 并返回非 0 —— 即使话题上确实有数据。build_map.sh 以前用它
    做"等 /scan 有数据"的探测，结果仿真一切正常却报
    "✗ 60 秒内 /scan 仍无数据"。这里直接用 rclpy 订阅，绕开 ros2cli。

用法：
    topic_ready.py /scan --timeout 60 --best-effort
    topic_ready.py /map  --timeout 30

退出码：0 = 在超时前收到了 --count 条消息；1 = 超时。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message


def resolve_type(node: Node, topic: str, timeout: float = 10.0):
    """从 ROS graph 里查话题的消息类型（话题出现可能比节点启动慢）。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for name, types in node.get_topic_names_and_types():
            if name == topic and types:
                return types[0]
        time.sleep(0.2)
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('topic', help='话题名，例如 /scan')
    ap.add_argument('--timeout', type=float, default=30.0, help='超时秒数（默认 30）')
    ap.add_argument('--count', type=int, default=1, help='至少收到多少条消息（默认 1）')
    ap.add_argument('--type', dest='msg_type',
                    help='消息类型；不给则从 ROS graph 自动查询')
    ap.add_argument('--best-effort', action='store_true',
                    help='用 BEST_EFFORT 订阅（/scan 这类传感器话题必须加）')
    ap.add_argument('--transient-local', action='store_true',
                    help='用 TRANSIENT_LOCAL 订阅（可拿到 latched 历史消息）')
    args = ap.parse_args()

    rclpy.init(args=[])
    node = Node('topic_ready_probe')
    try:
        type_name = args.msg_type or resolve_type(node, args.topic, args.timeout)
        if type_name is None:
            print(f'{args.topic}: 超时，ROS graph 里找不到该话题', file=sys.stderr)
            return 1
        msg_class = get_message(type_name)

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if args.best_effort else ReliabilityPolicy.RELIABLE),
            durability=(DurabilityPolicy.TRANSIENT_LOCAL
                        if args.transient_local else DurabilityPolicy.VOLATILE),
        )

        received = {'n': 0}

        def cb(_msg):
            received['n'] += 1

        node.create_subscription(msg_class, args.topic, cb, qos)

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if received['n'] >= args.count:
                print(f'{args.topic}: 收到 {received["n"]} 条 {type_name}', file=sys.stderr)
                return 0

        print(f'{args.topic}: 超时 {args.timeout:.0f} s，未收到消息（{type_name}）',
              file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
