#!/usr/bin/env python3
# ============================================================================
#  auto_drive_map.py —— 建 3D 先验地图时的自动行驶脚本
#
#  为什么不用键盘遥控：
#    先验地图需要覆盖尽可能多的视角，手工开容易漏、也难复现。这里用固定的
#    速度/时长跑一个小方环，保证每次生成的地图覆盖范围一致。
#
#  路径（机器人初始在原点、朝 +x）：
#    原地转 180° → 沿 -x 走 1.1 m → 转 90° → 沿 -y 走 1.1 m
#    → 转 90° → 沿 +x 走 1.1 m → 转 90° → 沿 +y 走 1.1 m 回到原点
#    最后原地转 360° 补一圈视角。
#    这个方环避开 (1.5, -0.18) 的箱子和 (-1.9, 2.4) 的球，不会撞。
#
#  话题：小车差速插件实际订阅 /cmd_vel（URDF 里 namespace=/demo 但
#        remapping cmd_vel:=/cmd_vel 覆盖成了绝对话题）。
# ============================================================================

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class AutoDrive(Node):
    def __init__(self, topic: str, rate: float):
        super().__init__('auto_drive_map')
        self.publisher_ = self.create_publisher(Twist, topic, 10)
        self.rate = rate
        self.get_logger().info(f'发布速度指令到 {topic}')

    def command(self, linear: float, angular: float, duration: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.get_logger().info(
            f'行驶 {duration:.1f}s：linear.x={linear:.2f} m/s, '
            f'angular.z={angular:.2f} rad/s')
        period = 1.0 / self.rate
        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            self.publisher_.publish(msg)
            time.sleep(period)
        self.stop()

    def stop(self) -> None:
        self.publisher_.publish(Twist())
        time.sleep(0.3)

    def run_square(self) -> None:
        # 先转 180°，让“前进”朝 -x，远离箱子。
        self.command(0.0, 0.6, 5.24)
        for _ in range(4):
            self.command(0.25, 0.0, 4.4)   # ~1.1 m
            self.command(0.0, 0.7, 2.24)   # ~90°
        # 回到原点后再完整转一圈，补足各方向墙面。
        self.command(0.0, 0.6, 10.5)


def main() -> None:
    parser = argparse.ArgumentParser(description='自动行驶以采集 3D 先验地图')
    parser.add_argument('--topic', default='/cmd_vel', help='速度指令话题')
    parser.add_argument('--rate', type=float, default=10.0, help='发布频率 Hz')
    args = parser.parse_args()

    rclpy.init()
    node = AutoDrive(args.topic, args.rate)
    try:
        node.run_square()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
