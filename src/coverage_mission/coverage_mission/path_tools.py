"""几何与消息构造工具：swath 投影、直线路径生成、位姿转换。"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def yaw_to_quaternion(yaw):
    """欧拉角 yaw -> geometry_msgs/Quaternion。"""
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q):
    """geometry_msgs/Quaternion -> yaw（只看 z 轴旋转）。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_pose_stamped(x, y, yaw, frame_id, stamp=None):
    """构造 PoseStamped。stamp 为 None 时留空，由下游按当前时间解释。"""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp is not None:
        pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_to_quaternion(float(yaw))
    return pose


def segment_length(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def segment_direction(a, b):
    """单位方向向量；零长度时返回 (1, 0)。"""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return 1.0, 0.0
    return dx / n, dy / n


def point_at(a, b, s):
    """swath 上参数 s（米）处的点，s 会夹到 [0, L]。"""
    length = segment_length(a, b)
    s = min(max(s, 0.0), length)
    ux, uy = segment_direction(a, b)
    return a[0] + ux * s, a[1] + uy * s


def project_point_on_segment(px, py, a, b):
    """把点投影到线段 ab，返回 (s, 夹紧后的 s, 最近点)。

    s 为从 a 沿 ab 方向的距离；clamped_s 夹到 [0, L]，最近点按 clamped_s 计算。
    """
    length = segment_length(a, b)
    if length < 1e-9:
        return 0.0, 0.0, (a[0], a[1])
    ux, uy = segment_direction(a, b)
    s = (px - a[0]) * ux + (py - a[1]) * uy
    clamped = min(max(s, 0.0), length)
    return s, clamped, (a[0] + ux * clamped, a[1] + uy * clamped)


def build_line_path(a, b, frame_id, step=0.1, stamp=None):
    """把一条线段展开成密集的 nav_msgs/Path（朝向沿线段）。

    控制器（MPPI）对只有两点的直线也能跟，但加密到 0.1 m 可以：
      - 让 FollowPath 的速度/朝向更平稳；
      - 让 RViz 里看到的"当前轨道"更清楚。
    """
    length = segment_length(a, b)
    if length < 1e-6:
        return Path()
    ux, uy = segment_direction(a, b)
    yaw = math.atan2(uy, ux)
    n = max(2, int(math.ceil(length / max(step, 1e-3))) + 1)

    path = Path()
    path.header.frame_id = frame_id
    if stamp is not None:
        path.header.stamp = stamp
    for i in range(n):
        t = i / float(n - 1)
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        path.poses.append(make_pose_stamped(x, y, yaw, frame_id, stamp))
    return path


def build_bezier_path(start, start_yaw, goal, goal_yaw, frame_id,
                      step=0.05, stamp=None):
    """三次贝塞尔换轨曲线：起终点切向分别沿 start_yaw / goal_yaw。

    弓字形相邻条带的换轨（半圆/半椭圆）由这里生成：
      - P0 = 当前车位姿；
      - P1 = P0 沿当前车头方向前进 dist/3；
      - P2 = 目标点沿"进入目标条带的方向"后退 dist/3；
      - P3 = 目标条带起点。
    起点切线 = 当前车头，终点切线 = 新条带方向，因此跟随时不需要
    "先原地转向再进入条带"。曲线是否可通行由调用方用代价地图校验，
    不通则回退到规划器，不依赖具体地图。
    """
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return Path()

    k = dist / 3.0
    p0 = (float(start[0]), float(start[1]))
    p1 = (p0[0] + k * math.cos(start_yaw),
          p0[1] + k * math.sin(start_yaw))
    p2 = (goal[0] - k * math.cos(goal_yaw),
          goal[1] - k * math.sin(goal_yaw))
    p3 = (float(goal[0]), float(goal[1]))

    n = max(10, int(math.ceil(dist / max(step, 1e-3))) + 1)
    path = Path()
    path.header.frame_id = frame_id
    if stamp is not None:
        path.header.stamp = stamp
    for i in range(n):
        t = i / float(n - 1)
        u = 1.0 - t
        w0, w1, w2, w3 = u * u * u, 3 * u * u * t, 3 * u * t * t, t * t * t
        x = w0 * p0[0] + w1 * p1[0] + w2 * p2[0] + w3 * p3[0]
        y = w0 * p0[1] + w1 * p1[1] + w2 * p2[1] + w3 * p3[1]
        tx = (3 * u * u * (p1[0] - p0[0]) +
              6 * u * t * (p2[0] - p1[0]) +
              3 * t * t * (p3[0] - p2[0]))
        ty = (3 * u * u * (p1[1] - p0[1]) +
              6 * u * t * (p2[1] - p1[1]) +
              3 * t * t * (p3[1] - p2[1]))
        if math.hypot(tx, ty) < 1e-6:
            # 控制点重合的退化情况：用弦方向
            tx, ty = dx, dy
        path.poses.append(
            make_pose_stamped(x, y, math.atan2(ty, tx), frame_id, stamp))
    return path


def build_semicircle_path(start, start_yaw, goal, frame_id, step=0.05,
                          stamp=None):
    """以 start->goal 为直径的半圆（对称 U 形换轨专用）。

    与贝塞尔不同，整条曲线曲率恒定，半径 = 直径/2，跟车更平顺；
    起点切向自动取最接近 start_yaw 的顺/逆时针方向，终点切向与起点
    相反（即下一条 swath 的行进方向）。直径按横向车道间距给定时，
    半圆只向换轨侧鼓出半个车道宽，不会顶到条带端点之外。
    """
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return Path()
    r = d / 2.0
    cx = start[0] + dx / 2.0
    cy = start[1] + dy / 2.0
    a0 = math.atan2(start[1] - cy, start[0] - cx)
    # 逆时针切向 = 半径方向 +90°；顺时针切向 = -90°
    ccw = abs(normalize_angle(a0 + math.pi / 2.0 - start_yaw))
    cw = abs(normalize_angle(a0 - math.pi / 2.0 - start_yaw))
    sweep = math.pi if ccw <= cw else -math.pi

    n = max(10, int(math.ceil(abs(sweep) * r / max(step, 1e-3))) + 1)
    path = Path()
    path.header.frame_id = frame_id
    if stamp is not None:
        path.header.stamp = stamp
    for i in range(n):
        ang = a0 + sweep * i / float(n - 1)
        x = cx + r * math.cos(ang)
        y = cy + r * math.sin(ang)
        tangent = ang + (math.pi / 2.0 if sweep > 0.0 else -math.pi / 2.0)
        path.poses.append(
            make_pose_stamped(x, y, tangent, frame_id, stamp))
    return path


def remaining_segment(a, b, s_from):
    """返回从参数 s_from 到终点的剩余线段端点。"""
    start = point_at(a, b, s_from)
    return start, (b[0], b[1])


def normalize_angle(angle):
    """把角度归一到 (-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle
