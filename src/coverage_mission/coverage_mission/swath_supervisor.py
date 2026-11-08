"""当前 swath 的前视障碍检查与"回轨点"计算。

思路：
  1. 把当前 swath 参数化为 P(s) = a + s·u，s 从机器人投影点开始；
  2. 沿 swath 前视 lookahead 米，检查以 swath 为中心线、半宽 half_width 的走廊；
  3. 走廊里出现代价 >= lethal_threshold 的格子 => 该 swath 被挡；
  4. 从障碍物之后继续找一段连续 clear_run 米的干净走廊，
     回轨点取在这段干净走廊起点再往后 rejoin_margin 米 ——
     回轨点严格落在原 swath 直线上，做到"回到同一条轨道"而不是回到原点。
"""

import math

import numpy as np

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from .path_tools import point_at, project_point_on_segment, segment_direction, segment_length


class SwathSupervisor:
    def __init__(self, node, topic='global_costmap/costmap',
                 lethal_threshold=90, half_width=0.55,
                 footprint_radius=0.5):
        self._node = node
        self._threshold = int(lethal_threshold)
        self._half_width = float(half_width)
        self._footprint_radius = float(footprint_radius)

        self._info = None
        self._grid = None
        self._stamp = None

        # 走廊横向采样偏移（-half_width ... +half_width）
        self._offsets = np.arange(
            -self._half_width, self._half_width + 1e-9, 0.05)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._sub = node.create_subscription(
            OccupancyGrid, topic, self._callback, qos)

    # ------------------------------------------------------------------ #
    # 代价地图缓存
    # ------------------------------------------------------------------ #
    def _callback(self, msg):
        self._info = msg.info
        self._stamp = msg.header.stamp
        try:
            self._grid = np.asarray(msg.data, dtype=np.int16).reshape(
                (msg.info.height, msg.info.width))
        except ValueError:
            # data 长度和宽高不一致（极少见），丢弃这一帧
            self._grid = None

    def has_map(self):
        return self._grid is not None and self._info is not None

    def is_fresh(self, max_age=5.0):
        if not self.has_map():
            return False
        if self._stamp is None:
            return True
        now = self._node.get_clock().now()
        stamp = Time.from_msg(self._stamp)
        return abs((now - stamp).nanoseconds) < int(max_age * 1e9)

    def _cell_value(self, x, y):
        info = self._info
        if info is None or self._grid is None:
            return None
        res = info.resolution
        ix = int(math.floor((x - info.origin.position.x) / res))
        iy = int(math.floor((y - info.origin.position.y) / res))
        if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
            return None
        return int(self._grid[iy, ix])

    def _blocked_at(self, x, y):
        value = self._cell_value(x, y)
        # 未知区域(-1)和地图外不当作障碍；真正的障碍需要 >= threshold
        return value is not None and value >= self._threshold

    # ------------------------------------------------------------------ #
    # 走廊检查
    # ------------------------------------------------------------------ #
    def corridor_blocked_at(self, a, u, n, s):
        """swath 参数 s 处、横向 ±half_width 的走廊是否被挡。"""
        px = a[0] + u[0] * s
        py = a[1] + u[1] * s
        for off in self._offsets:
            if self._blocked_at(px + n[0] * off, py + n[1] * off):
                return True
        return False

    def _corridor_clear(self, a, u, n, s_from, s_to, step=0.1):
        s = s_from
        while s <= s_to + 1e-9:
            if self.corridor_blocked_at(a, u, n, s):
                return False
            s += step
        return True

    def _disk_clear(self, x, y, radius, threshold):
        """以 (x, y) 为圆心、radius 为半径的圆内是否有 >= threshold 的栅格。

        直接遍历覆盖圆的全部栅格（按栅格中心到圆心的距离裁剪），而不是
        在圆内按 0.1 m 步长采样——后者会漏掉恰好落在采样点之间的 5 cm
        细障碍，弧形换轨的校验不能有这种缝隙。
        """
        info = self._info
        if info is None or self._grid is None:
            return True
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        ix0 = int(math.floor((x - radius - ox) / res))
        ix1 = int(math.floor((x + radius - ox) / res))
        iy0 = int(math.floor((y - radius - oy) / res))
        iy1 = int(math.floor((y + radius - oy) / res))
        ix0 = max(ix0, 0)
        iy0 = max(iy0, 0)
        ix1 = min(ix1, info.width - 1)
        iy1 = min(iy1, info.height - 1)
        r2 = radius * radius
        for iy in range(iy0, iy1 + 1):
            cy = oy + (iy + 0.5) * res
            dy = cy - y
            if dy * dy > r2:
                continue
            for ix in range(ix0, ix1 + 1):
                cx = ox + (ix + 0.5) * res
                dx = cx - x
                if dx * dx + dy * dy > r2:
                    continue
                if int(self._grid[iy, ix]) >= threshold:
                    return False
        return True

    def _footprint_free(self, x, y, step=0.1):
        # step 参数保留只为兼容旧调用；判定已改为遍历圆内全部栅格。
        return self._disk_clear(x, y, self._footprint_radius, self._threshold)

    def path_clear(self, points, threshold=None, radius=None):
        """任意折线/曲线的几何可通过性校验（用于弧形换轨）。

        与条带走廊检查不同，这里默认只看"致命"栅格（threshold=100，
        Nav2 把 254 映射为 100、膨胀层映射为 0~99），因为弧形换轨本来就
        要在条带端头的余量里转向；用膨胀层判断会把所有端头掉头都否掉。
        radius 用于近似车体半宽，只有真正贴着障碍物时才拒绝。

        代价地图为空/过期时返回 None，调用方据此回退到规划器。
        """
        if not self.is_fresh():
            return None
        thr = self._threshold if threshold is None else int(threshold)
        rad = self._footprint_radius if radius is None else float(radius)
        for x, y in points:
            if not self._disk_clear(x, y, rad, thr):
                return False
        return True

    # ------------------------------------------------------------------ #
    # 回轨点
    # ------------------------------------------------------------------ #
    def find_rejoin(self, a, b, s_now, lookahead=3.0, clear_run=1.4,
                    rejoin_margin=1.0, step=0.1):
        """找到"障碍物之后、仍在这条 swath 直线上"的回轨点。

        返回 (rejoin_s, None) 或 (None, 原因字符串)。
        rejoin_s 是沿 a->b 的参数（米），保证回轨点落在原直线上。
        """
        if not self.is_fresh():
            return None, 'costmap stale'

        length = segment_length(a, b)
        if length < 1e-6:
            return None, 'zero-length swath'
        u = segment_direction(a, b)
        n = (-u[1], u[0])

        # (1) 前视窗口内找第一个被挡的位置
        scan_end = min(length, s_now + lookahead)
        s_a = None
        s = max(0.0, s_now + 0.2)
        while s <= scan_end:
            if self.corridor_blocked_at(a, u, n, s):
                s_a = s
                break
            s += step
        if s_a is None:
            return None, 'corridor clear'

        # (2) 障碍之后找一段连续干净走廊，回轨点在其中再往后 rejoin_margin
        s = s_a
        while s <= length:
            window_end = min(length, s + clear_run)
            if self._corridor_clear(a, u, n, s, window_end):
                rejoin_s = min(length, s + rejoin_margin)
                # 障碍物太靠 swath 末端时，回轨点离障碍太近，放弃
                if rejoin_s - s < min(rejoin_margin, 0.5) and rejoin_s < length:
                    s += step
                    continue
                px, py = point_at(a, b, rejoin_s)
                if self._footprint_free(px, py):
                    return rejoin_s, None
            s += step

        return None, 'no clear run before swath end'

    def find_free_ahead(self, a, b, s_now, search=2.5, clear_run=1.0, step=0.25):
        """进度停滞兜底：在前方 search 米内找一个可通行的点作为绕行目标。

        只要求该点 footprint 可放且该点之后 clear_run 米走廊干净，
        不要求检测到明确的障碍物区间。
        """
        if not self.is_fresh():
            return None
        length = segment_length(a, b)
        u = segment_direction(a, b)
        n = (-u[1], u[0])

        s = min(length, s_now + 1.0)
        while s <= min(length, s_now + search):
            px, py = point_at(a, b, s)
            window_end = min(length, s + clear_run)
            if (self._footprint_free(px, py) and
                    self._corridor_clear(a, u, n, s, window_end)):
                return s
            s += step
        return None

    def scan(self, a, b, s_from=0.0, s_to=None, step=0.1):
        """沿 swath 采样走廊，返回 (被挡比例, 连续被挡区间列表)。

        用于"障碍密集区直接跳扫"：
          - 被挡比例高 => 这条 swath 不值得扫，整段跳过；
          - 连续被挡区间很长 => 是障碍群，直接跳到它之后继续。
        区间端点取采样点，误差不超过一个采样步长（默认 0.1 m）。
        代价地图不新鲜时返回 (None, [])。
        """
        if not self.is_fresh():
            return None, []
        length = segment_length(a, b)
        if length < 1e-6:
            return 0.0, []
        u = segment_direction(a, b)
        n = (-u[1], u[0])
        s_from = min(max(float(s_from), 0.0), length)
        s_to = length if s_to is None else min(max(float(s_to), s_from), length)
        if s_to - s_from < 1e-9:
            return 0.0, []

        count = max(2, int(math.ceil((s_to - s_from) / step)) + 1)
        spans = []
        span_start = None
        blocked = 0
        prev_s = s_from
        for i in range(count):
            s = s_from + (s_to - s_from) * i / float(count - 1)
            if self.corridor_blocked_at(a, u, n, s):
                blocked += 1
                if span_start is None:
                    span_start = prev_s
            elif span_start is not None:
                spans.append((span_start, prev_s))
                span_start = None
            prev_s = s
        if span_start is not None:
            spans.append((span_start, prev_s))
        return blocked / float(count), spans

    def project(self, px, py, a, b):
        """机器人投影到 swath 上的参数 s（夹到 [0, L]）。"""
        _, clamped, _ = project_point_on_segment(px, py, a, b)
        return clamped
