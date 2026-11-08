"""全覆盖清扫任务执行器。

职责：
  1. 调用 opennav_coverage 的 compute_coverage_path，得到有序 swath；
  2. 机器人不在清扫起点时，先用 Nav2 点到点导航过去；
  3. 逐条 swath 用 FollowPath 精确跟踪；
  4. 前视代价地图发现当前 swath 被挡 -> 取消跟踪 -> NavigateToPose 绕到
     "同一条 swath 直线上、障碍物之后"的点 -> 继续跟踪剩余段。

状态机：
  WAIT_READY -> PLAN_SEND -> PLAN_WAIT -> TRANSIT_SEND -> TRANSIT_WAIT
             -> EXEC_SEND -> EXEC_WAIT -> (DETOUR_*) -> ... -> DONE

平滑转场：
  TRANSIT_SEND -> EXEC_WAIT（弧形换轨，优先）
  从当前车头到下一条带起点生成三次贝塞尔曲线（相邻条带间即半圆/
  半椭圆），与条带拼成一条 FollowPath，机器人直接跟踪弧线换轨。

  TRANSIT_SEND -> TRANSIT_PLAN_WAIT -> EXEC_WAIT（回退）
  距离过远或弧线贴到障碍物时，用 planner_server 的 ComputePathToPose
  规划"到条带起点"的路径，同样与条带合并成一条 FollowPath。
  两条路线都不再"先停到起点、原地转向、再起步"。
"""

import math
import os

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from .area_extractor import point_in_polygon
from .coverage_client import build_goal, error_name, parse_swaths
from .path_tools import (
    build_bezier_path,
    build_line_path,
    build_semicircle_path,
    make_pose_stamped,
    normalize_angle,
    point_at,
    project_point_on_segment,
    quaternion_to_yaw,
    segment_direction,
    segment_length,
    yaw_to_quaternion,
)
from .swath_supervisor import SwathSupervisor

from opennav_coverage_msgs.action import ComputeCoveragePath


# 状态常量（字符串便于日志阅读）
WAIT_READY = 'WAIT_READY'
START_NAV_SEND = 'START_NAV_SEND'
START_NAV_WAIT = 'START_NAV_WAIT'
PLAN_SEND = 'PLAN_SEND'
PLAN_WAIT = 'PLAN_WAIT'
TRANSIT_SEND = 'TRANSIT_SEND'
TRANSIT_PLAN_WAIT = 'TRANSIT_PLAN_WAIT'
TRANSIT_WAIT = 'TRANSIT_WAIT'
TRANSIT_TIMEOUT_WAIT = 'TRANSIT_TIMEOUT_WAIT'
EXEC_SEND = 'EXEC_SEND'
EXEC_WAIT = 'EXEC_WAIT'
DETOUR_CANCEL_WAIT = 'DETOUR_CANCEL_WAIT'
DETOUR_PLAN_WAIT = 'DETOUR_PLAN_WAIT'
DETOUR_NAV_WAIT = 'DETOUR_NAV_WAIT'
DETOUR_ABORT_WAIT = 'DETOUR_ABORT_WAIT'
RETURN_WAIT = 'RETURN_WAIT'
CANCEL_WAIT = 'CANCEL_WAIT'
DONE = 'DONE'
ABORT = 'ABORT'


class CoverageExecutor(Node):

    def __init__(self):
        super().__init__('coverage_executor')

        # ---------------- 参数 ----------------
        default_area = self._default_area_path()
        self.declare_parameter('area_file', default_area)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('transit_tolerance', 0.3)
        self.declare_parameter('coverage_goal_checker', 'coverage_goal_checker')
        self.declare_parameter('monitor_rate', 10.0)

        # 平滑转场：不再"NavigateToPose 到条带起点 -> 原地转向 -> FollowPath"，
        # 而是用 planner_server 的 ComputePathToPose 规划到起点的路径，把条带
        # 直接拼在路径末尾，用一条 FollowPath 连续跟踪。MPPI 会沿切线并入
        # 条带，省掉停顿和原地转向；转场规划失败时自动回退到 NavigateToPose。
        self.declare_parameter('smooth_transit', True)
        self.declare_parameter('plan_timeout', 10.0)

        # 弧形换轨（优先于规划器）：换轨时直接生成"当前车头 -> 下一条带
        # 起点"的三次贝塞尔曲线，短距离相邻条带之间就是半圆/半椭圆。
        # 起终点切向分别与车头、条带方向一致，跟随时不需要先原地转向。
        # 曲线只依赖条带间距/朝向，并用代价地图的致命栅格 + 车体半宽校验；
        # 不通过或距离太远才回退规划器，因此换地图同样适用。
        self.declare_parameter('arc_connector', True)
        self.declare_parameter('arc_connector_max_length', 3.0)
        # 校验用的栅格阈值/半径。Nav2 的 global_costmap 发布为 0..100，
        # 100=致命障碍、99=内接膨胀，因此 threshold=100 只看真实障碍；
        # 半径取车体半宽（0.38 m）再留少量余量，贴到障碍物才否决。
        self.declare_parameter('arc_lethal_threshold', 100)
        self.declare_parameter('arc_clearance', 0.35)
        # 换轨提前量：U 形换轨时让当前条带提前一点结束、机器人尽早开始转弯，
        # 弧顶就不会越过条带端头顶进障碍物膨胀层。提前距离默认取横向间距的
        # 一半（本图 0.8 m 间距 -> 0.4 m），上限 turn_inset_max。
        # 纵向已经错开（跨孔洞/斜向换轨）时自动不提前，避免漏扫。
        self.declare_parameter('turn_inset_ratio', 0.5)
        self.declare_parameter('turn_inset_max', 0.6)
        # 机器人离条带直线横向多远以内才做前视障碍/停滞检查。转场段离条带
        # 很远，若此时就用条带走廊判断，会把"路过柱子的转场路径"误判成
        # "当前条带被挡"，提前触发绕行。
        self.declare_parameter('on_line_tolerance', 1.0)

        # 障碍密集区跳扫：静态孔洞在规划后已裁剪；这里处理孔洞之外、
        # 代价地图上仍然密集的障碍（噪声、堆放物等）。
        #   - 剩余条带被挡比例 >= dense_swath_ratio：整段放弃，换下一条
        #     swath（"障碍物多的地方不清扫，绕过去/换轨道"）；
        #   - 前方 dense_cluster_length 米以上的连续障碍群：直接跳到障碍群
        #     之后的干净段继续扫（"换到轨道的某一点开始清扫"）。
        self.declare_parameter('dense_obstacle_skip', True)
        self.declare_parameter('dense_swath_ratio', 0.5)
        self.declare_parameter('dense_cluster_length', 2.0)
        self.declare_parameter('dense_scan_ahead', 6.0)
        # 每条 swath 两端各缩短多少米（避免车头/车尾顶墙）
        self.declare_parameter('swath_end_trim', 0.25)
        self.declare_parameter('min_swath_length', 0.4)

        # 障碍检查
        self.declare_parameter('lookahead', 3.0)
        self.declare_parameter('corridor_half_width', 0.55)
        self.declare_parameter('footprint_radius', 0.5)
        self.declare_parameter('lethal_cost_threshold', 90)
        self.declare_parameter('clear_run', 1.4)
        self.declare_parameter('rejoin_margin', 1.0)
        self.declare_parameter('max_detours_per_swath', 3)

        # 进度监控
        self.declare_parameter('stall_distance', 0.1)
        self.declare_parameter('stall_yaw_delta', 0.15)
        self.declare_parameter('stall_time', 5.0)
        # 停滞/失败点距条带终点小于该值时，不再绕行，直接结束该段。
        # 条带终点离墙约 0.3 m，MPPI 在膨胀层附近压不进 0.12 m 容差，
        # 实测会在段末反复"停滞->回轨"，每次白耗几十秒。
        self.declare_parameter('end_stall_tolerance', 1.0)

        # 超时
        self.declare_parameter('nav_timeout', 120.0)
        self.declare_parameter('follow_timeout', 180.0)
        self.declare_parameter('return_to_start', False)

        # 规划时是否忽略区域里的孔洞（柱子/结构物）。
        # 【重要】Fields2Cover 在"带孔洞的多边形"上会按子区域分别生成条带、
        # 每块锚点不同，实测出现 0.1~0.5 m 的重复行和漏扫行（覆盖率只有 81%）。
        # 去掉孔洞后条带是整场统一的 0.8 m 网格（覆盖率 94%，核心区 98%）；
        # 柱子等障碍物由执行期前视代价地图检测、绕行再回轨（已验证）。
        self.declare_parameter('plan_ignore_holes', True)

        # 执行前按孔洞裁剪 swath（配合 plan_ignore_holes 使用）。
        # 规划忽略孔洞是为了让条带网格整场统一；但执行时不能真的把车开进
        # 柱子/结构物。这里把每条 swath 在孔洞边界外 hole_clearance 处断开，
        # 剩下的连续段逐段执行（段内 FollowPath，段间 NavigateToPose 绕过去）。
        # 区域多边形已经让孔洞比真实障碍物外扩约 0.55 m，再加 0.15 m 余量，
        # 车头（前伸 0.545 m）停在断点处也不会贴到障碍物。
        self.declare_parameter('clip_swaths_at_holes', True)
        self.declare_parameter('hole_clearance', 0.15)

        # 定位跳变保护：map->base_link 单次跳变超过该值，或跑出所有分区
        # 包围盒外 pose_sanity_margin 米，就终止任务并停车。
        # 【背景】实测 swath 穿过地图上的噪声/障碍密集区时，FAST-LIO 一旦
        # 跟踪失败会发散（位姿在几秒内飘到几十米外），执行器若继续用错误
        # 位姿发指令，车会盲开。宁可停机重定位，也不能盲开。
        self.declare_parameter('pose_jump_limit', 5.0)
        self.declare_parameter('pose_sanity_margin', 5.0)

        # 覆盖规划参数（透传给 opennav_coverage）
        self.declare_parameter('generate_decomp', True)
        self.declare_parameter('decomp_type', 'BOUSTROPHEDON')
        self.declare_parameter('decomp_split_angle', 0.0)
        # 该版 Coverage Server 的 headland 阶段不生成地头 swath，开启会漏扫边界
        self.declare_parameter('generate_headland', False)
        self.declare_parameter('headland_width', 0.8)
        self.declare_parameter('swath_objective', 'LENGTH')
        self.declare_parameter('swath_mode', 'SET_ANGLE')
        self.declare_parameter('swath_angle', 0.0)
        self.declare_parameter('route_mode', 'TSP')
        self.declare_parameter('path_type', 'DUBIN')
        self.declare_parameter('path_continuity', 'CONTINUOUS')
        self.declare_parameter('turn_point_distance', 0.1)

        self._frame = self.get_parameter('frame_id').value
        self._base_frame = self.get_parameter('base_frame').value
        self._goal_checker = self.get_parameter(
            'coverage_goal_checker').value

        # ---------------- TF ----------------
        from tf2_ros import Buffer, TransformListener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---------------- action clients ----------------
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._follow_client = ActionClient(self, FollowPath, 'follow_path')
        self._planner_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')
        self._cov_client = ActionClient(
            self, ComputeCoveragePath, 'compute_coverage_path')

        # ---------------- 障碍检查 ----------------
        self._supervisor = SwathSupervisor(
            self,
            topic='global_costmap/costmap',
            lethal_threshold=self.get_parameter(
                'lethal_cost_threshold').value,
            half_width=self.get_parameter('corridor_half_width').value,
            footprint_radius=self.get_parameter('footprint_radius').value)

        # ---------------- 发布 ----------------
        self._status_pub = self.create_publisher(String, 'coverage/status', 10)
        # swath/规划路径每条只发一次，晚加入的 RViz/调试工具会错过，因此用
        # TRANSIENT_LOCAL（latched）留住最后一条。
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._swath_pub = self.create_publisher(
            Path, 'coverage/current_swath', latched)
        self._nav_path_pub = self.create_publisher(
            Path, 'coverage/planned_path', latched)
        self._blocked_pub = self.create_publisher(
            Marker, 'coverage/blocked_interval', 10)
        self._rejoin_pub = self.create_publisher(
            PoseStamped, 'coverage/rejoin_pose', 10)
        # 平滑转场时实际下发的"转场 + 条带"合并路径，便于在 RViz 里确认
        # 机器人确实是沿一条连续路径并入条带。
        self._transit_pub = self.create_publisher(
            Path, 'coverage/transit_path', latched)

        # ---------------- service ----------------
        self.create_service(Trigger, 'coverage/start', self._on_start)
        self.create_service(Trigger, 'coverage/cancel', self._on_cancel)

        # ---------------- 任务状态 ----------------
        self._state = WAIT_READY
        self._zones = []
        self._zone_idx = 0
        self._start_pose = None        # [x, y, yaw]
        self._components = []          # [(a, b), ...]
        self._comp_idx = 0
        self._swath_start_s = 0.0
        self._detour_count = 0
        self._transit_retries = 0
        self._detour_nav_retries = 0
        self._start_nav_retries = 0
        self._timeout_retry_state = TRANSIT_SEND
        self._skipped = []
        self._rejoin_s = 0.0
        # 绕行期间屏蔽前视障碍检查，直到机器人沿条带的投影越过回轨点。
        # 没有这道闸，绕行的 FollowPath 刚发出 0.5 s 就会因为"同一个障碍物
        # 仍在 s_now 前方"被再次判定为被挡 -> 重新绕行 -> 把刚发出的
        # FollowPath 取消掉；0.5 s 一轮，2 s 内耗完 max_detours_per_swath，
        # 结果是整段剩余轨迹被跳过（本来只是要绕过一个障碍物）。
        self._detour_guard_s = None

        self._active_kind = None       # 'nav' | 'follow' | 'plan' | 'coverage'
        self._pending = False
        self._done = False
        self._status = None
        self._result = None
        self._goal_handle = None
        self._goal_start = self.get_clock().now()
        # 每次发新 goal 递增；旧 goal 的回调据此丢弃，避免"取消后立刻发新
        # goal"时旧结果覆盖新 goal 的 _done/_status。
        self._action_epoch = 0
        # TRANSIT_PLAN_WAIT/DETOUR_PLAN_WAIT 共用：'transit' | 'detour'
        self._plan_purpose = 'transit'
        self._stall_fallbacks = 0

        self._last_check_time = self.get_clock().now()
        self._last_progress_pose = None
        self._last_progress_time = self.get_clock().now()

        self._last_sane_pose = None
        self._last_sane_time = self.get_clock().now()
        self._sanity_bbox = None       # (xmin, xmax, ymin, ymax)

        self._start_requested = bool(self.get_parameter('auto_start').value)
        self._cancel_requested = False

        self._load_area()

        rate = 1.0 / max(1.0, float(self.get_parameter('monitor_rate').value))
        self.create_timer(rate, self._tick)

        self.get_logger().info(
            'coverage_executor 已启动：state=%s, area_file=%s' %
            (self._state, self.get_parameter('area_file').value))

    # ================================================================== #
    # 初始化辅助
    # ================================================================== #
    @staticmethod
    def _default_area_path():
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('coverage_mission')
            return os.path.join(share, 'config', 'coverage_area.yaml')
        except Exception:
            return ''

    def _load_area(self):
        # area_file 留空时回退到包内自带的 config/coverage_area.yaml，
        # 这样 YAML 里不必写死 build 产物路径。
        path = self.get_parameter('area_file').value or self._default_area_path()
        if not path or not os.path.exists(path):
            self.get_logger().error('覆盖区域文件不存在：%s' % str(path))
            self._zones = []
            return False
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error('读取覆盖区域失败：%s' % str(exc))
            self._zones = []
            return False

        self._frame = data.get('frame_id', self._frame)
        zones = data.get('zones')
        if zones is None and 'outer' in data:
            zones = [data]
        zones = zones or []

        valid = []
        for i, zone in enumerate(zones):
            outer = zone.get('outer') or []
            if len(outer) < 3:
                self.get_logger().warn('区域 %d 外环点太少，忽略' % i)
                continue
            holes = [
                h for h in (zone.get('holes') or []) if len(h) >= 3]
            valid.append({'outer': outer, 'holes': holes})
        self._zones = valid

        start = data.get('start_pose')
        if start is not None and len(start) >= 2:
            yaw = float(start[2]) if len(start) >= 3 else 0.0
            self._start_pose = [float(start[0]), float(start[1]), yaw]
        else:
            self._start_pose = None

        # 所有分区外环的包围盒，用于定位跳变保护（见 _check_pose_sanity）
        if self._zones:
            xs = [p[0] for z in self._zones for p in z['outer']]
            ys = [p[1] for z in self._zones for p in z['outer']]
            self._sanity_bbox = (min(xs), max(xs), min(ys), max(ys))
        else:
            self._sanity_bbox = None

        self.get_logger().info(
            '已加载覆盖区域：%d 个分区，起点=%s' %
            (len(self._zones), str(self._start_pose)))
        return bool(self._zones)

    # ================================================================== #
    # service
    # ================================================================== #
    def _on_start(self, request, response):
        if self._state not in (WAIT_READY, DONE, ABORT):
            response.success = False
            response.message = '任务正在执行：%s' % self._state
            return response
        if not self._load_area():
            response.success = False
            response.message = '没有可用的覆盖区域'
            return response
        self._start_requested = True
        response.success = True
        response.message = '收到启动请求'
        return response

    def _on_cancel(self, request, response):
        if self._state in (DONE, ABORT, WAIT_READY):
            response.success = False
            response.message = '当前没有正在执行的任务'
            return response
        self._cancel_requested = True
        self._cancel_active()
        self._set_state(CANCEL_WAIT)
        response.success = True
        response.message = '正在取消任务'
        return response

    # ================================================================== #
    # action 基础设施
    # ================================================================== #
    def _send_action(self, client, goal, kind):
        self._action_epoch += 1
        epoch = self._action_epoch
        self._active_kind = kind
        self._pending = True
        self._done = False
        self._status = None
        self._result = None
        self._goal_handle = None
        self._goal_start = self.get_clock().now()
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_goal_response(f, epoch))

    def _on_goal_response(self, future, epoch):
        goal_handle = future.result()
        if epoch != self._action_epoch:
            # 已被新 goal 取代（例如规划超时后回退到 NavigateToPose）
            return
        self._pending = False
        if goal_handle is None or not goal_handle.accepted:
            self._done = True
            self._status = GoalStatus.STATUS_UNKNOWN
            self._result = None
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._on_result(f, epoch))

    def _on_result(self, future, epoch):
        wrapped = future.result()
        if epoch != self._action_epoch:
            return
        self._status = wrapped.status
        self._result = wrapped.result
        self._done = True

    def _goal_elapsed(self):
        return (self.get_clock().now() - self._goal_start).nanoseconds / 1e9

    def _cancel_active(self):
        if self._goal_handle is not None and not self._done:
            self._goal_handle.cancel_goal_async()

    def _clients_ready(self):
        return (self._nav_client.server_is_ready() and
                self._follow_client.server_is_ready() and
                self._cov_client.server_is_ready())

    # ================================================================== #
    # 主循环
    # ================================================================== #
    def _tick(self):
        try:
            self._dispatch()
        except Exception as exc:  # 防止单次异常打断整个状态机
            self.get_logger().error('执行器异常：%r' % exc)

    def _dispatch(self):
        if not self._check_pose_sanity():
            return
        if self._state == WAIT_READY:
            self._tick_wait_ready()
        elif self._state == START_NAV_SEND:
            self._tick_start_nav_send()
        elif self._state == START_NAV_WAIT:
            self._tick_start_nav_wait()
        elif self._state == PLAN_SEND:
            self._tick_plan_send()
        elif self._state == PLAN_WAIT:
            self._tick_plan_wait()
        elif self._state == TRANSIT_SEND:
            self._tick_transit_send()
        elif self._state == TRANSIT_PLAN_WAIT:
            self._tick_transit_plan_wait()
        elif self._state == TRANSIT_WAIT:
            self._tick_transit_wait()
        elif self._state == TRANSIT_TIMEOUT_WAIT:
            self._tick_transit_timeout_wait()
        elif self._state == EXEC_SEND:
            self._tick_exec_send()
        elif self._state == EXEC_WAIT:
            self._tick_exec_wait()
        elif self._state == DETOUR_CANCEL_WAIT:
            self._tick_detour_cancel_wait()
        elif self._state == DETOUR_PLAN_WAIT:
            self._tick_detour_plan_wait()
        elif self._state == DETOUR_NAV_WAIT:
            self._tick_detour_nav_wait()
        elif self._state == DETOUR_ABORT_WAIT:
            self._tick_detour_abort_wait()
        elif self._state == RETURN_WAIT:
            self._tick_return_wait()
        elif self._state == CANCEL_WAIT:
            self._tick_cancel_wait()

    def _check_pose_sanity(self):
        """定位跳变保护：位姿异常时终止任务并停车。

        【实测背景】run10 执行到第 12 条 swath 时，条带穿过地图西北角
        噪声/障碍密集区，FAST-LIO 跟踪失败后位姿在 3 s 内飘到 20 m 外
        （之后 odom 一路发散到公里级）。执行器继续用错误位姿发指令，
        车会盲开，最后转场连续失败才终止。
        这里在每次 tick 检查：
          - 相邻 tick（<=1 s）位姿跳变 > pose_jump_limit；
          - 位姿跑出所有分区包围盒外 pose_sanity_margin 米。
        触发后取消当前 goal（Nav2 会立刻停车）并终止任务；重新定位后
        再调用 /coverage/start 即可。
        """
        if self._state in (WAIT_READY, DONE, ABORT):
            return True
        pose = self._robot_pose()
        if pose is None:
            return True
        now = self.get_clock().now()
        if self._last_sane_pose is not None:
            dt = (now - self._last_sane_time).nanoseconds * 1e-9
            if 0.0 < dt <= 1.0:
                jump = math.hypot(pose[0] - self._last_sane_pose[0],
                                  pose[1] - self._last_sane_pose[1])
                limit = float(self.get_parameter('pose_jump_limit').value)
                if jump > limit:
                    self._abort(
                        '定位跳变 %.1f m（疑似定位发散），已停车' % jump)
                    return False
        # 只有进入"分区内执行"阶段才用包围盒判断；去清扫起点之前允许
        # 机器人在分区外（例如车库另一头的充电位）。
        in_zone_states = (TRANSIT_SEND, TRANSIT_PLAN_WAIT, TRANSIT_WAIT,
                          TRANSIT_TIMEOUT_WAIT, EXEC_SEND, EXEC_WAIT,
                          DETOUR_CANCEL_WAIT, DETOUR_PLAN_WAIT,
                          DETOUR_NAV_WAIT, DETOUR_ABORT_WAIT, RETURN_WAIT)
        if self._sanity_bbox is not None and self._state in in_zone_states:
            margin = float(self.get_parameter('pose_sanity_margin').value)
            x0, x1, y0, y1 = self._sanity_bbox
            if not (x0 - margin <= pose[0] <= x1 + margin and
                    y0 - margin <= pose[1] <= y1 + margin):
                self._abort(
                    '定位跑到分区范围外 (%.1f, %.1f)（疑似定位发散），已停车' %
                    (pose[0], pose[1]))
                return False
        self._last_sane_pose = pose
        self._last_sane_time = now
        return True

    # ------------------------------------------------------------------ #
    def _tick_wait_ready(self):
        if not self._start_requested:
            return
        if not self._zones:
            return
        if not self._clients_ready():
            return
        self._start_requested = False
        self._reset_mission()
        self.get_logger().info('所有 action server 就绪，开始覆盖任务')
        self._set_state(START_NAV_SEND)

    # ------------------------------------------------------------------ #
    # 需求 2 的前半段：机器人不在清扫起点时，先用现有点到点导航过去
    # ------------------------------------------------------------------ #
    def _tick_start_nav_send(self):
        if self._start_pose is None:
            self._set_state(PLAN_SEND)
            return
        pose = self._robot_pose()
        if pose is None:
            return
        dist = math.hypot(pose[0] - self._start_pose[0],
                          pose[1] - self._start_pose[1])
        if dist <= float(self.get_parameter('transit_tolerance').value):
            self.get_logger().info(
                '已在清扫起点附近（%.2f m），直接开始覆盖规划' % dist)
            self._set_state(PLAN_SEND)
            return

        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(
            self._start_pose[0], self._start_pose[1],
            self._start_pose[2], self._frame)
        self._send_action(self._nav_client, goal, 'nav')
        self.get_logger().info(
            '机器人不在清扫起点，先导航到 (%.2f, %.2f)，距离 %.2f m' %
            (self._start_pose[0], self._start_pose[1], dist))
        self._set_state(START_NAV_WAIT)

    def _tick_start_nav_wait(self):
        if self._pending:
            return
        if self._done:
            if self._status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('已到达清扫起点，开始覆盖规划')
                self._transit_retries = 0
                self._set_state(PLAN_SEND)
                return
            self._start_nav_retries += 1
            if self._start_nav_retries > 2:
                self._abort('无法到达清扫起点')
            else:
                self.get_logger().warn(
                    '前往清扫起点失败，重试 %d/2' % self._start_nav_retries)
                self._set_state(START_NAV_SEND)
            return
        if self._goal_elapsed() > float(self.get_parameter('nav_timeout').value):
            self.get_logger().warn('前往清扫起点超时，取消后重试')
            self._timeout_retry_state = START_NAV_SEND
            self._cancel_active()
            self._set_state(TRANSIT_TIMEOUT_WAIT)

    def _reset_mission(self):
        self._zone_idx = 0
        self._comp_idx = 0
        self._swath_start_s = 0.0
        self._detour_count = 0
        self._detour_guard_s = None
        self._transit_retries = 0
        self._detour_nav_retries = 0
        self._start_nav_retries = 0
        self._timeout_retry_state = TRANSIT_SEND
        self._stall_fallbacks = 0
        self._skipped = []
        self._cancel_requested = False
        self._last_sane_pose = None
        self._last_sane_time = self.get_clock().now()

    def _tick_plan_send(self):
        if self._zone_idx >= len(self._zones):
            self._finish_mission()
            return
        start_xy = None
        if self._start_pose is not None:
            start_xy = (self._start_pose[0], self._start_pose[1])
        else:
            pose = self._robot_pose()
            if pose is None:
                return
            start_xy = (pose[0], pose[1])

        zone = self._zones[self._zone_idx]
        # 规划用多边形：默认去掉孔洞，保证整场条带网格统一（见参数说明）。
        # 柱子/结构物仍留在代价地图里，由执行期绕行处理。
        plan_zone = zone
        holes = zone.get('holes', [])
        if bool(self.get_parameter('plan_ignore_holes').value) and holes:
            plan_zone = {'outer': zone['outer'], 'holes': []}
        goal = build_goal(plan_zone, self._frame, start_xy,
                          self._coverage_options())
        self._send_action(self._cov_client, goal, 'coverage')
        self.get_logger().info(
            '请求覆盖规划：分区 %d/%d，外环 %d 点，孔洞 %d 个%s' %
            (self._zone_idx + 1, len(self._zones),
             len(zone['outer']), len(holes),
             '（规划忽略孔洞，障碍物执行期绕行）' if plan_zone is not zone else ''))
        self._set_state(PLAN_WAIT)

    def _tick_plan_wait(self):
        if self._pending:
            return
        if not self._done:
            if self._goal_elapsed() > 60.0:
                self._cancel_active()
                self._abort('覆盖规划超时')
            return
        if self._status != GoalStatus.STATUS_SUCCEEDED:
            self._abort('覆盖规划 action 失败：status=%s' % self._status)
            return
        result = self._result
        if result is None or result.error_code != 0:
            code = 0 if result is None else result.error_code
            self._abort('覆盖规划返回错误 %s' % error_name(code))
            return

        raw_swaths = self._trim_swaths(parse_swaths(result))
        if not raw_swaths:
            self.get_logger().warn('规划结果没有 swath，跳过该分区')
            self._finish_zone()
            return
        swaths = self._clip_swaths(raw_swaths, self._zones[self._zone_idx])
        if not swaths:
            self.get_logger().warn('裁剪后没有可执行的 swath，跳过该分区')
            self._finish_zone()
            return

        self._components = swaths
        self._comp_idx = 0
        self._swath_start_s = 0.0
        self._detour_count = 0
        self._detour_guard_s = None
        self._transit_retries = 0
        self._publish_swath_path(swaths)
        self.get_logger().info(
            '覆盖规划完成：%d 条 swath%s，规划耗时 %.2f s' %
            (len(raw_swaths),
             '' if len(swaths) == len(raw_swaths) else
             '（按孔洞裁剪为 %d 段可执行轨迹）' % len(swaths),
             result.planning_time.sec +
             result.planning_time.nanosec * 1e-9))
        self._set_state(TRANSIT_SEND)

    # ------------------------------------------------------------------ #
    def _tick_transit_send(self):
        if self._comp_idx >= len(self._components):
            self._finish_zone()
            return
        pose = self._robot_pose()
        if pose is None:
            return
        a, b = self._components[self._comp_idx]
        dist = math.hypot(pose[0] - a[0], pose[1] - a[1])
        if dist <= float(self.get_parameter('transit_tolerance').value):
            self._set_state(EXEC_SEND)
            return

        # 障碍密集的条带不必开过去：直接换下一条（requirements: 绕过去/换轨道）
        if self._dense_precheck(a, b):
            return

        # 优先：几何弧形换轨（半圆/半椭圆），一条 FollowPath 直接跟踪
        if self._send_arc_transit(a, b, pose, dist):
            return

        # 回退：规划一条"到起点"的路径，与条带合并后一次 FollowPath
        if self._smooth_enabled():
            if self._send_plan_request(a, 'transit'):
                return
        self._send_transit_nav(a, b, dist)

    def _send_arc_transit(self, a, b, pose, dist):
        """相邻条带之间用几何弧线换轨，而不是点到点规划。

        起点 = 当前位姿（切线 = 车头方向），终点 = 下一段条带起点
        （切线 = 条带行进方向）；短距离时三次贝塞尔就是半圆/半椭圆。
        曲线先做代价地图校验，不通过/地图未就绪/距离过远则返回 False，
        由调用方回退到 ComputePathToPose 或 NavigateToPose。

        掉头（U 形换轨）时终点不贴向墙面：弧线终点取"与起点同一纵向
        坐标、横向一个车道间距"的点，做成一架对称的 U，并用真正的半圆
        （半径 = 半个车道间距）代替贝塞尔，曲率恒定、跟车更平顺。机器人
        已经提前 turn_inset 停轨（0.8 m 车道时提前 0.4 m），半圆向墙侧
        最多鼓出 0.4 m，弧顶恰好落在条带端点之内，不会侵入障碍物膨胀层；
        下一条 swath 从弧线终点对应的 s 开始跟踪，端头那一小段由弧线
        扫过的清洗带覆盖。
        """
        if not bool(self.get_parameter('arc_connector').value):
            return False
        max_len = float(self.get_parameter('arc_connector_max_length').value)
        if dist > max_len:
            return False

        ux, uy = segment_direction(a, b)
        goal_yaw = math.atan2(uy, ux)

        goal = (a[0], a[1])
        s_from = 0.0
        along = (a[0] - pose[0]) * ux + (a[1] - pose[1]) * uy
        lateral = abs(-(a[0] - pose[0]) * uy + (a[1] - pose[1]) * ux)
        max_early = float(self.get_parameter('turn_inset_max').value) + 0.3
        # 掉头时车头应基本朝 -u（半圆起点切向固定为 -u，偏差大则接成折角）
        head_err = abs(normalize_angle(
            pose[2] - math.atan2(-uy, -ux)))
        if (head_err < 0.35 and
                -max_early <= along < -1e-3 and  # 已按提前量停轨（正常 U 形）
                0.7 <= lateral <= 1.5):          # 半圆半径 = lateral/2 >= 0.35 m
            # （横向太窄时半圆半径低于 min_turning_radius，退回贝塞尔换轨）
            length = segment_length(a, b)
            if -along + 0.2 < length:
                # 终点 = 下一条带直线上、与起点纵向坐标相同的点
                goal = (a[0] - along * ux, a[1] - along * uy)
                s_from = -along

        if s_from > 0.0:
            # 对称 U：用真正的半圆（半径 = 半个车道间距），曲率恒定
            arc = build_semicircle_path(
                (pose[0], pose[1]), pose[2], goal, self._frame)
        else:
            arc = build_bezier_path(
                (pose[0], pose[1]), pose[2], goal, goal_yaw, self._frame)
        if len(arc.poses) < 3:
            return False

        clear = self._supervisor.path_clear(
            [(p.pose.position.x, p.pose.position.y) for p in arc.poses],
            threshold=self.get_parameter('arc_lethal_threshold').value,
            radius=self.get_parameter('arc_clearance').value)
        if clear is not True:
            self.get_logger().info(
                '弧形换轨校验未通过（%s），回退规划器' %
                ('地图未就绪' if clear is None else '曲线贴到障碍物'))
            return False

        arc_len = 0.0
        for p0, p1 in zip(arc.poses, arc.poses[1:]):
            arc_len += math.hypot(
                p1.pose.position.x - p0.pose.position.x,
                p1.pose.position.y - p0.pose.position.y)
        if s_from > 0.0:
            self.get_logger().info(
                '对称 U 形换轨：从 (%.2f, %.2f) 掉头进入第 %d 条 swath 的'
                ' s=%.2f m 处，弧长约 %.2f m' %
                (pose[0], pose[1], self._comp_idx + 1, s_from, arc_len))
        else:
            self.get_logger().info(
                '弧形换轨：从 (%.2f, %.2f) 到第 %d 条 swath 起点 (%.2f, %.2f)，'
                '弧长约 %.2f m' %
                (pose[0], pose[1], self._comp_idx + 1, a[0], a[1], arc_len))
        self._start_smooth_follow(arc, s_from, kind='arc')
        return True

    def _send_transit_nav(self, a=None, b=None, dist=None):
        """回退方案：NavigateToPose 到条带起点（会原地转向，但带恢复行为）。"""
        if a is None or b is None:
            a, b = self._components[self._comp_idx]
        if dist is None:
            pose = self._robot_pose()
            if pose is None:
                self._set_state(TRANSIT_SEND)
                return
            dist = math.hypot(pose[0] - a[0], pose[1] - a[1])
        if dist <= float(self.get_parameter('transit_tolerance').value):
            self._set_state(EXEC_SEND)
            return

        ux, uy = segment_direction(a, b)
        yaw = math.atan2(uy, ux)
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(a[0], a[1], yaw, self._frame)
        self._send_action(self._nav_client, goal, 'nav')
        self.get_logger().info(
            '转场到第 %d 条 swath 起点 (%.2f, %.2f)，距离 %.2f m' %
            (self._comp_idx + 1, a[0], a[1], dist))
        self._set_state(TRANSIT_WAIT)

    def _smooth_enabled(self):
        return (bool(self.get_parameter('smooth_transit').value) and
                self._planner_client.server_is_ready())

    def _send_plan_request(self, point, purpose):
        """向 planner_server 请求到 point 的接近路径。

        规划成功后由 _tick_transit_plan_wait / _tick_detour_plan_wait 把
        条带拼到路径末尾，用一条 FollowPath 连续跟踪（平滑并入）。
        """
        a, b = self._components[self._comp_idx]
        ux, uy = segment_direction(a, b)
        yaw = math.atan2(uy, ux)
        goal = ComputePathToPose.Goal()
        goal.goal = make_pose_stamped(
            point[0], point[1], yaw, self._frame)
        goal.use_start = False
        goal.planner_id = ''
        self._plan_purpose = purpose
        self._send_action(self._planner_client, goal, 'plan')
        self.get_logger().info(
            '平滑%s：规划接近路径到 (%.2f, %.2f)' %
            ('转场' if purpose == 'transit' else '回轨', point[0], point[1]))
        self._set_state(TRANSIT_PLAN_WAIT if purpose == 'transit'
                        else DETOUR_PLAN_WAIT)
        return True

    def _tick_transit_plan_wait(self):
        self._tick_approach_plan_wait()

    def _tick_detour_plan_wait(self):
        self._tick_approach_plan_wait()

    def _tick_approach_plan_wait(self):
        detour = self._plan_purpose == 'detour'
        if self._pending:
            return
        if not self._done:
            if self._goal_elapsed() > float(
                    self.get_parameter('plan_timeout').value):
                self.get_logger().warn(
                    '接近路径规划超时，回退到 NavigateToPose')
                self._cancel_active()
                self._send_nav_fallback()
            return
        if (self._status != GoalStatus.STATUS_SUCCEEDED or
                self._result is None or
                len(self._result.path.poses) == 0):
            self.get_logger().warn(
                '接近路径规划失败（status=%s），回退到 NavigateToPose' %
                self._status)
            self._send_nav_fallback()
            return
        if detour:
            self._swath_start_s = self._rejoin_s
        self._start_smooth_follow(self._result.path, self._swath_start_s)

    def _send_nav_fallback(self):
        """平滑转场/回轨规划失败时退回 NavigateToPose。"""
        if self._plan_purpose == 'detour':
            self._send_detour_nav()
        else:
            self._send_transit_nav()

    def _start_smooth_follow(self, plan_path, s_from, kind='plan'):
        """把接近路径（规划器路径或几何弧线）与条带拼成一条 FollowPath。"""
        # 只有"规划器路径 + 回轨"才是 detour；几何弧线一律按转场处理，
        # 避免上一条 swath 的 _plan_purpose 残留影响回退分支/日志。
        detour = (kind == 'plan' and self._plan_purpose == 'detour')
        a, b = self._components[self._comp_idx]
        if segment_length(a, b) - s_from < 0.1:
            # 回轨点已经到段末：没有剩余条带可跟，直接结束该段，
            # 否则会为最后几厘米再发一次 FollowPath，失败后又回轨到同一点。
            self.get_logger().info(
                '回轨点已在第 %d 条 swath 末端，直接结束该段' %
                (self._comp_idx + 1))
            self._advance_component()
            return
        path = self._combined_path(plan_path, a, b, s_from, shorten=not detour)
        if len(path.poses) < 2:
            if detour:
                self._send_detour_nav()
            else:
                self._send_transit_nav()
            return
        self._transit_pub.publish(path)
        s_end = (segment_length(a, b) if detour else self._swath_end_s(a, b))
        self._swath_pub.publish(build_line_path(
            point_at(a, b, s_from), point_at(a, b, max(s_from, s_end)),
            self._frame))
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ''
        goal.goal_checker_id = self._goal_checker
        self._send_action(self._follow_client, goal, 'follow')
        self._reset_progress_monitor()
        self._last_check_time = self.get_clock().now()
        if detour:
            self.get_logger().info(
                '已平滑并入第 %d 条 swath，从 s=%.2f m 继续' %
                (self._comp_idx + 1, s_from))
        elif kind == 'arc':
            self.get_logger().info(
                '弧形换轨：弧线与第 %d 条 swath 合并为一条 FollowPath' %
                (self._comp_idx + 1))
        else:
            self.get_logger().info(
                '平滑转场：转场与第 %d 条 swath 合并为一条 FollowPath' %
                (self._comp_idx + 1))
        self._set_state(EXEC_WAIT)

    def _combined_path(self, plan_path, a, b, s_from, shorten=True):
        path = Path()
        path.header.frame_id = self._frame
        path.header.stamp = self.get_clock().now().to_msg()
        start = point_at(a, b, s_from)
        for pose in plan_path.poses:
            if math.hypot(pose.pose.position.x - start[0],
                          pose.pose.position.y - start[1]) < 0.05:
                continue
            path.poses.append(pose)
        s_end = self._swath_end_s(a, b) if shorten else segment_length(a, b)
        path.poses.extend(build_line_path(
            start, point_at(a, b, max(s_from, s_end)), self._frame).poses)
        return path

    def _tick_transit_wait(self):
        if self._pending:
            return
        if self._done:
            if self._status == GoalStatus.STATUS_SUCCEEDED:
                self._set_state(EXEC_SEND)
                return
            self._transit_retries += 1
            if self._transit_retries > 2:
                self._abort('转场导航连续失败')
            else:
                self.get_logger().warn(
                    '转场失败，重试 %d/2' % self._transit_retries)
                self._set_state(TRANSIT_SEND)
            return
        if self._goal_elapsed() > float(self.get_parameter('nav_timeout').value):
            self.get_logger().warn('转场超时，取消后重试')
            self._timeout_retry_state = TRANSIT_SEND
            self._cancel_active()
            self._set_state(TRANSIT_TIMEOUT_WAIT)

    def _tick_transit_timeout_wait(self):
        if self._pending or not self._done:
            return
        self._transit_retries += 1
        if self._transit_retries > 2:
            self._abort('导航连续超时')
        else:
            self._set_state(self._timeout_retry_state)

    # ------------------------------------------------------------------ #
    def _swath_end_s(self, a, b):
        """当前条带跟踪到哪个参数 s 结束（默认全长）。

        如果下一条 swath 的起点就在本段末端附近，说明是弓字形 U 形换轨：
        提前 turn_inset 结束，让机器人尽早转弯。提前量按"横向间距的一半"
        估算，只有当换轨点纵向还没有错开时才生效——跨孔洞/斜向换轨的
        纵向错位本身已经给了转弯空间，不再额外漏扫。
        """
        length = segment_length(a, b)
        if self._comp_idx + 1 >= len(self._components):
            return length
        a_next, _ = self._components[self._comp_idx + 1]
        if math.hypot(a_next[0] - b[0],
                      a_next[1] - b[1]) > float(
                          self.get_parameter('arc_connector_max_length').value):
            return length
        ux, uy = segment_direction(a, b)
        dx = a_next[0] - b[0]
        dy = a_next[1] - b[1]
        along = dx * ux + dy * uy          # 换轨点在条带方向上已经错开多少
        lateral = abs(-dx * uy + dy * ux)  # 两条 swath 的横向间距
        want = min(float(self.get_parameter('turn_inset_ratio').value) * lateral,
                   float(self.get_parameter('turn_inset_max').value))
        inset = min(max(want - along, 0.0), want)
        if inset <= 1e-3:
            return length
        return max(0.1, length - inset)

    def _tick_exec_send(self):
        if self._comp_idx >= len(self._components):
            self._finish_zone()
            return

        a, b = self._components[self._comp_idx]
        s_end = self._swath_end_s(a, b)
        if s_end - self._swath_start_s < 0.1:
            self._advance_component()
            return

        start = point_at(a, b, self._swath_start_s)
        path = build_line_path(start, point_at(a, b, s_end), self._frame)
        if len(path.poses) < 2:
            self._advance_component()
            return

        self._swath_pub.publish(path)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ''
        goal.goal_checker_id = self._goal_checker
        self._send_action(self._follow_client, goal, 'follow')
        self._reset_progress_monitor()
        self._last_check_time = self.get_clock().now()
        self._set_state(EXEC_WAIT)

    def _tick_exec_wait(self):
        if self._pending:
            return
        if self._done:
            if self._status == GoalStatus.STATUS_SUCCEEDED:
                self._advance_component()
                return
            # FollowPath 被中止/取消：先尝试绕行，失败则跳过剩余段
            self.get_logger().warn(
                'FollowPath 结束 status=%s，尝试绕行' % self._status)
            if not self._try_detour():
                self._skip_remaining('FollowPath status=%s' % self._status)
            return

        if self._goal_elapsed() > float(self.get_parameter('follow_timeout').value):
            self.get_logger().warn('FollowPath 超时，尝试绕行')
            if not self._try_detour():
                self._skip_remaining('follow timeout')
            return

        # 平滑转场时 FollowPath 前半段是"去条带起点"的转场路径，机器人
        # 可能离当前条带很远。只有接近条带才做前视障碍检查，否则会把
        # 转场途中路过的柱子误判成"当前条带被挡"。
        pose = self._robot_pose()
        near_line = True
        if pose is not None:
            a, b = self._components[self._comp_idx]
            near_line = (self._point_segment_distance(
                pose[0], pose[1], a[0], a[1], b[0], b[1]) <=
                float(self.get_parameter('on_line_tolerance').value))
        if near_line:
            self._check_blocked()
        # 上一个检查可能已经触发绕行/跳过并切换了状态，避免重复动作
        if self._state == EXEC_WAIT:
            self._check_stall(near_line)

    # ------------------------------------------------------------------ #
    def _check_blocked(self):
        now = self.get_clock().now()
        if (now - self._last_check_time).nanoseconds < 5e8:  # 0.5 s
            return
        self._last_check_time = now

        pose = self._robot_pose()
        if pose is None:
            return
        a, b = self._components[self._comp_idx]
        s_now = self._supervisor.project(pose[0], pose[1], a, b)
        length = segment_length(a, b)
        if s_now >= length - 0.2:
            return

        # 正在绕行同一个障碍物：机器人沿条带的投影还没越过回轨点，
        # 前视检查必然仍看到那个障碍物。此时再判定"被挡"只会把刚发出的
        # 绕行 FollowPath 取消掉，绕行永远走不完（见 _detour_guard_s 注释）。
        if self._detour_guard_s is not None:
            if s_now < self._detour_guard_s - 0.3:
                return
            self._detour_guard_s = None

        # 障碍密集区：整段放弃（换轨道）或直接跳过障碍群（换点继续），
        # 不要一个接一个地绕行、把时间耗在密集区里。
        if self._dense_decision(a, b, s_now):
            return

        rejoin, reason = self._supervisor.find_rejoin(
            a, b, s_now,
            lookahead=float(self.get_parameter('lookahead').value),
            clear_run=float(self.get_parameter('clear_run').value),
            rejoin_margin=float(self.get_parameter('rejoin_margin').value))
        if rejoin is not None:
            self.get_logger().warn(
                '当前 swath 被挡：s=%.2f m，绕行后回轨点 s=%.2f m' %
                (s_now, rejoin))
            self._begin_detour(rejoin)
        elif reason not in ('corridor clear', 'costmap stale'):
            self.get_logger().warn(
                '当前 swath 被挡且找不到回轨点（%s），跳过剩余段' % reason)
            self._skip_remaining(reason)

    def _check_stall(self, near_line=True):
        pose = self._robot_pose()
        if pose is None:
            return
        if self._last_progress_pose is None:
            self._reset_progress_monitor(pose)
            return

        dx = pose[0] - self._last_progress_pose[0]
        dy = pose[1] - self._last_progress_pose[1]
        dyaw = abs(normalize_angle(pose[2] - self._last_progress_pose[2]))
        if (math.hypot(dx, dy) > float(self.get_parameter('stall_distance').value)
                or dyaw > float(self.get_parameter('stall_yaw_delta').value)):
            self._last_progress_pose = pose
            self._last_progress_time = self.get_clock().now()
            return

        elapsed = (self.get_clock().now() -
                   self._last_progress_time).nanoseconds / 1e9
        if elapsed < float(self.get_parameter('stall_time').value):
            return

        if not near_line:
            # 转场段卡住：FollowPath 合并路径不经过 BT 的恢复行为，
            # 这里退回 NavigateToPose 到条带起点（带 Nav2 恢复行为）。
            self._stall_fallbacks += 1
            if self._stall_fallbacks > 1:
                self.get_logger().warn(
                    '转场段连续停滞，跳过第 %d 条 swath' %
                    (self._comp_idx + 1))
                self._skip_remaining('transit stalled', use_projection=False)
                return
            self.get_logger().warn(
                '转场段停滞 %.1f s，回退到 NavigateToPose' % elapsed)
            self._cancel_active()
            self._send_transit_nav()
            return

        a, b = self._components[self._comp_idx]
        s_now = self._supervisor.project(pose[0], pose[1], a, b)
        # 距条带终点已经很近时，0.12 m 的终点容差在膨胀层附近压不进去，
        # 再绕行只会反复回轨。剩余这点直接收尾，换下一条。
        remain = segment_length(a, b) - s_now
        if remain <= float(self.get_parameter('end_stall_tolerance').value):
            self.get_logger().warn(
                '距条带终点仅 %.2f m，停滞剩余段直接结束' % remain)
            self._skip_remaining('stalled near swath end')
            return
        rejoin = self._supervisor.find_free_ahead(a, b, s_now)
        if rejoin is not None:
            self.get_logger().warn('进度停滞 %.1f s，绕行到 s=%.2f m' %
                                   (elapsed, rejoin))
            self._begin_detour(rejoin)
        else:
            self.get_logger().warn('进度停滞且前方无可行点，跳过剩余段')
            self._skip_remaining('stalled')

    def _reset_progress_monitor(self, pose=None):
        if pose is None:
            pose = self._robot_pose()
        self._last_progress_pose = pose
        self._last_progress_time = self.get_clock().now()

    # ------------------------------------------------------------------ #
    def _try_detour(self):
        pose = self._robot_pose()
        if pose is None:
            return False
        a, b = self._components[self._comp_idx]
        s_now = self._supervisor.project(pose[0], pose[1], a, b)
        remain = segment_length(a, b) - s_now
        if remain <= float(self.get_parameter('end_stall_tolerance').value):
            self.get_logger().warn(
                'FollowPath 在距条带终点 %.2f m 处失败，直接结束该段' % remain)
            self._skip_remaining('failed near swath end')
            return True
        rejoin, _ = self._supervisor.find_rejoin(
            a, b, s_now,
            lookahead=float(self.get_parameter('lookahead').value),
            clear_run=float(self.get_parameter('clear_run').value),
            rejoin_margin=float(self.get_parameter('rejoin_margin').value))
        if rejoin is None:
            rejoin = self._supervisor.find_free_ahead(a, b, s_now)
        if rejoin is None:
            return False
        self._begin_detour(rejoin)
        return True

    def _dense_precheck(self, a, b):
        """转场前整条 swath 预检：走廊被挡比例过高就直接换轨道。"""
        if not bool(self.get_parameter('dense_obstacle_skip').value):
            return False
        ratio, _ = self._supervisor.scan(a, b, 0.0)
        if ratio is None:          # 代价地图还没准备好，不做判断
            return False
        limit = float(self.get_parameter('dense_swath_ratio').value)
        if ratio < limit:
            return False
        self.get_logger().warn(
            '第 %d 条 swath 走廊被挡 %.0f%%（阈值 %.0f%%），整条跳过' %
            (self._comp_idx + 1, ratio * 100.0, limit * 100.0))
        self._skip_remaining(
            'dense swath %.0f%%' % (ratio * 100.0), use_projection=False)
        return True

    def _dense_decision(self, a, b, s_now):
        """执行期密集障碍决策：返回 True 表示已经跳过或跳点。"""
        if not bool(self.get_parameter('dense_obstacle_skip').value):
            return False
        length = segment_length(a, b)
        if length - s_now < 0.5:
            return False

        # (1) 剩余段整体被挡比例过高 -> 放弃剩余部分，换下一条 swath
        ratio, _ = self._supervisor.scan(a, b, s_now)
        if ratio is None:
            return False
        limit = float(self.get_parameter('dense_swath_ratio').value)
        if ratio >= limit:
            self.get_logger().warn(
                'swath %d 剩余 %.1f m 中走廊被挡 %.0f%%（阈值 %.0f%%），'
                '跳过剩余段' %
                (self._comp_idx + 1, length - s_now,
                 ratio * 100.0, limit * 100.0))
            self._skip_remaining(
                'dense remaining %.0f%%' % (ratio * 100.0))
            return True

        # (2) 近处有长障碍群 -> 直接跳到障碍群之后的干净段继续
        scan_to = min(
            length, s_now + float(
                self.get_parameter('dense_scan_ahead').value))
        _, spans = self._supervisor.scan(a, b, s_now, s_to=scan_to)
        if not spans:
            return False
        clear_run = float(self.get_parameter('clear_run').value)
        rejoin_margin = float(self.get_parameter('rejoin_margin').value)
        cluster_len = float(self.get_parameter('dense_cluster_length').value)
        for s0, s1 in spans:
            if s1 - s0 < cluster_len:
                continue
            next_start = length
            for t0, _ in spans:
                if t0 > s1 + 1e-6:
                    next_start = t0
                    break
            if next_start - s1 < clear_run:
                continue
            target = min(length, s1 + rejoin_margin)
            if target <= s0:
                continue
            self.get_logger().warn(
                'swath %d 前方障碍群 [%.1f, %.1f] m（%.1f m 长），'
                '直接跳到 s=%.1f m 继续清扫' %
                (self._comp_idx + 1, s0, s1, s1 - s0, target))
            self._begin_detour(target)
            return True
        return False

    def _send_detour_goal(self):
        """绕行目标：优先用平滑路径（规划 + 剩余条带一次 FollowPath）。"""
        if self._smooth_enabled():
            a, b = self._components[self._comp_idx]
            if self._send_plan_request(
                    point_at(a, b, self._rejoin_s), 'detour'):
                return
        self._send_detour_nav()

    def _begin_detour(self, rejoin_s):
        max_detours = int(self.get_parameter('max_detours_per_swath').value)
        if self._detour_count >= max_detours:
            self._skip_remaining('max detours reached')
            return
        self._detour_count += 1
        self._rejoin_s = float(rejoin_s)
        self._detour_nav_retries = 0
        # 绕行期间屏蔽前视检查，直到投影越过回轨点（取历史最大值，
        # 避免连续两次绕行把闸门往回挪）。
        self._detour_guard_s = max(self._detour_guard_s or 0.0, float(rejoin_s))

        a, b = self._components[self._comp_idx]
        self._publish_blocked(a, b, self._swath_start_s, rejoin_s)
        self._publish_rejoin(a, b, rejoin_s)

        if (self._active_kind == 'follow' and self._goal_handle is not None
                and not self._done):
            self._cancel_active()
            self._set_state(DETOUR_CANCEL_WAIT)
        else:
            self._send_detour_goal()

    def _send_detour_nav(self):
        a, b = self._components[self._comp_idx]
        point = point_at(a, b, self._rejoin_s)
        ux, uy = segment_direction(a, b)
        yaw = math.atan2(uy, ux)
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(point[0], point[1], yaw, self._frame)
        self._send_action(self._nav_client, goal, 'nav')
        self.get_logger().info(
            '绕行：导航到同一条 swath 上的回轨点 (%.2f, %.2f)' %
            (point[0], point[1]))
        self._set_state(DETOUR_NAV_WAIT)

    def _tick_detour_cancel_wait(self):
        if self._pending:
            return
        if self._done:
            self._send_detour_goal()
            return
        if self._goal_elapsed() > 5.0:
            # 极端情况下 cancel 迟迟不返回，直接发绕行 goal（预占当前 goal）
            self.get_logger().warn('取消 FollowPath 超时，直接发送绕行目标')
            self._send_detour_goal()

    def _tick_detour_nav_wait(self):
        if self._pending:
            return
        if self._done:
            if self._status == GoalStatus.STATUS_SUCCEEDED:
                self._swath_start_s = self._rejoin_s
                self._reset_progress_monitor()
                self.get_logger().info(
                    '已回到 swath，从 s=%.2f m 继续' % self._swath_start_s)
                self._set_state(EXEC_SEND)
                return
            if self._detour_nav_retries < 1:
                self._detour_nav_retries += 1
                self.get_logger().warn('绕行导航失败，重试一次')
                self._send_detour_nav()
            else:
                self._skip_remaining('detour navigation failed')
            return
        if self._goal_elapsed() > float(self.get_parameter('nav_timeout').value):
            self.get_logger().warn('绕行导航超时，取消并跳过剩余段')
            self._cancel_active()
            self._set_state(DETOUR_ABORT_WAIT)

    def _tick_detour_abort_wait(self):
        if self._pending or not self._done:
            return
        self._skip_remaining('detour timeout')

    def _tick_return_wait(self):
        if self._pending or not self._done:
            return
        self._finish_mission()

    def _tick_cancel_wait(self):
        if self._pending:
            return
        if self._done or self._goal_handle is None:
            self.get_logger().info('任务已取消')
            self._set_state(DONE)
            self._publish_status('CANCELED')
            return
        if self._goal_elapsed() > 5.0:
            self._set_state(DONE)
            self._publish_status('CANCELED')

    # ------------------------------------------------------------------ #
    def _advance_component(self):
        self._comp_idx += 1
        self._swath_start_s = 0.0
        self._detour_count = 0
        self._detour_guard_s = None
        self._transit_retries = 0
        self._stall_fallbacks = 0
        self._set_state(TRANSIT_SEND)

    def _skip_remaining(self, reason, use_projection=True):
        a, b = self._components[self._comp_idx]
        length = segment_length(a, b)
        s_from = self._swath_start_s
        if use_projection:
            pose = self._robot_pose()
            if pose is not None:
                s_now = self._supervisor.project(pose[0], pose[1], a, b)
                s_from = max(s_now, s_from)
        self._skipped.append({
            'zone': self._zone_idx,
            'swath': self._comp_idx,
            'from_s': float(s_from),
            'to_s': float(length),
            'reason': reason,
        })
        self._publish_blocked(a, b, s_from, length)
        self.get_logger().warn(
            'swath %d 剩余段 [%.1f, %.1f] m 跳过（%s）' %
            (self._comp_idx + 1, s_from, length, reason))
        self._advance_component()

    def _finish_zone(self):
        self._comp_idx = 0
        self._swath_start_s = 0.0
        self._detour_count = 0
        self._detour_guard_s = None
        if self._zone_idx + 1 < len(self._zones):
            self._zone_idx += 1
            self.get_logger().info(
                '分区完成，进入下一个分区 %d/%d' %
                (self._zone_idx + 1, len(self._zones)))
            self._set_state(PLAN_SEND)
            return
        if (bool(self.get_parameter('return_to_start').value) and
                self._start_pose is not None):
            goal = NavigateToPose.Goal()
            goal.pose = make_pose_stamped(
                self._start_pose[0], self._start_pose[1],
                self._start_pose[2], self._frame)
            self._send_action(self._nav_client, goal, 'nav')
            self._set_state(RETURN_WAIT)
            return
        self._finish_mission()

    def _finish_mission(self):
        self._set_state(DONE)
        if self._skipped:
            self.get_logger().warn(
                '任务完成，有 %d 段被跳过（可用 retry 后重新执行）' %
                len(self._skipped))
        else:
            self.get_logger().info('任务完成：全部 swath 已执行')
        self._publish_status('DONE skipped=%d' % len(self._skipped))

    def _abort(self, reason):
        self.get_logger().error('任务终止：%s' % reason)
        self._cancel_active()
        self._set_state(ABORT)
        self._publish_status('ABORT: %s' % reason)

    # ================================================================== #
    # 工具
    # ================================================================== #
    def _trim_swaths(self, swaths):
        """两端各缩短 swath_end_trim 米，并丢弃过短的碎片。

        区域多边形按"车体中心可达"内缩，但车头还向前伸出约 0.55 m；
        不缩短的话，每条 swath 的终点会让车头贴到墙上。
        """
        trim = float(self.get_parameter('swath_end_trim').value)
        min_len = float(self.get_parameter('min_swath_length').value)
        result = []
        for a, b in swaths:
            length = segment_length(a, b)
            t = min(trim, max(0.0, 0.45 * length))
            if length - 2.0 * t < min_len:
                continue
            result.append((point_at(a, b, t), point_at(a, b, length - t)))
        return result

    @staticmethod
    def _point_segment_distance(px, py, ax, ay, bx, by):
        vx, vy = bx - ax, by - ay
        wx, wy = px - ax, py - ay
        vv = vx * vx + vy * vy
        if vv <= 1e-12:
            return math.hypot(wx, wy)
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
        return math.hypot(px - (ax + t * vx), py - (ay + t * vy))

    def _blocked_by_holes(self, x, y, holes, clearance):
        """点是否落在孔洞里或距孔洞边界不足 clearance。"""
        for poly in holes:
            if point_in_polygon(x, y, poly):
                return True
            n = len(poly)
            for i in range(n):
                ax, ay = poly[i]
                bx, by = poly[(i + 1) % n]
                if self._point_segment_distance(
                        x, y, ax, ay, bx, by) < clearance:
                    return True
        return False

    def _clip_swaths(self, swaths, zone):
        """按孔洞把 swath 断成若干可执行段（plan_ignore_holes 的配套）。

        规划阶段忽略孔洞，条带才是整场统一的 0.8 m 网格；执行阶段则在
        孔洞边界外 hole_clearance 处断开，一段一段地扫。这样：
          - 柱子/结构物不再是"开到跟前再临时绕行"，不会出现回轨点落到
            障碍物内部、或车头撞上去的情况；
          - 大块噪声/障碍密集区（本图西北角）整段跳过，不会把车带进去；
          - 动态障碍物（临时箱子）仍由前视检查绕行回轨处理。
        采样步长 0.05 m，断点最多比理论边界保守 5 cm。
        """
        if not bool(self.get_parameter('clip_swaths_at_holes').value):
            return swaths
        holes = zone.get('holes') or []
        if not holes:
            return swaths
        clearance = float(self.get_parameter('hole_clearance').value)
        min_len = float(self.get_parameter('min_swath_length').value)
        step = 0.05
        clipped = []
        for a, b in swaths:
            length = segment_length(a, b)
            if length <= 0.0:
                continue
            samples = max(2, int(math.ceil(length / step)))
            runs = []
            run_start = None
            last_ok = None
            for i in range(samples + 1):
                s = length * i / samples
                x, y = point_at(a, b, s)
                blocked = self._blocked_by_holes(
                    x, y, holes, clearance)
                if blocked:
                    if run_start is not None:
                        runs.append((run_start, last_ok))
                        run_start = None
                else:
                    if run_start is None:
                        run_start = s
                    last_ok = s
            if run_start is not None:
                runs.append((run_start, last_ok))
            for s0, s1 in runs:
                if s1 - s0 < min_len:
                    continue
                clipped.append(
                    (point_at(a, b, s0), point_at(a, b, s1)))
        return clipped

    def _coverage_options(self):
        return {
            'generate_decomp': bool(self.get_parameter('generate_decomp').value),
            'decomp_type': self.get_parameter('decomp_type').value,
            'decomp_split_angle': self.get_parameter('decomp_split_angle').value,
            'generate_headland': bool(self.get_parameter('generate_headland').value),
            'headland_width': self.get_parameter('headland_width').value,
            # 只要 TSP 排好序的 swath 列表；不生成 DUBIN 连接路径
            # （连接段会被标成 HL_SWATH 混进 swaths，逐条执行用不上）
            'generate_route': True,
            'generate_path': False,
            'swath_objective': self.get_parameter('swath_objective').value,
            'swath_mode': self.get_parameter('swath_mode').value,
            'swath_angle': self.get_parameter('swath_angle').value,
            'route_mode': self.get_parameter('route_mode').value,
            'path_type': self.get_parameter('path_type').value,
            'path_continuity': self.get_parameter('path_continuity').value,
            'turn_point_distance': self.get_parameter('turn_point_distance').value,
        }

    def _robot_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._frame, self._base_frame, Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, quaternion_to_yaw(q))

    def _set_state(self, new_state):
        if new_state != self._state:
            self.get_logger().debug('state: %s -> %s' %
                                    (self._state, new_state))
        self._state = new_state
        self._publish_status(new_state)

    def _publish_status(self, text):
        msg = String()
        if self._components:
            text = '%s zone=%d/%d swath=%d/%d detours=%d skipped=%d' % (
                text, self._zone_idx + 1, len(self._zones),
                min(self._comp_idx + 1, len(self._components)),
                len(self._components), self._detour_count,
                len(self._skipped))
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_nav_path(self, path):
        if path is not None and len(path.poses) > 0:
            self._nav_path_pub.publish(path)

    def _publish_swath_path(self, swaths):
        """把全部 swath 画成一条 Path（RViz 里的 coverage/planned_path）。

        每条 swath 连续放"起点、终点"两个位姿：RViz 的 Path 显示会把相邻位姿
        连成线，于是每条 swath 是实线、条带之间的转场是斜连线，一眼就能看出
        整场的往复覆盖路径和 TSP 执行顺序。
        """
        path = Path()
        path.header.frame_id = self._frame
        path.header.stamp = self.get_clock().now().to_msg()
        for a, b in swaths:
            ux, uy = segment_direction(a, b)
            yaw = math.atan2(uy, ux)
            path.poses.append(
                make_pose_stamped(a[0], a[1], yaw, self._frame))
            path.poses.append(
                make_pose_stamped(b[0], b[1], yaw, self._frame))
        if path.poses:
            self._nav_path_pub.publish(path)

    def _publish_blocked(self, a, b, s_from, s_to):
        p0 = point_at(a, b, s_from)
        p1 = point_at(a, b, s_to)
        marker = Marker()
        marker.header.frame_id = self._frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'coverage_blocked'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0
        from geometry_msgs.msg import Point
        for p in (p0, p1):
            pt = Point()
            pt.x = p[0]
            pt.y = p[1]
            pt.z = 0.1
            marker.points.append(pt)
        self._blocked_pub.publish(marker)

    def _publish_rejoin(self, a, b, s):
        p = point_at(a, b, s)
        ux, uy = segment_direction(a, b)
        pose = make_pose_stamped(p[0], p[1], math.atan2(uy, ux), self._frame)
        pose.header.stamp = self.get_clock().now().to_msg()
        self._rejoin_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # launch 收到 Ctrl+C 时 rclpy 的 signal handler 可能已经 shutdown，
            # 再调一次会抛 "rcl_shutdown already called"，忽略即可。
            pass


if __name__ == '__main__':
    main()
