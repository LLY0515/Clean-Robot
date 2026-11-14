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
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
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
START_NAV_RETRY_WAIT = 'START_NAV_RETRY_WAIT'
# 起点头导航失败后的退避时间（秒）：Nav2 lifecycle 激活需要几秒，
# 期间 NavigateToPose 会被 action server 立刻拒绝。
START_NAV_RETRY_DELAY = 3.0
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
        # 并入角保护：接近路径末段方向与条带方向的夹角超过该值（弧度）时，
        # 不再"转场 + 条带"拼成一条 FollowPath。
        # 【实测背景】run4 第 1 段从 (-2.17,-5.00) 规划到条带东端
        # (1.53,-8.45)，接近路径末段朝东南、条带朝西，夹角 133°，拼接处是
        # 一个尖角。DWB 按最近点裁剪全局路径：车头一拐弯离开条带线，最近点
        # 就跳回"去起点"那条腿，控制器又把它开回尖角，结果在原地 2 m 范围
        # 来回画圈 190 s，最后靠 FollowPath 超时脱身，还漏扫了前 3.19 m。
        # 夹角过大时改用 NavigateToPose 到条带起点（到位后 BT 会原地转向），
        # 条带本身仍是一条直线，不存在两条腿互相抢最近点的问题。
        self.declare_parameter('merge_max_join_angle', 1.0)
        # 条带起点可站立性：条带端头离墙太近时，车体在起点原地转身会扫到
        # 障碍（车体半对角线 0.77 m）。这里要求起点周围 entry_clearance 米
        # 内没有致命栅格；不满足就把起点沿条带方向内移，最多 entry_max_shift。
        # 【实测背景】本图第 1 段起点 (1.53,-8.45) 的最近致命格只有 0.72 m，
        # 车在角上既转不开也退不出：NavigateToPose 反复 patience 超时，
        # FollowPath 的 419 条轨迹全部被 ObstacleFootprint 判碰撞。
        self.declare_parameter('entry_clearance', 0.95)
        self.declare_parameter('entry_max_shift', 2.0)
        # 整段跳过确认：单帧代价地图可能因为定位抖动、点云补全滞后或
        # 重定位瞬间的位姿误差而临时"看不见"站得下的位置，直接丢掉
        # 整条 swath 代价太大（run47 第 33 条就在 FAST-LIO 不稳定的
        # 窗口里被判成"起点 2.0 m 内都贴着障碍"而跳过）。首次判定
        # 跳过只记下时间，等 entry_confirm_delay 秒后复查仍然被挡才
        # 真正跳过。
        self.declare_parameter('entry_confirm_delay', 1.5)

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
        # 【改由控制器处理条带上的单个障碍】ego_nav2_controller 自带平滑绕行
        # （smooth_detour_enabled）：它贴着原航线走到障碍物前方约 0.7 m 才横移，
        # 一直保持横向偏移直到**车尾**越过障碍，再平滑回轨。执行器原来会取消
        # FollowPath、让规划器重新规划一条到回轨点的路径，而规划器按最短路径
        # 贴着膨胀层走，车尾还没越过障碍就开始斜切回条带——实测车尾角点离
        # 障碍只剩 0.14 m（用户反馈"车屁股摆尾容易撞到障碍物"）。
        # 打开后单个障碍交给控制器绕行；密集障碍区整段跳过、找不到回轨点
        # 直接跳过剩余段等逻辑不受影响，控制器绕不过去时仍有停滞恢复兜底。
        self.declare_parameter('controller_handles_swath_obstacles', True)
        # FollowPath 在这么短时间内就 abort，说明控制器根本没起步（位姿被
        # 判碰撞、所有采样轨迹一起作废），换回轨点没有意义，也不算一次绕行。
        self.declare_parameter('detour_min_elapsed', 1.0)

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
        #
        # hole_clearance 只解决"横向别贴着柱子"，不解决"车头别顶到柱子"：
        # 断点之后机器人停在段末时，车体中心还在断点上，而车头已经前伸
        # 0.55 m（footprint 前角 0.665 m），必然扫进孔洞。实测第 16 段就是这样
        # 断在柱子前 0.15 m，DWB 的 ObstacleFootprint 判定当前位姿已碰撞，
        # 419 条轨迹全废、控制器 patience 超时。
        # 所以孔洞两侧的断点在沿 swath 方向要再退 hole_end_clearance：
        # 车头 0.55 m + 0.05 m 余量。它只作用在被孔洞截出的端点，swath 的
        # 真实两端仍只受 swath_end_trim 约束。
        self.declare_parameter('clip_swaths_at_holes', True)
        self.declare_parameter('hole_clearance', 0.15)
        self.declare_parameter('hole_end_clearance', 0.60)

        # 定位跳变保护：map->base_link 单次跳变超过该值，或跑出所有分区
        # 包围盒外 pose_sanity_margin 米，就终止任务并停车。
        # 【背景】实测 swath 穿过地图上的噪声/障碍密集区时，FAST-LIO 一旦
        # 跟踪失败会发散（位姿在几秒内飘到几十米外），执行器若继续用错误
        # 位姿发指令，车会盲开。宁可停机重定位，也不能盲开。
        self.declare_parameter('pose_jump_limit', 5.0)
        self.declare_parameter('pose_sanity_margin', 5.0)
        # 里程计停更保护：map->base_link 的时间戳超过该秒数不再更新，说明
        # FAST-LIO / 重定位没有输出新位姿。
        # 【实测背景】run47 末尾 FAST-LIO 停更，控制器每 0.6 s 报一次
        # "Stale pose or invalid measured velocity"（patience 35 次），但执行器
        # 的跳变/越界保护看不到异常——位姿原地冻结，既不跳变也不越界。结果
        # 卡在"转场->Goal failed->重试"循环里无法自愈（每轮还要等 BT 恢复
        # 行为跑完 ~50 s）。这里显式检查位姿新鲜度：先调一次 /relocalize 争取
        # 自动恢复，宽限期内仍无新位姿就终止任务并停车，交给人工重新定位。
        self.declare_parameter('pose_stale_timeout', 5.0)
        self.declare_parameter('pose_stale_grace', 20.0)

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
        # 关键节点生命周期自愈：见 _ensure_node_active()
        self._lifecycle = {}
        for name in ('coverage_server', 'map_server'):
            self._lifecycle[name] = {
                'state_client': self.create_client(
                    GetState, name + '/get_state'),
                'change_client': self.create_client(
                    ChangeState, name + '/change_state'),
                'active': False,
                'pending': None,      # (kind, future)：在途的 get_state/change_state
                'retry_after': None,  # 转换失败后的重试时刻
            }

        # 静态地图必须真的收到才能开始任务（map_server 没激活时 static_layer
        # 是空的，全局规划器会一直报 "Robot is out of bounds"）。
        self._map_msg = None
        self.create_subscription(
            OccupancyGrid, 'map', self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

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
        self._quick_aborts = 0
        self._transit_retries = 0
        self._detour_nav_retries = 0
        # 已经做过"起点内移"检查的条带下标（每段只做一次）
        self._entry_checked = set()
        # 第 idx 条 swath 首次被判"起点贴障碍"的时刻：跳过是永久决定，
        # 必须等 entry_confirm_delay 秒复查仍然被挡才执行（见 _entry_adjust）。
        self._entry_blocked_since = {}
        self._start_nav_retries = 0
        # 【修复启动竞态】Nav2 的 action server 随节点创建就出现，但
        # lifecycle 还没 activate 时 NavigateToPose 会被立刻拒绝（<0.1 s
        # 返回失败）。原来重试之间没有间隔，0.5 s 内全部打完，任务直接
        # "无法到达清扫起点"终止（run25）。失败后先退避再重试。
        self._start_nav_retry_at = None
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
        # 最近一次成功 TF 查询的时间戳（sim time）与"开始停更"的时刻。
        self._last_tf_stamp = None
        self._stale_since = None
        self._relocalize_client = self.create_client(Trigger, 'relocalize')

        self._start_requested = bool(self.get_parameter('auto_start').value)
        self._cancel_requested = False
        self._wait_ready_warned = None

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

    def _ensure_node_active(self, name):
        """返回节点是否已 active；未 active 时按需驱动生命周期（自愈）。

        【实测背景】run49 启动时 lifecycle_manager_localization 报
        "failed to send response to /map_server/change_state (timeout)"，
        配置回调只跑了 0.1 s 就发响应仍超时（DDS 响应在负载下丢 ACK），
        管理器放弃后续 activate：map_server 停在 inactive，global_costmap
        的 static_layer 一直是空的（"Can't update static costmap layer,
        no map received"），规划器一直报 "Robot is out of bounds"，车
        纹丝不动。run44 / run48 是同一个故障打在 coverage_server 上
        （configure 初始化 Fields2Cover 时响应超时，随后
        "覆盖规划 action 失败：status=0"）。

        这里不依赖生命周期管理器的时序：执行器自己查状态，需要时代为
        configure / activate（每次 tick 推进一小步，不阻塞状态机）。
        """
        info = self._lifecycle[name]
        if info['active']:
            return True
        st_client = info['state_client']
        ch_client = info['change_client']
        if not (st_client.service_is_ready() and
                ch_client.service_is_ready()):
            return False
        now = self.get_clock().now()

        if info['pending'] is not None:
            kind, fut = info['pending']
            if not fut.done():
                return False
            info['pending'] = None
            resp = fut.result()
            if kind == 'state':
                if resp is None:
                    info['retry_after'] = now + Duration(seconds=2.0)
                    return False
                state = resp.current_state.id
                if state == State.PRIMARY_STATE_ACTIVE:
                    info['active'] = True
                    self.get_logger().info('%s 状态正常（active）' % name)
                    return True
                if state == State.PRIMARY_STATE_UNCONFIGURED:
                    transfer = Transition.TRANSITION_CONFIGURE
                    kind = 'configure'
                elif state == State.PRIMARY_STATE_INACTIVE:
                    transfer = Transition.TRANSITION_ACTIVATE
                    kind = 'activate'
                else:
                    info['retry_after'] = now + Duration(seconds=2.0)
                    self.get_logger().warn(
                        '%s 处于 state=%d，无法代为驱动生命周期'
                        % (name, state))
                    return False
                req = ChangeState.Request()
                req.transition.id = transfer
                info['pending'] = (kind, ch_client.call_async(req))
                self.get_logger().warn(
                    '%s 停在 %s（生命周期管理器可能已失败），执行器代为 %s' %
                    (name,
                     'unconfigured' if kind == 'configure' else 'inactive',
                     kind))
                return False
            # change_state 结果
            if kind == 'activate' and resp is not None and resp.success:
                info['active'] = True
                self.get_logger().info('%s 已由执行器激活' % name)
                return True
            if resp is None or not resp.success:
                info['retry_after'] = now + Duration(seconds=2.0)
                self.get_logger().warn(
                    '%s %s 转换失败，2 s 后重试' % (name, kind))
                return False
            # configure 成功：下个 tick 重新查状态，继续 activate
        if info['retry_after'] is not None and now < info['retry_after']:
            return False
        info['pending'] = ('state', st_client.call_async(GetState.Request()))
        return False

    def _on_map(self, msg):
        if self._map_msg is None:
            self.get_logger().info(
                '已收到静态地图 %d x %d（分辨率 %.2f m）' %
                (msg.info.width, msg.info.height, msg.info.resolution))
        self._map_msg = msg

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
        elif self._state == START_NAV_RETRY_WAIT:
            self._tick_start_nav_retry_wait()
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
        now = self.get_clock().now()
        if self._check_pose_stale(now):
            return False
        if pose is None:
            return True
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

    def _check_pose_stale(self, now):
        """位姿停更保护（pose_stale_timeout / pose_stale_grace）。

        返回 True 表示当前位姿已经停更、本 tick 不应继续下发任何动作。
        首次发现停更时请求一次 /relocalize 争取自动恢复；宽限期内仍没有
        新位姿就终止任务并停车（Nav2 取消 goal 即停车）。

        【实测背景】run47 末尾 FAST-LIO 停更，map->base_link 的时间戳不再
        前进。位姿原地冻结，跳变/越界检查都发现不了，执行器于是陷在
        "转场 -> Goal failed -> 重试"里，控制器只能一遍遍报 patience。
        """
        if self._state in (WAIT_READY, DONE, ABORT, CANCEL_WAIT):
            return False
        if self._last_tf_stamp is None:
            return False
        age = (now - Time.from_msg(self._last_tf_stamp)).nanoseconds * 1e-9
        limit = float(self.get_parameter('pose_stale_timeout').value)
        if limit <= 0 or age <= limit:
            if self._stale_since is not None:
                self.get_logger().info('位姿恢复更新（停更 %.1f s 后），继续任务'
                                       % age)
            self._stale_since = None
            return False
        if self._stale_since is None:
            self._stale_since = now
            self.get_logger().warn(
                '位姿已 %.1f s 未更新（FAST-LIO/重定位停更），暂停任务并请求 '
                '/relocalize' % age)
            self._request_relocalize()
            return True
        waited = (now - self._stale_since).nanoseconds * 1e-9
        grace = float(self.get_parameter('pose_stale_grace').value)
        if waited > grace:
            self._abort('定位里程计停更 %.1f s（FAST-LIO/重定位无新位姿），已停车'
                        % age)
        return True

    def _request_relocalize(self):
        client = self._relocalize_client
        if not client.service_is_ready():
            self.get_logger().warn(
                '3D 重定位服务 /relocalize 不可用，等待宽限期结束')
            return
        client.call_async(Trigger.Request())

    # ------------------------------------------------------------------ #
    def _tick_wait_ready(self):
        if not self._start_requested:
            return
        if not self._zones:
            return
        if not self._clients_ready():
            return
        # 关键节点可能因生命周期管理器超时停在 inactive（见
        # _ensure_node_active 的说明），等它们真正 active、静态地图也收到
        # 之后再开始任务，否则会带着空静态层导航、原地不动。
        missing = []
        if not self._ensure_node_active('map_server'):
            missing.append('map_server 未激活')
        if self._map_msg is None:
            missing.append('静态地图未收到')
        if not self._ensure_node_active('coverage_server'):
            missing.append('coverage_server 未激活')
        if missing:
            now = self.get_clock().now()
            if (self._wait_ready_warned is None or
                    (now - self._wait_ready_warned).nanoseconds * 1e-9 > 5.0):
                self._wait_ready_warned = now
                self.get_logger().warn(
                    '等待启动条件：%s' % '、'.join(missing))
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
            if self._start_nav_retries > 5:
                self._abort('无法到达清扫起点')
            else:
                self.get_logger().warn(
                    '前往清扫起点失败，%.0f s 后重试 %d/5' %
                    (START_NAV_RETRY_DELAY, self._start_nav_retries))
                self._start_nav_retry_at = self.get_clock().now() + Duration(
                    seconds=START_NAV_RETRY_DELAY)
                self._set_state(START_NAV_RETRY_WAIT)
            return
        if self._goal_elapsed() > float(self.get_parameter('nav_timeout').value):
            self.get_logger().warn('前往清扫起点超时，取消后重试')
            self._timeout_retry_state = START_NAV_SEND
            self._cancel_active()
            self._set_state(TRANSIT_TIMEOUT_WAIT)

    def _tick_start_nav_retry_wait(self):
        if self._start_nav_retry_at is None or (
                self.get_clock().now() < self._start_nav_retry_at):
            return
        self._start_nav_retry_at = None
        self._set_state(START_NAV_SEND)

    def _reset_mission(self):
        self._zone_idx = 0
        self._comp_idx = 0
        self._swath_start_s = 0.0
        self._detour_count = 0
        self._quick_aborts = 0
        self._detour_guard_s = None
        self._transit_retries = 0
        self._detour_nav_retries = 0
        self._start_nav_retries = 0
        self._start_nav_retry_at = None
        self._timeout_retry_state = TRANSIT_SEND
        self._stall_fallbacks = 0
        self._entry_checked = set()
        self._entry_blocked_since = {}
        self._skipped = []
        self._cancel_requested = False
        self._last_sane_pose = None
        self._last_sane_time = self.get_clock().now()
        self._stale_since = None

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
        self._entry_checked = set()
        self._detour_count = 0
        self._quick_aborts = 0
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
    def _entry_adjust(self):
        """条带起点贴障碍时，把起点沿条带方向内移（每段只做一次）。

        车体半对角线 0.77 m：条带端头离墙/柱子太近时，车在起点既转不开
        （原地转身会扫到障碍），也退不出去（min_vel_x=0 不许倒车）——
        表现为 NavigateToPose 反复 patience 超时，或者 FollowPath 的
        419 条采样轨迹全部被 ObstacleFootprint 判碰撞。

        这里要求起点周围 entry_clearance 米内没有致命栅格；不满足就沿
        条带方向每 0.1 m 内移，最多 entry_max_shift 米。被放弃的端点段
        很短（本图 0.5 m 左右），换来的是这条 swath 能真正走通。
        内移上限内仍站不住则整段跳过（对应"障碍密集处可以不清扫"），
        但要先等 entry_confirm_delay 秒复查确认（见该参数的注释）。

        返回 False 表示本 tick 不下发任何动作：要么整段已跳过，要么
        正在等待跳过确认；调用方应直接返回，状态机下个 tick 会重新进来。
        """
        idx = self._comp_idx
        if idx in self._entry_checked:
            return True
        a, b = self._components[idx]
        length = segment_length(a, b)
        radius = float(self.get_parameter('entry_clearance').value)
        max_shift = min(float(self.get_parameter('entry_max_shift').value),
                        max(length - 0.3, 0.0))
        s = 0.0
        chosen = None
        while s <= max_shift + 1e-6:
            p = point_at(a, b, s)
            clear = self._supervisor.path_clear(
                [(p[0], p[1])], threshold=100, radius=radius)
            if clear is None:
                # 代价地图还没就绪/过期：下个 tick 再判断，别急着内移
                return True
            if clear:
                chosen = s
                break
            s = round(s + 0.1, 3)
        if chosen is None:
            now = self.get_clock().now()
            since = self._entry_blocked_since.get(idx)
            if since is None:
                self._entry_blocked_since[idx] = now
                self.get_logger().warn(
                    '第 %d 条 swath 起点 %.1f m 内都贴着障碍（车体转不开），'
                    '%.1f s 后复查，仍被挡则整段跳过' %
                    (idx + 1, max_shift,
                     float(self.get_parameter('entry_confirm_delay').value)))
                return False
            waited = (now - since).nanoseconds * 1e-9
            if waited < float(self.get_parameter('entry_confirm_delay').value):
                return False
            self._entry_blocked_since.pop(idx, None)
            self._entry_checked.add(idx)
            self.get_logger().warn(
                '第 %d 条 swath 起点 %.1f m 内都贴着障碍（车体转不开），'
                '复查 %.1f s 仍被挡，整段跳过' % (idx + 1, max_shift, waited))
            self._skipped.append({
                'zone': self._zone_idx,
                'swath': idx,
                'from_s': 0.0,
                'to_s': float(length),
                'reason': 'entry blocked',
            })
            self._publish_blocked(a, b, 0.0, length)
            self._advance_component()
            return False
        self._entry_checked.add(idx)
        self._entry_blocked_since.pop(idx, None)
        if chosen <= 1e-3:
            return True
        a_new = point_at(a, b, chosen)
        self._components[idx] = (a_new, b)
        self.get_logger().warn(
            '第 %d 条 swath 起点 %.1f m 内有障碍（车体在起点转不开），'
            '起点沿条带内移 %.2f m 到 (%.2f, %.2f)，放弃端点段 [0, %.2f] m' %
            (idx + 1, radius, chosen, a_new[0], a_new[1], chosen))
        return True

    def _tick_transit_send(self):
        if self._comp_idx >= len(self._components):
            self._finish_zone()
            return
        pose = self._robot_pose()
        if pose is None:
            return
        if not self._entry_adjust():
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
        a, b = self._components[self._comp_idx]
        if self._join_too_sharp(self._result.path, a, b):
            self._send_nav_fallback()
            return
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
        plan_poses = list(plan_path.poses)
        if shorten and s_from <= 1e-6:
            # 接近路径若"沿条带线倒着"走进条带起点，拼接后会在起点处形成
            # 180° 折返（去程和回程压在同一条线上）。DWB 按最近点裁剪全局
            # 路径，会在重叠的两条腿上反复切换，车原地画圈——实测第 1 段
            # 转了 190 s，直到 FollowPath 超时才继续。这里把并入点前移到
            # 接近路径第一次落到条带线上的位置，去掉折返。
            spur_s, keep = self._plan_spur(plan_poses, a, b)
            if spur_s > 0.0:
                plan_poses = plan_poses[:keep]
                s_from = spur_s
                self.get_logger().info(
                    '接近路径沿条带线反向重叠 %.2f m，并入点前移到 '
                    's=%.2f m 以消除折返' % (spur_s, s_from))
        start = point_at(a, b, s_from)
        for pose in plan_poses:
            if math.hypot(pose.pose.position.x - start[0],
                          pose.pose.position.y - start[1]) < 0.05:
                continue
            path.poses.append(pose)
        s_end = self._swath_end_s(a, b) if shorten else segment_length(a, b)
        path.poses.extend(build_line_path(
            start, point_at(a, b, max(s_from, s_end)), self._frame).poses)
        return path

    def _plan_spur(self, poses, a, b):
        """检测接近路径末段是否沿条带线、朝条带起点方向重叠。

        返回 (s_join, keep)：s_join > 0 表示并入点应前移到条带上的
        s_join，接近路径只保留前 keep 个位姿；否则返回 (0.0, len(poses))。

        判据：路径末尾连续落在条带线上（横向偏差 <= 0.12 m），并且这段
        贴线尾部的投影 s 从 s_join 递减到 0 附近——也就是先沿 +u 方向
        走过了条带的一部分，再倒着开回起点。合并后条带又从起点沿 +u
        重新走一遍，形成 180° 折返。
        """
        tol = 0.12
        length = segment_length(a, b)
        if len(poses) < 5 or length < 1e-3:
            return 0.0, len(poses)
        s_list = []
        lat_list = []
        for p in poses:
            px = p.pose.position.x
            py = p.pose.position.y
            s, _sc, proj = project_point_on_segment(px, py, a, b)
            s_list.append(s)
            lat_list.append(math.hypot(px - proj[0], py - proj[1]))
        # 接近路径终点就是条带起点，末位姿必须已经贴在条带线上
        if lat_list[-1] > tol:
            return 0.0, len(poses)
        k = len(poses) - 1
        while k > 0 and lat_list[k - 1] <= tol:
            k -= 1
        s_join = s_list[k]
        if s_join < 0.30 or s_list[-1] > 0.15:
            return 0.0, len(poses)
        if s_join - s_list[-1] < 0.30:
            return 0.0, len(poses)
        # 并入点前移后仍要留下足够长的条带可跟踪
        if s_join > length - 0.5:
            return 0.0, len(poses)
        return s_join, k + 1

    def _join_too_sharp(self, plan_path, a, b):
        """接近路径末段方向与条带方向的夹角是否过大（合并即形成尖角）。

        夹角大时合并路径会出现"去程一条腿、条带一条腿"互相抢最近点的
        情况：车头一拐弯离开条带线，DWB 把最近点判到去程那条腿上，控制器
        又把它开回尖角，来回画圈。夹角小（从条带后方顺着并入）时没有这种
        歧义，继续沿用拼接以保证平顺。
        """
        poses = plan_path.poses
        if len(poses) < 2:
            return False
        pts = [(p.pose.position.x, p.pose.position.y) for p in poses]
        # 从终点往回取至少 0.5 m 的窗口估计末段方向（避开终点处的稀疏位姿）
        acc = 0.0
        k = len(pts) - 1
        while k > 0 and acc < 0.5:
            acc += math.hypot(pts[k][0] - pts[k - 1][0],
                              pts[k][1] - pts[k - 1][1])
            k -= 1
        hx = pts[-1][0] - pts[k][0]
        hy = pts[-1][1] - pts[k][1]
        if acc < 0.2 or math.hypot(hx, hy) < 1e-6:
            return False
        ux, uy = segment_direction(a, b)
        diff = abs(normalize_angle(
            math.atan2(hy, hx) - math.atan2(uy, ux)))
        limit = float(self.get_parameter('merge_max_join_angle').value)
        if diff <= limit:
            return False
        self.get_logger().warn(
            '接近路径末段与条带方向夹角 %.0f°（阈值 %.0f°），不拼接成一条 '
            'FollowPath（尖角处会反复打转），改用 NavigateToPose 到起点/回轨点'
            % (math.degrees(diff), math.degrees(limit)))
        return True

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
        if not self._entry_adjust():
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
            if self.get_parameter('controller_handles_swath_obstacles').value:
                return
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

        # 机器人离当前条带直线太远 => 这不是"条带被挡"，而是转场/绕行途中
        # FollowPath 被中止。此时 project() 会把投影夹紧到 0 或 L，若拿它算
        # 剩余距离，一个离条带 6 m 的机器人会被误判成"已经到条带终点"，
        # 于是每段都秒跳过（实测 77 s 跳完 21 段）。改为回退到
        # NavigateToPose 重新去条带起点（带 Nav2 恢复行为），并限制重试次数。
        lateral = self._point_segment_distance(
            pose[0], pose[1], a[0], a[1], b[0], b[1])
        if lateral > float(self.get_parameter('on_line_tolerance').value):
            self._transit_retries += 1
            if self._transit_retries > 2:
                self.get_logger().warn(
                    '转场重试 %d 次仍未回到第 %d 条 swath，跳过该段' %
                    (self._transit_retries, self._comp_idx + 1))
                self._skip_remaining('transit retries exhausted',
                                     use_projection=False)
                return True
            self.get_logger().warn(
                '机器人距第 %d 条 swath 还有 %.2f m，重新转场（%d/2）' %
                (self._comp_idx + 1, lateral, self._transit_retries))
            if not self._done:
                self._cancel_active()
            self._send_transit_nav()
            return True

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

        # 秒 abort 识别：控制器 1 s 内就把刚发出的 FollowPath 中止，说明车
        # 根本没起步（典型是局部代价地图在 footprint 内标了障碍，419 条采样
        # 轨迹全判碰撞）。这种 abort 换回轨点也救不回来，还会在 1.5 s 内把
        # max_detours 烧光、整段跳过。改为退回 NavigateToPose：BT 会在
        # FollowPath 失败时 ClearEntireCostmap（清掉误标/残留栅格），这是
        # 执行器直连 FollowPath 时拿不到的恢复能力。
        if (self._active_kind == 'follow' and self._done and
                self._goal_elapsed() < float(
                    self.get_parameter('detour_min_elapsed').value)):
            self._quick_aborts += 1
        else:
            self._quick_aborts = 0

        if self._quick_aborts >= 2:
            self._quick_aborts = 0
            if self._transit_retries < 2:
                self._transit_retries += 1
                self.get_logger().warn(
                    '控制器连续秒 abort（无法起步），改用 NavigateToPose '
                    '脱困并清理代价地图（%d/2）' % self._transit_retries)
                self._rejoin_s = float(rejoin_s)
                a, b = self._components[self._comp_idx]
                self._publish_rejoin(a, b, rejoin_s)
                if not self._done:
                    self._cancel_active()
                self._send_detour_nav()
                return
            self.get_logger().warn('脱困重试耗尽，跳过本段剩余轨迹')
            self._skip_remaining('controller cannot start')
            return

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
        self._quick_aborts = 0
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
                # 只有机器人确实在这条条带上时，投影才有意义。离线的车
                # 投影会被夹紧到 0/L，记下来的"剩余段"就是假的。
                lateral = self._point_segment_distance(
                    pose[0], pose[1], a[0], a[1], b[0], b[1])
                if lateral <= float(
                        self.get_parameter('on_line_tolerance').value):
                    s_now = self._supervisor.project(pose[0], pose[1], a, b)
                    s_from = max(s_now, s_from)
                else:
                    self.get_logger().warn(
                        '机器人离线 %.2f m，跳过记录按段起点 %.2f m 计' %
                        (lateral, s_from))
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
        self._quick_aborts = 0
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
        end_clearance = float(self.get_parameter('hole_end_clearance').value)
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
                # 被孔洞截出的端点（不在 swath 本身上）沿 swath 方向再退
                # hole_end_clearance：停在断点时车体中心离柱子还有余量，
                # 车头/车尾才不会伸进孔洞。swath 的真实两端不动。
                if s0 > 1e-6:
                    s0 = min(s0 + end_clearance, s1)
                if s1 < length - 1e-6:
                    s1 = max(s1 - end_clearance, s0)
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
        # 记下 TF 链的实际时间戳：map->base_link 是由 map->odom 与
        # odom->base_link 组合出来的，"最新时间"取两者的较小值——FAST-LIO
        # 或重定位任一侧停更，这个戳就停在原处，用它做停更检测。
        self._last_tf_stamp = tf.header.stamp
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
