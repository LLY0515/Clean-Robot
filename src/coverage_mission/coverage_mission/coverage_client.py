"""opennav_coverage ComputeCoveragePath action 的 goal 构造与结果解析。"""

from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates

from .path_tools import segment_length

ERROR_NAMES = {
    ComputeCoveragePath.Result.NONE: 'NONE',
    ComputeCoveragePath.Result.INTERNAL_F2C_ERROR: 'INTERNAL_F2C_ERROR',
    ComputeCoveragePath.Result.INVALID_MODE_SET: 'INVALID_MODE_SET',
    ComputeCoveragePath.Result.INVALID_REQUEST: 'INVALID_REQUEST',
    ComputeCoveragePath.Result.INVALID_COORDS: 'INVALID_COORDS',
}


def _closed_ring(points):
    """保证首尾相同；F2C 要求闭合环。"""
    ring = [(float(x), float(y)) for x, y in points]
    if not ring:
        return ring
    if ring[0] != ring[-1]:
        ring.append(ring[0])
    return ring


def _to_coordinates(points):
    msg = Coordinates()
    for x, y in _closed_ring(points):
        c = Coordinate()
        c.axis1 = x
        c.axis2 = y
        msg.coordinates.append(c)
    return msg


def build_goal(zone, frame_id, start_xy=None, options=None):
    """根据区域（outer + holes）构造 ComputeCoveragePath.Goal。

    zone: {'outer': [[x,y],...], 'holes': [[[x,y],...], ...]}
    start_xy: 清扫起点 (x, y)；为 None 时由服务端按默认 TSP 起点处理
    options: 覆盖规划参数（见 coverage_params.yaml）
    """
    options = options or {}
    goal = ComputeCoveragePath.Goal()

    goal.frame_id = frame_id
    goal.use_gml_file = False

    # 分解与四阶段开关
    goal.generate_decomp = bool(options.get('generate_decomp', True))
    goal.decomp_mode.mode = str(options.get('decomp_type', 'BOUSTROPHEDON'))
    goal.decomp_mode.split_angle = float(options.get('decomp_split_angle', 0.0))
    goal.generate_headland = bool(options.get('generate_headland', True))
    goal.generate_route = bool(options.get('generate_route', True))
    goal.generate_path = bool(options.get('generate_path', True))

    # 各阶段模式（显式填写，避免依赖服务端默认值）
    goal.headland_mode.mode = str(options.get('headland_type', 'CONSTANT'))
    goal.headland_mode.width = float(options.get('headland_width', 0.8))

    goal.swath_mode.objective = str(options.get('swath_objective', 'LENGTH'))
    goal.swath_mode.mode = str(options.get('swath_mode', 'SET_ANGLE'))
    goal.swath_mode.best_angle = float(options.get('swath_angle', 0.0))
    goal.swath_mode.step_angle = float(options.get('swath_step_angle', 0.0174533))

    # 分解 + 非 TSP 会被服务端拒绝，这里固定用 TSP
    goal.route_mode.mode = str(options.get('route_mode', 'TSP'))

    goal.path_mode.mode = str(options.get('path_type', 'DUBIN'))
    goal.path_mode.continuity_mode = str(
        options.get('path_continuity', 'CONTINUOUS'))
    goal.path_mode.turn_point_distance = float(
        options.get('turn_point_distance', 0.1))

    # 多边形：第一个是外环，后续是孔洞
    goal.polygons.append(_to_coordinates(zone['outer']))
    for hole in zone.get('holes', []):
        goal.polygons.append(_to_coordinates(hole))

    if start_xy is not None:
        goal.use_start_pose = True
        goal.start_pose.axis1 = float(start_xy[0])
        goal.start_pose.axis2 = float(start_xy[1])
    else:
        goal.use_start_pose = False

    return goal


def parse_swaths(result):
    """从 action result 里取出有序 swath 列表 [(a, b), ...]（map 帧）。"""
    swaths = []
    if result is None:
        return swaths
    for swath in result.coverage_path.swaths:
        a = (float(swath.start.x), float(swath.start.y))
        b = (float(swath.end.x), float(swath.end.y))
        if segment_length(a, b) > 1e-3:
            swaths.append((a, b))
    return swaths


def error_name(code):
    return ERROR_NAMES.get(int(code), 'UNKNOWN(%d)' % int(code))
