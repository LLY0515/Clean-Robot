"""RTAB-Map 2D 建图（激光 + FAST-LIO 里程计）。

用途：替代 slam_toolbox 在线建图，输出 nav_msgs/OccupancyGrid
     （用 sam_bot_description/tools/save_map.py 保存成 pgm+yaml）。

输入：
  /scan      pointcloud_to_laserscan 投影出的 2D 激光（display.launch.py 已提供）
  /Odometry  FAST-LIO 里程计，同时也有 odom->base_footprint 的 TF

输出：
  /map       OccupancyGrid
  map->odom  TF（由本节点发布，替代 slam_toolbox）

典型用法（build_map.sh 会自动带上这些参数）：
  ros2 launch sam_bot_description rtabmap_mapping.launch.py \
    database_path:=/tmp/rtabmap_my_map.db
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    use_rtabmapviz = LaunchConfiguration('rtabmapviz')

    # ---- RTAB-Map 核心参数 ----
    # 注意：RTAB-Map 的 "核心参数"（带 / 的键）在 ROS 2 里一律以字符串传入，
    # 由节点内部转成对应的 bool/int/float，写成 Python bool 会类型不匹配。
    # 节点自身声明的参数（subscribe_* / frame_id / publish_tf ...）必须用真实
    # 的 Python 类型；只有 RTAB-Map 的"核心参数"（键里带 / 的那些，如 Reg/Force3DoF）
    # 才由节点内部从字符串解析。
    params = {
        # 坐标系
        'frame_id': 'base_footprint',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'publish_tf': True,              # 发布 map->odom
        'approx_sync': True,             # /scan 与 /Odometry 时间戳不必严格相等
        'queue_size': 30,
        'wait_for_transform': 0.2,

        # 输入：只用 2D 激光 + 外部里程计，不跑 rtabmap 自带的视觉/ICP 里程计
        'subscribe_scan': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_odom_info': False,

        # 平面机器人：强制 6DoF 退化为 3DoF（x, y, yaw），避免俯仰/横滚漂移
        'Reg/Force3DoF': 'true',
        # 激光点云配准用 ICP（RTAB-Map 对 2D 激光的推荐配置）
        'Reg/Strategy': '1',
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/PointToPlane': 'false',
        # 纯 2D 时关掉重力约束
        # 回环：纯激光模式 BoW（视觉词袋）被禁用，只能靠"空间邻近"触发，
        # 该路径需要 Reg/Strategy=1（ICP）。
        #
        # NeighborLinkRefining 默认 True 时，RTAB-Map 会用视觉特征去精修相邻
        # 节点之间的位姿；纯激光下没有图像，每帧都报
        #   Memory.cpp: computeTransform() Missing visual features ... Transform
        #   cannot be estimated.
        # 而且它会把 ICP 已经对好的位姿再拉一遍，反而制造重影。
        # 同一段 4 趟往返路线（8 m，4 次 180° 原地转）实测：
        #   True  → 占用格到真墙 p90 0.30 m，离墙>0.3 m 的杂点 11.4%
        #   False → 同上 0.20 m / 1.6%（slam_toolbox 是 0.25 m / 9.7%）
        # 所以这里必须关掉。
        'RGBD/ProximityBySpace': 'true',
        'RGBD/NeighborLinkRefining': 'False',
        'Optimizer/GravitySigma': '0',

        # 栅格地图：用激光、2D、不按深度
        'Grid/Sensor': '0',              # 0 = 只用激光
        'Grid/FromDepth': 'false',
        'Grid/3D': 'false',
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '15.0',
        'Grid/RayTracing': 'true',
        # 忽略贴着机器人本体的回波（官方示例同样设为 0.2）
        'Grid/RangeMin': '0.2',

        # 建图节奏：走 0.2 m 或转 0.15 rad 加一个节点
        'RGBD/LinearUpdate': '0.2',
        'RGBD/AngularUpdate': '0.15',

        # 长期建图时控制内存/数据库
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',

        'database_path': database_path,
        'use_sim_time': use_sim_time,
    }

    # rtabmap 0.23（Humble）里建图节点在 rtabmap_slam 包里；
    # rtabmap_ros 只是元包，没有可执行文件。
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[params],
        remappings=[
            ('scan', scan_topic),
            ('odom', odom_topic),
        ],
    )

    rtabmapviz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        condition=IfCondition(use_rtabmapviz),
        parameters=[params],
        remappings=[
            ('scan', scan_topic),
            ('odom', odom_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='使用 Gazebo 的仿真时钟'),
        DeclareLaunchArgument(
            'database_path', default_value='/tmp/rtabmap_default.db',
            description='RTAB-Map 数据库路径；建图前应删除以从零开始'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan', description='2D 激光话题'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/Odometry', description='FAST-LIO 里程计话题'),
        DeclareLaunchArgument(
            'rtabmapviz', default_value='False', description='是否打开 RTAB-Map 可视化'),
        rtabmap_node,
        rtabmapviz_node,
    ])
