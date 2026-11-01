import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ================= 1. 获取各个包的路径 =================
    pkg_share = get_package_share_directory('sam_bot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # ================= 2. 定义文件路径 =================
    # 你的 URDF 模型路径 (安装后位于 share/<pkg>/description/ 下)
    urdf_file = os.path.join(pkg_share, 'description', 'sam_bot_description.urdf')
    # 你的世界文件路径
    world_file = os.path.join(pkg_share, 'world', 'my_world.sdf')
    # 你刚刚保存的地图路径 (在新建的 map 文件夹下)
    map_file = os.path.join(pkg_share, 'map', 'my_map.yaml')
    # 你的 Nav2 参数文件
    param_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # 使用 Nav2 官方自带的 RViz 配置文件（它已经配置好了所有代价地图、路径等显示项）
    rviz_config_file = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    # 声明使用仿真时间参数（必须设置为 True，否则 TF 会错乱）
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ================= 3. 配置各个节点 =================

    # (1) 启动 Gazebo 服务端 (加载世界)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # (2) 启动 Gazebo 客户端 (图形界面)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # (3) 启动机器人状态发布者 (解析 URDF，发布 TF)
    #     注意：模型文件是 xacro 格式，必须用 xacro 展开成纯 URDF 再传给 robot_description，
    #     否则 <xacro:macro>/<xacro:property> 不会被展开，轮子/惯量会丢失，Gazebo 里机器人会散架。
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 【改动】必须用 ParameterValue(value_type=str) 显式声明为字符串，
            # 否则 xacro 展开的 URDF 里若含半角冒号+空格（如 mid360.xacro 的中文注释
            # “最大测距: 15m”），launch 会当 YAML 解析而报 “mapping values are not allowed here”。
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
        }]
    )

    # (4) 在 Gazebo 中生成机器人实体
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'sam_bot'],
        output='screen'
    )

    # (4.5) 启动 robot_localization EKF：URDF 里 diff_drive 的 publish_odom_tf=false，
    #       必须由 EKF 融合里程计 + IMU 发布 odom->base_link 的 TF，
    #       否则 AMCL / 代价地图拿不到 odom->base_link，导航无法定位。
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}]
    )

    # (4.6) 【改动】把 Mid-360 的 3D 点云 (/livox/lidar) 投影成 2D 激光扫描 (/scan)，
    #       供 Nav2 的 AMCL 与代价地图障碍层使用（它们只认 LaserScan）。
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',      # 先把点云变换到 base_link，再投影成 2D
            'transform_tolerance': 0.01,
            # 只保留 base_link 高度范围内的点投影成 2D。base_link 离地约 0.15 m，
            # 地面在 base_link 系下约 -0.15 m。min_height 必须高于地面，否则最下沿
            # -7.22° 波束打出的地面环(~2.1 m)会遮蔽 4.9 m 处的墙，导致 AMCL 无墙面信息而漂移。
            'min_height': -0.05,
            'max_height': 1.0,
            'angle_min': -3.141592653589793,
            'angle_max': 3.141592653589793,
            'angle_increment': 0.0174532925199433,  # 1° → 360 个点
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 15.0,                # 与 mid360.xacro 里 laser_max_range 一致
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # (5) 启动完整的 Nav2 导航栈
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': param_file
        }.items()
    )

    # (6) 启动 RViz2 并加载默认导航配置
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        # 与 display.launch.py 一致：强制 GL 2.1，避免 nav2 的 Map/Costmap 着色器
        # (sampler1D) 在 GL3+ core profile 下崩溃
        additional_env={'MESA_GL_VERSION_OVERRIDE': '2.1'},
        output='screen'
    )

    # ================= 4. 返回 LaunchDescription =================
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        ekf_node,
        pointcloud_to_laserscan,
        nav2_bringup,
        rviz2
    ])