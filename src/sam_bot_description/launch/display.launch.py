import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    pkg_fast_lio_share = launch_ros.substitutions.FindPackageShare(package='fast_lio').find('fast_lio')
    fastlio_config = os.path.join(pkg_fast_lio_share, 'config', 'mid360_sim.yaml')
    default_model_path = os.path.join(pkg_share, 'description/sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    # 【改动】换成停车场场景 world/my_carpark（约 20×20 m：四周 2.5 m 围墙 +
    # 内部隔墙/隔离墩 + 2 辆静止车辆 suv / hatchback_red）。
    # 该文件是从 Gazebo 直接保存的完整 world，文件名没有 .sdf 后缀，gzserver
    # 按内容解析 SDF，实测可正常加载。
    # 其他场景保留在 world/ 下，可随时改回：
    #   my_newworld.sdf（建模导出的楼宇）、my_world.sdf（10×10 方围墙 + 箱子/球）。
    world_path = os.path.join(pkg_share, 'world/my_carpark')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # 【改动】ParameterValue(value_type=str) 防止 xacro 展开结果被当 YAML 解析（同 navigation.launch.py）
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz')),
        # 强制 GL 2.1 让 Ogre 退回旧版 RenderSystem_GL：rviz 的 Map/Costmap 着色器
        # 用了 sampler1D（1D 纹理），而 GL3+ core profile 已移除 1D 纹理，会导致
        # "active samplers with a different type refer to the same texture image unit" 崩溃。
        additional_env={'MESA_GL_VERSION_OVERRIDE': '2.1'},
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # 【改动】-timeout 放宽到 120s，避免 gzserver 启动慢时 spawn 服务还没就绪就退出。
        # 【改动】-z 0.24：新 CAD 模型轮底在 base_link 系 z=-0.24。
        # 这里与 navigation.launch.py 保持一致：-z 0.25 会先下落 1cm 再弹跳，
        # 正好落在 FAST-LIO 的 IMU 静止初始化窗口里，污染重力估计。建 3D 先验
        # 地图也用这个高度，才能和导航时的 odom 系 z 原点对齐。
        # 【改动】-x/-y 2.0 2.7：停车场场景里这里离最近隔墙 3.35 m（按 my_carpark
        # 的 38 个 box 碰撞体算的），车体约 0.7×0.5 m，出生空间宽裕。
        # 必须与 navigation.launch.py 一致，建图和导航的 map 原点才对得上。
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description',
                   '-timeout', '120', '-x', '2.0', '-y', '2.7', '-z', '0.24'],
        output='screen'
    )

    # FAST-LIO：LiDAR+IMU 紧耦合里程计（替代原 robot_localization EKF），发布 odom->base_link TF
    fastlio_node = launch_ros.actions.Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        # 【改动】参数文件与 map_file_path 都可从命令行覆盖：
        #   ros2 launch ... display.launch.py \
        #     fastlio_config:=.../mid360_sim_mapping.yaml \
        #     fastlio_map_file:=/abs/path/prior_3d_map.pcd
        # 建 3D 先验地图时用 mapping 配置（pcd_save_en:=true），
        # 再由 build_3d_map.sh 调用 /map_save 服务把累积点云写成 PCD。
        parameters=[LaunchConfiguration('fastlio_config'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'map_file_path': LaunchConfiguration('fastlio_map_file')}]
    )

    # 【改动】把 Mid-360 的 3D 点云 (/livox/lidar) 投影成 2D 激光扫描 (/scan)。
    # display.launch.py 用于建图流程，slam_toolbox 只订阅 /scan(LaserScan)，
    # 所以这里也必须起一个 pointcloud_to_laserscan 节点来产出 /scan。
    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            # 地面在 base_link 系下约 -0.24 m，min_height 必须高于地面，否则会把
            # 地面环投影成假墙。
            'min_height': -0.05,
            # 【改动】建图用薄片放宽到 2.50 m（墙体高 2.5 m）。
            # 仿真雷达每帧只有 2500 条射线，落在这个薄片里的点本来就不多，
            # 放宽能显著提高 /scan 的有效光束数。
            # 雷达 30° 前倾时实测每帧：约 62% 的点在 15 m 内就打在地面上，
            # 落在本薄片内只有约 38%，/scan 360 束里有回波 153~166 束
            # （水平安装时可到 258~276 束）。倾斜会让有效光束变少、2D 建图变差，
            # 但这是当前要求的物理安装角。
            # 原先担心 z≈0.845 的雷达底座板被投影成假障碍（navigation.launch.py
            # 的老注释），实测连采 60 帧、任何角度都没有出现 <0.5 m 的回波。
            'max_height': 2.50,
            'angle_min': -3.141592653589793,
            'angle_max': 3.141592653589793,
            'angle_increment': 0.0174532925199433,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 15.0,
            # 【改动】use_inf 改为 False（原来是 True）。
            # 真值为 True 时，没有回波的方向填 +inf：实测 360 束里约有 210 束是 inf。
            # slam_toolbox 会自己丢掉 inf，但 RTAB-Map 把 LaserScan 转点云时会把
            # inf 带进去，建图几十个节点后内存爆掉——实测跑到 ~350 个节点时崩在
            # OpenCV "Failed to allocate 132 GB"（整车系统内存才 11 GB）。
            # False 时超量程/无回波填 range_max + inf_epsilon = 16.0，
            # 仍大于 slam_toolbox 的 max_laser_range(15.0)，依旧被当作"最远量程"
            # 处理，对 slam_toolbox 没有副作用。
            'use_inf': False,
            'inf_epsilon': 1.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable Gazebo GUI and joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='rviz', default_value='True',
                                            description='Flag to enable rviz2'),
        launch.actions.DeclareLaunchArgument(name='fastlio_config', default_value=fastlio_config,
                                            description='FAST-LIO parameter file'),
        launch.actions.DeclareLaunchArgument(name='fastlio_map_file', default_value='',
                                            description='FAST-LIO map_file_path (PCD save target)'),
        # 【改动】gui:=False 时只起 gzserver，不起 gzclient。本机是 WSL2 + Mesa
        # swrast 软件渲染，gzclient 打开 50×50 m 大场景会吃掉大量 CPU、拖慢仿真。
        # 当前 build_map.sh / build_3d_map.sh 都传 gui:=True（建图时需要看 Gazebo）；
        # 只有显式传 gui:=False 或 HEADLESS=1 时才走下面的 gzserver 分支。
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', world_path],
            condition=launch.conditions.IfCondition(LaunchConfiguration('gui')),
            output='screen'),
        launch.actions.ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', world_path],
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
            output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        fastlio_node,
        pointcloud_to_laserscan_node,
        rviz_node
    ])