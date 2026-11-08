"""启动全覆盖清扫任务：Coverage Server + 生命周期管理 + 执行器。

单独使用：
  ros2 launch coverage_mission coverage.launch.py
与导航一起使用：
  ros2 launch sam_bot_description navigation.launch.py coverage:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('coverage_mission')

    default_params = os.path.join(pkg_share, 'config', 'coverage_params.yaml')
    default_area = os.path.join(pkg_share, 'config', 'coverage_area.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    # 注意：launch configuration 是全局的。这个文件会被 navigation.launch.py
    # include，而那边已经声明了 params_file / autostart，同名参数会被父级
    # 抢占（现象：coverage_server 实际加载的是 nav2_params.yaml，
    # operation_width 退回默认 2.5 m，扫出来的条带间距全错）。
    # 因此这里统一加 coverage_ 前缀，避免与 Nav2 的参数名冲突。
    params_file = LaunchConfiguration('coverage_params_file')
    area_file = LaunchConfiguration('coverage_area_file')
    autostart = LaunchConfiguration('coverage_autostart')

    # 1) Coverage Server（opennav_coverage 的 LifecycleNode）
    coverage_server = Node(
        package='opennav_coverage',
        executable='opennav_coverage',
        name='coverage_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # 2) 管理 coverage_server 的生命周期。
    #    延迟 8 s：等 /clock、TF 和 DDS 发现就绪，避免 change_state 请求丢失。
    lifecycle_manager = TimerAction(
        period=8.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_coverage',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['coverage_server'],
                'bond_timeout': 10.0,
            }],
        )],
    )

    # 3) 任务执行器（等 action server 就绪后自动开始）
    executor = TimerAction(
        period=12.0,
        actions=[Node(
            package='coverage_mission',
            executable='coverage_executor',
            name='coverage_executor',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'area_file': area_file,
            }],
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟'),
        DeclareLaunchArgument(
            'coverage_params_file', default_value=default_params,
            description='覆盖任务参数文件'),
        DeclareLaunchArgument(
            'coverage_area_file', default_value=default_area,
            description='覆盖区域 YAML（由 extract_coverage_area 生成）'),
        DeclareLaunchArgument(
            'coverage_autostart', default_value='true',
            description='是否自动 configure/activate coverage_server'),
        coverage_server,
        lifecycle_manager,
        executor,
    ])
