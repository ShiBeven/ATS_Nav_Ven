import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('standard_robot_pp_ros2')
    vision_share = get_package_share_directory('sp_vision25')

    nav_params = os.path.join(nav_share, 'config', 'standard_robot_pp_ros2.yaml')
    # 视觉配置：路径解析宏同时支持 share 目录定位，见 sp_vision25/tools/path.cpp
    vision_config = os.path.join(vision_share, 'configs', 'standard3.yaml')

    vision_bin = os.path.join(vision_share, 'lib', 'sp_vision25', 'auto_aim_debug_mpc')

    return LaunchDescription([
        # 1) 导航串口节点（独占 /dev/ttyACM0，含 gimbal 桥接）
        Node(
            package='standard_robot_pp_ros2',
            executable='standard_robot_pp_ros2_node',
            name='standard_robot_pp_ros2',
            output='screen',
            parameters=[nav_params],
        ),
        # 2) 视觉主程序（话题桥接，不再开串口）
        Node(
            package='sp_vision25',
            executable='auto_aim_debug_mpc',
            name='vision_auto_aim',
            output='screen',
            arguments=[vision_config],
            emulate_tty=True,
        ),
    ])
