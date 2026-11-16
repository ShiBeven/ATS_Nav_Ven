import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    simulator_share = get_package_share_directory("rmu_gazebo_simulator")
    navigation_share = get_package_share_directory("pb2025_nav_bringup")
    behavior_share = get_package_share_directory("pb2025_sentry_behavior")
    vision_share = get_package_share_directory("sp_vision25")

    world = LaunchConfiguration("world")
    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    launch_vision = LaunchConfiguration("launch_vision")
    launch_trajectory_optimizer = LaunchConfiguration("launch_trajectory_optimizer")
    launch_small_gicp_relocalization = LaunchConfiguration(
        "launch_small_gicp_relocalization"
    )
    log_level = LaunchConfiguration("log_level")
    map_file = LaunchConfiguration("map")
    vision_config = LaunchConfiguration("vision_config")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_share, "launch", "bringup_sim.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, "launch", "rm_navigation_simulation_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "world": world,
            "map": map_file,
            "prior_pcd_file": "",
            "use_sim_time": "True",
            "use_rviz": use_rviz,
            "use_composition": "False",
            "launch_chassis_vel_transform": "True",
            "launch_small_gicp_relocalization": launch_small_gicp_relocalization,
            "launch_trajectory_optimizer": launch_trajectory_optimizer,
            "use_respawn": "False",
            "log_level": log_level,
        }.items(),
    )

    gimbal_manager = Node(
        package="standard_robot_pp_ros2",
        executable="gimbal_manager_node",
        name="gimbal_manager",
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    fake_inputs = Node(
        package="pb2025_sentry_bringup",
        executable="fake_decision_sim_inputs.py",
        name="fake_decision_sim_inputs",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "publish_vision_target": False,
                "publish_robot_status": False,
                "publish_rfid_status": False,
            }
        ],
    )

    referee_adapter = Node(
        package="pb2025_sentry_bringup",
        executable="gazebo_referee_adapter.py",
        name="gazebo_referee_adapter",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": True, "robot_name": namespace}
        ],
    )

    behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(behavior_share, "launch", "pb2025_sentry_behavior_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": "True",
            "params_file": os.path.join(behavior_share, "params", "sentry_behavior.yaml"),
            "log_level": log_level,
        }.items(),
    )

    delayed_behavior = TimerAction(period=5.0, actions=[behavior])

    vision = Node(
        package="sp_vision25",
        executable="auto_aim_debug_mpc",
        namespace=namespace,
        output="screen",
        arguments=[vision_config],
        parameters=[{"use_sim_time": True}],
        additional_env={
            "SP_VISION_CAMERA_MODE": "ros_image",
            "SP_VISION_CAMERA_TOPIC": "front_industrial_camera/image",
            "SP_VISION_BULLET_SPEED": "20.0",
        },
        condition=IfCondition(launch_vision),
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="rmuc_2025"),
            DeclareLaunchArgument("namespace", default_value="red_standard_robot1"),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("launch_vision", default_value="True"),
            DeclareLaunchArgument("launch_trajectory_optimizer", default_value="True"),
            DeclareLaunchArgument(
                "launch_small_gicp_relocalization", default_value="True"
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [
                        navigation_share,
                        "map",
                        "simulation",
                        PythonExpression(["'", world, " + '.yaml'"]),
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "vision_config",
                default_value=os.path.join(vision_share, "configs", "standard3.yaml"),
            ),
            gazebo,
            navigation,
            gimbal_manager,
            fake_inputs,
            referee_adapter,
            delayed_behavior,
            vision,
        ]
    )
