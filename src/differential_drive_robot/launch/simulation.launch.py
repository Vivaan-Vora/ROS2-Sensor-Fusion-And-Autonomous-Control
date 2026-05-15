"""Launch Gazebo and the complete differential-drive sensor fusion stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("differential_drive_robot")
    gazebo_ros_share = FindPackageShare("gazebo_ros")

    robot_config = PathJoinSubstitution([pkg_share, "config", "robot.yaml"])
    world_path = PathJoinSubstitution([pkg_share, "gazebo", "worlds", "robot_world.sdf"])
    model_path = PathJoinSubstitution([pkg_share, "gazebo", "models"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world, "verbose": "false"}.items(),
    )

    common_parameters = [robot_config, {"use_sim_time": use_sim_time}]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=world_path),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value=[model_path, ":", EnvironmentVariable("GAZEBO_MODEL_PATH", default_value="")],
            ),
            gazebo_launch,
            Node(
                package="differential_drive_robot",
                executable="imu_publisher.py",
                name="imu_publisher",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="differential_drive_robot",
                executable="odom_publisher.py",
                name="odom_publisher",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="differential_drive_robot",
                executable="sensor_fusion_node",
                name="sensor_fusion_node",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="differential_drive_robot",
                executable="controller_node",
                name="controller_node",
                output="screen",
                parameters=common_parameters,
            ),
        ]
    )
