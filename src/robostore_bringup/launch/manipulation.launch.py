import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    use_respawn = LaunchConfiguration("use_respawn")
    params_file = LaunchConfiguration("params_file")
    collision_objects_file = LaunchConfiguration("collision_objects_file")
    poses_file = LaunchConfiguration("poses_file")

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("robostore_bringup"), "params", "manipulation_config.yaml"),
        description="",
    )
    declare_col_obj_file_cmd = DeclareLaunchArgument(
        "collision_objects_file",
        default_value=os.path.join(
            get_package_share_directory("robostore_bringup"), "params", "collision_objects.yaml"),
        description="",
    )
    declare_poses_file_cmd = DeclareLaunchArgument(
        "poses_file",
        default_value=os.path.join(
            get_package_share_directory("robostore_bringup"), "params", "poses.yaml"),
        description="",
    )

    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_col_obj_file_cmd)
    ld.add_action(declare_poses_file_cmd)

    manipulation_server = Node(
        package='robotic_platform',
        executable='manipulation_server',
        parameters=[
            params_file,
            {
                "collision_objects_file": collision_objects_file,
                "poses_file": poses_file,
            }
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', "info"],
        emulate_tty=True,
    )

    robot_node = Node(
        package="nachi_robot",
        executable="nachi_robot",
        parameters=[
            params_file,
        ],
        respawn=use_respawn,
        respawn_delay=3.0,
        output="both",
    )

    vacuum_gripper_node = Node(
        package="vacuum_gripper",
        executable="vacuum_gripper",
        parameters=[
            params_file,
        ],
        respawn=use_respawn,
        respawn_delay=3.0,
        output="both",
    )

    robot_controller_node = Node(
        package="robot_controller",
        executable="robot_controller_node",
        parameters=[
            params_file,
        ],
        respawn=use_respawn,
        respawn_delay=3.0,
        output="screen",
    )


    ld.add_action(manipulation_server)
    ld.add_action(robot_node)
    ld.add_action(vacuum_gripper_node)
    ld.add_action(robot_controller_node)

    return ld