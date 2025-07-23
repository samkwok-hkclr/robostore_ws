import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    params_file = LaunchConfiguration("params_file")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("robostore_bringup"), "params", "camera_w_d405_config.yaml"),
        description="",
    )

    ld.add_action(declare_params_file_cmd)

    realsense_node = Node(
        package='realsense2_camera',
        namespace="robostore",
        name="left_hand",
        executable='realsense2_camera_node',
        parameters=[
            params_file
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', "info"],
        emulate_tty=True,
    )

    camera_manager = Node(
        package='camera_manager',
        # namespace="",
        executable='camera_manager',
        parameters=[
            params_file
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', "info"],
        emulate_tty=True,
    )

    ld.add_action(realsense_node)
    ld.add_action(camera_manager)

    return ld