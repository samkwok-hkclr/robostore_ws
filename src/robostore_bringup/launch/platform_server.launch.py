import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

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

    robot_description_config = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robostore_bringup"), "urdf", "16w_env.urdf.xacro"]
            )
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_config, value_type=str),
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[
            ('/joint_states', '/robotic_platform/joint_states'), 
            # ('/tf', '/robotic_platform/tf'), 
            # ('/tf_static', '/robotic_platform/tf_static'),
            ('/robot_description', '/robotic_platform/robot_description')
        ]
    )

    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_col_obj_file_cmd)
    ld.add_action(declare_poses_file_cmd)

    platform_server = Node(
        package='robotic_platform',
        executable='platform_server',
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


    ld.add_action(platform_server)
    ld.add_action(robot_node)
    ld.add_action(vacuum_gripper_node)
    ld.add_action(robot_controller_node)

    ld.add_action(robot_state_publisher)

    return ld