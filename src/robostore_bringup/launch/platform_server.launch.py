import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_robot", package_name="dual_arm_robot")
        .robot_description(
            file_path=os.path.join(
                    get_package_share_directory("robostore_bringup"),
                    "urdf/dual_arm_robot_16w_env.urdf.xacro",
            )
        )
            # file_path="urdf/dual_arm_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_arm_robot.srdf")
        .to_moveit_configs()
    )
    
    ld = LaunchDescription()

    use_respawn = LaunchConfiguration("use_respawn")
    params_file = LaunchConfiguration("params_file")
    # collision_objects_file = LaunchConfiguration("collision_objects_file")
    poses_file = LaunchConfiguration("poses_file")

    declare_use_respawn = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("robostore_bringup"), "params", "manipulation_config.yaml"),
        description="",
    )
    
    # declare_col_obj_file_cmd = DeclareLaunchArgument(
    #     "collision_objects_file",
    #     default_value=os.path.join(
    #         get_package_share_directory("robostore_bringup"), "params", "collision_objects.yaml"),
    #     description="",
    # )
    declare_poses_file_cmd = DeclareLaunchArgument(
        "poses_file",
        default_value=os.path.join(
            get_package_share_directory("robostore_bringup"), "params", "poses.yaml"),
        description="",
    )

    ld.add_action(declare_use_respawn)
    ld.add_action(declare_params_file)
    # ld.add_action(declare_col_obj_file_cmd)
    ld.add_action(declare_poses_file_cmd)

    platform_server = Node(
        package='robotic_platform',
        executable='platform_server',
        parameters=[
            params_file,
            {
                # "collision_objects_file": collision_objects_file,
                "poses_file": poses_file,
            }
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', "info"],
        emulate_tty=True,
    )
    ld.add_action(platform_server)

    gripper_node = Node(
        package="gripper",
        executable="gripper",
        parameters=[
            params_file,
        ],
        respawn=use_respawn,
        respawn_delay=3.0,
        output="both",
    )
    ld.add_action(gripper_node)
    
    controllers = ["left_arm", "left_action_arm", "right_arm", "right_action_arm", "fold_elevator"]
    
    for controller in controllers:
        node = Node(
            package="robot_controller",
            executable="robot_controller_node",
            namespace=controller,
            parameters=[
                params_file,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,  
            ],
            respawn=use_respawn,
            respawn_delay=3.0,
            output="screen",
            remappings=[
                ("robot_description", "/robot_description"),
                ("robot_description_semantic", "/robot_description_semantic"),
                ("get_planning_scene", "/get_planning_scene"),
                ("apply_planning_scene", "/apply_planning_scene"),
                ("joint_states", "/joint_states"),
            ]
        )
        ld.add_action(node)

    return ld