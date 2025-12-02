import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    print(absolute_file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    ld = LaunchDescription()
    
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_robot", package_name="dual_arm_robot")
        .robot_description(
            file_path=os.path.join(
                    get_package_share_directory("robostore_bringup"),
                    "urdf/dual_arm_robot_16w_env.urdf.xacro",
            ),
            mappings={
                "initial_positions_file": os.path.join(
                    get_package_share_directory("dual_arm_robot"),
                    "config/initial_positions.yaml",
            )},
        )
        .robot_description_semantic(file_path="config/dual_arm_robot.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # .planning_pipelines(
        #     pipelines=["chomp", "pilz_industrial_motion_planner"] # "ompl", 
        # )
        .sensors_3d(
            file_path=os.path.join(
                get_package_share_directory("dual_arm_robot"),
                "config/sensors_3d.yaml",
            )
        )
        .to_moveit_configs()
    )
    
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    
    left_arm_sim = LaunchConfiguration("left_arm_sim")
    left_arm_can_interface = LaunchConfiguration("left_arm_can_interface")
    right_arm_sim = LaunchConfiguration("right_arm_sim")
    right_arm_can_interface = LaunchConfiguration("right_arm_can_interface")
    fold_elevator_sim = LaunchConfiguration("fold_elevator_sim")
    fold_elevator_can_interface = LaunchConfiguration("fold_elevator_can_interface")
    
    use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")
    pub_freq = DeclareLaunchArgument("publish_frequency", default_value="15.0")

    allow_traj_exec = DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    pub_mon_planning_scene = DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    cap = DeclareLaunchArgument("capabilities", default_value=moveit_config.move_group_capabilities["capabilities"])
    dis_cap= DeclareLaunchArgument("disable_capabilities", default_value=moveit_config.move_group_capabilities["disable_capabilities"])
    mon_dynamics = DeclareBooleanLaunchArg("monitor_dynamics", default_value=False)
    
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_arm_sim",
            default_value="true",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_arm_can_interface",
            default_value="sim_can0",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_arm_sim",
            default_value="true",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_arm_can_interface",
            default_value="sim_can1",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "fold_elevator_sim",
            default_value="true",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "fold_elevator_can_interface",
            default_value="sim_can2",
        )
    )
    
    for arg in declared_arguments:
        ld.add_action(arg)
    
    ld.add_action(use_rviz)
    ld.add_action(pub_freq)

    ld.add_action(allow_traj_exec)
    ld.add_action(pub_mon_planning_scene)
    ld.add_action(cap)
    ld.add_action(dis_cap)
    ld.add_action(mon_dynamics)
    
    # ompl_planning_pipeline_config = { 
    #     "move_group" : {
    #         "planning_plugin" : "ompl_interface/OMPLPlanner",
    #         "request_adapters" : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
    #         "start_state_max_bounds_error" : 0.1 
    #     } 
    # }
    # ompl_planning_yaml = load_yaml("dual_arm_robot", "config/ompl_planning.yaml")
    # ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    
    move_group_configuration = {
        # "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    octomap_config = {
        # "octomap_frame": "base_link",  # if mobile robot, should be a fixed frame in the world
        "octomap_resolution": 0.025,
        "max_range": 2.0
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        octomap_config,
        # ompl_planning_pipeline_config
    ]
    
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        remappings=[
            ("~/tf", "/tf"),
            ("~/tf_static", "/tf_static"),
            ("~/robot_description", "/robot_description"),
        ]
    )
    ld.add_action(run_move_group_node)
    
    # Declare launch arguments
    xacro_file_arg = DeclareLaunchArgument(
        "xacro_file",
        default_value="dual_arm_robot_16w_env.urdf.xacro",
        description=""
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="robot_view.rviz"
        ,
        description="RViz config file"
    )

    # Get package share directory
    package_dir = get_package_share_directory("robostore_bringup")
    
    # XACRO processing - this will be processed at launch time
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                package_dir,
                "urdf",
                LaunchConfiguration("xacro_file")
            ]),
            " ",
            "left_arm_sim:=",
            left_arm_sim,
            " ",
            "left_arm_can_interface:=",
            left_arm_can_interface,
            " ",
            "right_arm_sim:=",
            right_arm_sim,
            " ",
            "right_arm_can_interface:=",
            right_arm_can_interface,
            " ",
            "fold_elevator_sim:=",
            fold_elevator_sim,
            " ",
            "fold_elevator_can_interface:=",
            fold_elevator_can_interface,
            " ",
        ]
    )
    
    # RViz config path
    rviz_config_file = PathJoinSubstitution([
        package_dir,
        "rviz",
        LaunchConfiguration("rviz_config")
    ])
    
    # Add launch arguments
    ld.add_action(xacro_file_arg)
    ld.add_action(rviz_config_arg)
    # ld.add_action(use_gui_arg)
    
    # Robot State Publisher - REMOVED DUPLICATE
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description_content
            }],
            remappings=[
                ("~/robot_description", "/robot_description"),
            ]
        )
    )
    
    # RViz
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
            ],
        )
    )
    
    can_brigdes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("can_brigde"),
                    "launch",
                    "can_brigde.launch.py")
            )
        )
    ld.add_action(can_brigdes)
    
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            # moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ]
    )
    ros2_control_event = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=ros2_control,
                on_start=[can_brigdes]
            )
        )
    ld.add_action(ros2_control_event)

    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "-c", "controller_manager"
                ],
                output="screen",
                remappings=[
                    # ("~/tf", "tf"),
                    # ("~/tf_static", "tf_static")
                ]
            )
        )

    
    return ld