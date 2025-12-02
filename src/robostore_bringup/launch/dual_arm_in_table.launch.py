import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    EmitEvent, 
    ExecuteProcess,
    LogInfo, 
    RegisterEventHandler, 
    TimerAction, 
    Shutdown,
    OpaqueFunction
)
from launch.event_handlers import (
    OnExecutionComplete, 
    OnProcessExit,
    OnProcessIO, 
    OnProcessStart, 
     
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, 
    Command, 
    FindExecutable, 
    PathJoinSubstitution,
    PythonExpression
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def load_yaml(package_name, file_path):
    """Load YAML configuration file."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError as e:
        print(f"Failed to load YAML file {absolute_file_path}: {e}")
        return None


def declare_launch_arguments():
    """Declare all launch arguments."""
    arguments = []
    
    # Core MoveIt arguments
    arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz"
        )
    )
    
    arguments.append(
        DeclareLaunchArgument(
            "publish_frequency",
            default_value="15.0",
            description="Robot state publisher frequency"
        )
    )
    
    arguments.append(
        DeclareBooleanLaunchArg(
            "allow_trajectory_execution",
            default_value=True,
            description="Allow trajectory execution"
        )
    )
    
    arguments.append(
        DeclareBooleanLaunchArg(
            "publish_monitored_planning_scene",
            default_value=True,
            description="Publish monitored planning scene"
        )
    )
    
    arguments.append(
        DeclareBooleanLaunchArg(
            "monitor_dynamics",
            default_value=False,
            description="Monitor dynamics"
        )
    )
    
    # Robot configuration arguments
    arguments.append(
        DeclareLaunchArgument(
            "xacro_file",
            default_value="dual_arm_in_table.urdf.xacro",
            description="URDF XACRO file name"
        )
    )
    
    arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="dual_arm_in_table.rviz",
            description="RViz configuration file"
        )
    )
    
    # Left arm configuration
    arguments.append(
        DeclareLaunchArgument(
            "left_arm_sim",
            default_value="false",
            description="Use simulation for left arm"
        )
    )
    
    arguments.append(
        DeclareLaunchArgument(
            "left_arm_can_interface",
            default_value="can0",
            description="CAN interface for left arm"
        )
    )
    
    # Right arm configuration
    arguments.append(
        DeclareLaunchArgument(
            "right_arm_sim",
            default_value="false",
            description="Use simulation for right arm"
        )
    )
    
    arguments.append(
        DeclareLaunchArgument(
            "right_arm_can_interface",
            default_value="can1",
            description="CAN interface for right arm"
        )
    )
    
    return arguments


def create_moveit_configuration(context):
    """Create MoveIt configuration."""
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_robot", package_name="dual_arm_robot")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("robostore_bringup"),
                "urdf/dual_arm_in_table.urdf.xacro",
            ),
            mappings={
                "initial_positions_file": os.path.join(
                    get_package_share_directory("dual_arm_robot"),
                    "config/initial_positions.yaml",
                )
            },
        )
        .robot_description_semantic(file_path="config/dual_arm.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .sensors_3d(
            file_path=os.path.join(
                get_package_share_directory("dual_arm_robot"),
                "config/sensors_3d.yaml",
            )
        )
        .to_moveit_configs()
    )
    
    # Declare capabilities arguments (need to be done after moveit_config is created)
    capabilities = DeclareLaunchArgument(
        "capabilities",
        default_value=moveit_config.move_group_capabilities["capabilities"]
    )
    
    disable_capabilities = DeclareLaunchArgument(
        "disable_capabilities",
        default_value=moveit_config.move_group_capabilities["disable_capabilities"]
    )
    
    return moveit_config, [capabilities, disable_capabilities]


def create_move_group_configuration(moveit_config):
    """Create move group configuration parameters."""
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    
    move_group_configuration = {
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }
    
    octomap_config = {
        "octomap_resolution": 0.025,
        "max_range": 2.0
    }
    
    return [moveit_config.to_dict(), move_group_configuration, octomap_config]


def create_robot_state_publisher(context):
    """Create robot state publisher node."""
    left_arm_sim = LaunchConfiguration("left_arm_sim")
    left_arm_can_interface = LaunchConfiguration("left_arm_can_interface")
    right_arm_sim = LaunchConfiguration("right_arm_sim")
    right_arm_can_interface = LaunchConfiguration("right_arm_can_interface")
    xacro_file = LaunchConfiguration("xacro_file")
    
    package_dir = get_package_share_directory("robostore_bringup")
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                package_dir,
                "urdf",
                xacro_file
            ]),
            " ",
            "left_arm_sim:=", left_arm_sim,
            " ",
            "left_arm_can_interface:=", left_arm_can_interface,
            " ",
            "right_arm_sim:=", right_arm_sim,
            " ",
            "right_arm_can_interface:=", right_arm_can_interface,
        ]
    )
    
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "publish_frequency": LaunchConfiguration("publish_frequency")
        }],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ]
    )


def create_rviz_node(moveit_config, context):
    """Create RViz node."""
    rviz_config = LaunchConfiguration("rviz_config")
    package_dir = get_package_share_directory("robostore_bringup")
    
    rviz_config_file = PathJoinSubstitution([
        package_dir,
        "rviz",
        rviz_config
    ])
    
    return Node(
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
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )


def create_move_group_node(move_group_params):
    """Create move_group node."""
    return Node(
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

def create_ros2_control_node(moveit_config):
    """Create ROS2 control node."""
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ]
    )


def create_controller_spawners():
    """Create controller spawner nodes."""
    controllers = ["left_arm_controller", "right_arm_controller", "joint_state_broadcaster"]
    nodes = []
    
    for controller in controllers:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "controller_manager"],
                output="screen",
                remappings=[
                    ("~/tf", "/tf"),
                    ("~/tf_static", "/tf_static")
                ]
            )
        )
    
    return nodes

def generate_launch_description():
    """Generate launch description."""
    ld = LaunchDescription()
    
    # 1. Declare launch arguments
    ld.add_entity(OpaqueFunction(function=lambda context: declare_launch_arguments()))
    
    # 2. Create MoveIt configuration (with additional capability arguments)
    def setup_moveit_config(context):
        moveit_config, capability_args = create_moveit_configuration(context)
        
        # Add capability arguments to launch description
        for arg in capability_args:
            ld.add_action(arg)
        
        # 3. Create move group configuration
        move_group_params = create_move_group_configuration(moveit_config)
        
        # 4. Add nodes to launch description
        ld.add_action(create_robot_state_publisher(context))
        ld.add_action(create_move_group_node(move_group_params))
        ld.add_action(create_rviz_node(moveit_config, context))
        
        # 5. Add ROS2 control with delay
        ros2_control_node = create_ros2_control_node(moveit_config)
        ld.add_action(TimerAction(period=3.0, actions=[ros2_control_node]))
        
        # 6. Add controller spawners
        for spawner in create_controller_spawners():
            ld.add_action(spawner)
        
    # Use OpaqueFunction to access context for MoveIt configuration
    ld.add_entity(OpaqueFunction(function=setup_moveit_config))
    
    return ld