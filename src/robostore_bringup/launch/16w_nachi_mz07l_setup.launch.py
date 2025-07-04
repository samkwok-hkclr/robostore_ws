import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print(absolute_file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    launch_rviz = LaunchConfiguration("launch_rviz")

    declare_launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="True",
        description="Launch RViz",
    )

    description_pkg_name = "mz07l_description"
    moveit_config_pkg_name = "mz07l_moveit_config"

    robot_description_config = load_file(description_pkg_name, "urdf/mz07L_dual.urdf")
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file(moveit_config_pkg_name, "config/mz07L.srdf")
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml(moveit_config_pkg_name, 'config/kinematics.yaml')

    joint_limits_yaml = {'robot_description_planning': load_yaml(moveit_config_pkg_name, 'config/joint_limits.yaml')}

    ompl_planning_pipeline_config = { 
        'move_group' : {
            'planning_plugin' : 'ompl_interface/OMPLPlanner',
            'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
            'start_state_max_bounds_error' : 0.1 
        } 
    }
    ompl_planning_yaml = load_yaml(moveit_config_pkg_name, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    controllers_yaml = load_yaml(moveit_config_pkg_name, 'config/controllers.yaml')

    moveit_controllers = { 
        'moveit_simple_controller_manager' : controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 20.0,
        'trajectory_execution.allowed_start_tolerance': 0.1
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True
    }

    move_group_node_param = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        ompl_planning_pipeline_config,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        joint_limits_yaml
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        #prefix=['gdb -ex=r --args'],
        parameters=move_group_node_param
    )

    # Publish TF Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    robot_node = Node(
        package='nachi_robot',
        executable='nachi_robot_node',
        name='nachi_robot_node',
        output='both',
        parameters=[
            {
                "ip": "192.168.2.20",
                "model": "mz07L",
                "fake": True,
            }
        ]
    )

    # RViz
    rviz_config_file = os.path.join(get_package_share_directory("robostore_bringup"), "rviz", "moveit.rviz")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition = IfCondition(launch_rviz),
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description,
                    robot_description_semantic,
                    ompl_planning_pipeline_config,
                    kinematics_yaml,
                    joint_limits_yaml])

    ld = LaunchDescription()

    ld.add_action(declare_launch_rviz_cmd)
    ld.add_action(move_group_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(robot_node)
    ld.add_action(rviz_node)

    return ld