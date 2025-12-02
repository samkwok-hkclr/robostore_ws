import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    LogInfo,
    SetLaunchConfiguration
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate simulation launch description."""
    
    ld = LaunchDescription()

    # Set simulation parameters
    ld.add_action(
        SetLaunchConfiguration('left_arm_sim', 'true')
    )
    ld.add_action(
        SetLaunchConfiguration('right_arm_sim', 'true')
    )
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robostore_bringup'),
                    'launch',
                    'dual_arm_in_table.launch.py'
                ])
            ]),
            launch_arguments={
                'left_arm_sim': 'true',
                'right_arm_sim': 'true',
                'use_rviz': 'true',
                'xacro_file': 'dual_arm_in_table.urdf.xacro',
            }.items()
        )
    )
    
    return ld