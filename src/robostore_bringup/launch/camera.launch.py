import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.substitutions import FindPackageShare
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from lifecycle_msgs.msg import Transition

left_camera_params = {
    "camera_name": "left_camera",
    "serial_no": "405622074042",
    "usb_port_id": "",
    "device_type": "d435(?!i)",
    "initial_reset": True,

    "enable_color": True,
    "rgb_camera.color_profile": "1280,720,6",
    "rgb_camera.color_format": "RGB8",
    "rgb_camera.enable_auto_exposure": True,
    "rgb_camera.power_line_frequency": 1,
    "enable_auto_white_balance": True,   # flat param (not under rgb_camera)
    
    # "hdr_merge": True,

    "enable_depth": True,
    "depth_module.depth_profile": "1280,720,6",
    "depth_module.depth_format": "Z16",
    # "depth_module.hdr_enabled": True,
    "depth_module.min_distance": 190,
    "depth_module.digital_gain": 2,
    "depth_module.receiver_gain": 18,
    "depth_module.noise_filtering": 4,
    "depth_module.post_processing_sharpening": 1,
    "depth_module.pre_processing_sharpening": 0,
    "depth_module.sensor_mode": 1,
    "depth_module.visual_preset": 5,
    "depth_module.invalidation_bypass": False,

    "enable_infra": False,
    "enable_infra1": False,
    "enable_infra2": False,
    "enable_confidence": False,

    "enable_gyro": False,
    "enable_accel": False,
    "gyro_fps": 0,
    "accel_fps": 0,
    "unite_imu_method": 2,
    "enable_sync": True,
    "intra_process_comms": True,

    "enable_rgbd": True,

    "pointcloud__neon_.enable": True,
    "pointcloud__neon_.stream_filter": 2,
    "pointcloud__neon_.ordered_pc": False,
    "pointcloud__neon_.allow_no_texture_points": False,

    "align_depth.enable": True,
    "colorizer.enable": False,
    "decimation_filter.enable": True,
    "spatial_filter.enable": True,
    "temporal_filter.enable": True,
    "disparity_filter.enable": False,
    "hole_filling_filter.enable": True,
    "hdr_merge.enable": True,

    "publish_tf": True,
    "tf_publish_rate": 0.0,
    "publish_odom_tf": False,

    "clip_distance": 1.0,
    "angular_velocity_cov": 0.01,
    "linear_accel_cov": 0.01,
    "diagnostics_period": 1.0,
    "wait_for_device_timeout": -1.0,
    "reconnect_timeout": 3.0,
}

right_camera_params = {
    "camera_name": "right_camera",
    "serial_no": "138422074515",
    "usb_port_id": "",
    "device_type": "d435(?!i)",
    "initial_reset": True,

    "enable_color": True,
    "rgb_camera.color_profile": "1280,720,6",
    "rgb_camera.color_format": "RGB8",
    "rgb_camera.enable_auto_exposure": True,
    "rgb_camera.power_line_frequency": 1,
    "enable_auto_white_balance": True,
    
    # "hdr_merge": True,

    "enable_depth": True,
    "depth_module.depth_profile": "1280,720,6",
    "depth_module.depth_format": "Z16",
    # "depth_module.hdr_enabled": True,
    "depth_module.min_distance": 190,
    "depth_module.digital_gain": 2,
    "depth_module.receiver_gain": 18,
    "depth_module.noise_filtering": 4,
    "depth_module.post_processing_sharpening": 1,
    "depth_module.pre_processing_sharpening": 0,
    "depth_module.sensor_mode": 1,
    "depth_module.visual_preset": 5,
    "depth_module.invalidation_bypass": False,

    "enable_infra": False,
    "enable_infra1": False,
    "enable_infra2": False,
    "enable_confidence": False,

    "enable_gyro": False,
    "enable_accel": False,
    "gyro_fps": 0,
    "accel_fps": 0,
    "unite_imu_method": 2,
    "enable_sync": True,
    "intra_process_comms": True,

    "enable_rgbd": True,

    "pointcloud__neon_.enable": True,
    "pointcloud__neon_.stream_filter": 2,
    "pointcloud__neon_.ordered_pc": False,
    "pointcloud__neon_.allow_no_texture_points": False,

    "align_depth.enable": True,
    "colorizer.enable": False,
    "decimation_filter.enable": True,
    "spatial_filter.enable": True,
    "temporal_filter.enable": True,
    "disparity_filter.enable": False,
    "hole_filling_filter.enable": True,
    "hdr_merge.enable": True,

    "publish_tf": True,
    "tf_publish_rate": 0.0,
    "publish_odom_tf": False,

    "clip_distance": 1.0,
    "angular_velocity_cov": 0.01,
    "linear_accel_cov": 0.01,
    "diagnostics_period": 1.0,
    "wait_for_device_timeout": -1.0,
    "reconnect_timeout": 3.0,
}

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def generate_launch_description():
    ld = LaunchDescription()

    camera_params_file = LaunchConfiguration("camera_params_file")

    camera_params_file_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=os.path.join(
            get_package_share_directory("robostore_bringup"), "params", "camera_w_d435_config.yaml"),
        description=""
    )
    auto_configure_arg = DeclareLaunchArgument('auto_configure', default_value='true')
    auto_activate_arg = DeclareLaunchArgument('auto_activate', default_value='false')

    ld.add_action(camera_params_file_arg)
    ld.add_action(auto_configure_arg)
    ld.add_action(auto_activate_arg)

    CAMERA_NODE = ["left_camera", "right_camera"]
    
    for camera in CAMERA_NODE:
        # Use LifecycleNode for better lifecycle management
        params = left_camera_params if camera == "left_camera" else right_camera_params
        node = LifecycleNode(
            package='realsense2_camera',
            namespace=camera,
            name="realsense",
            executable='realsense2_camera_node',
            parameters=[
                camera_params_file,
                params
            ],
            output="screen",
            arguments=['--ros-args', '--log-level', "info"],
            emulate_tty=True,
        )
        
        configure_event_handler = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node,
                on_start=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(node),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        ),
                    ),
                ],
            ),
            condition=IfCondition(LaunchConfiguration('auto_configure')),
        )
        
        activate_event_handler = RegisterEventHandler(
            event_handler=OnStateTransition(
                target_lifecycle_node=node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        ),
                    ),
                ],
            ),
            condition=IfCondition(LaunchConfiguration('auto_activate')),
        )
        
        ld.add_action(node)
        ld.add_action(configure_event_handler)
        ld.add_action(activate_event_handler)
            
    camera_manager = Node(
        package='camera_manager',
        executable='camera_manager',
        parameters=[camera_params_file],
        output="screen",
        arguments=['--ros-args', '--log-level', "info"],
        emulate_tty=True,
    )

    # web_video_server = Node(
    #     package='web_video_server',
    #     executable='web_video_server',
    #     parameters=[camera_params_file],
    #     output="screen",
    #     arguments=['--ros-args', '--log-level', "info"],
    #     emulate_tty=True,
    # )

    ld.add_action(camera_manager)
    # ld.add_action(web_video_server)

    return ld

