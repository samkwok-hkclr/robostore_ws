import os
import sys
import xacro
import tempfile
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

'''
THIS LAUNCH FILE IS FOR VIEWING CAMERA MODEL ONLY!!!
'''

def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path

def generate_launch_description():
    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'test_l515_camera.urdf.xacro')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics' : 'true', 'add_plug' : 'true'})

    model_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments = [urdf]
    )
    return launch.LaunchDescription([model_node])
