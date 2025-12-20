from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_s1_driver = get_package_share_directory('s1_driver')
    
    # Default parameter file path
    default_params_file = os.path.join(pkg_s1_driver, 'config', 's1_driver_params.yaml')

    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the ROS2 parameters file to use'
    )

    # Create the node
    s1_driver_node = Node(
        package='s1_driver',
        executable='s1_driver_node',
        name='s1_driver',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True
    )

    return LaunchDescription([
        params_file_arg,
        s1_driver_node
    ])
