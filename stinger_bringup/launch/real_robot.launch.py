'''
launches the tug
TODO: TF, sensors, localization, motors, perception, autonomy (state + LiDAR obstacle node), controllers.
before launching sudo pigpiod
'''

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    # URDF -> robot_state_publisher, gives the real robot the base_link -> sensor TFs (the sim spawn launch does this in sim)
    xacro_file = os.path.join(
        get_package_share_directory('stinger_description'),
        'urdf',
        'stinger_tug.urdf.xacro'
    )
    robot_description = xacro.process(xacro_file)
    bringup_share = get_package_share_directory('stinger_bringup')
    controller_share = get_package_share_directory('stinger_controller')
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'sensors.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'localization.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items(),  # real robot has no /clock
        ),
        Node(
            package='stinger_bringup',
            executable='motor-node',
            name='motor_node',
            output='screen',
        ),
        Node(
            package='stinger_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
        ),
        Node(
            package='stinger_autonomy',
            executable='obstacle_node',
            name='obstacle_node',
            output='screen',
        ),
        Node(
            package='stinger_autonomy',
            executable='state_node',
            name='state_node',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(controller_share, 'launch', 'controller.launch.py'))
        ),
    ])