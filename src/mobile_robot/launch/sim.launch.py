import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('mobile_robot')
    gazebo_launch = os.path.join(package_share, 'launch', 'gazebo_model.launch.py')
    rviz_launch = os.path.join(package_share, 'launch', 'rviz_sensors.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(package_share, 'worlds', 'obstacle_course.sdf'),
            description='Gazebo world file to load.',
        ),
        DeclareLaunchArgument(
            'arducam_pan_initial',
            default_value='0.0',
            description='Initial pan angle for the Arducam PTZ head.',
        ),
        DeclareLaunchArgument(
            'arducam_tilt_initial',
            default_value='0.0',
            description='Initial tilt angle for the Arducam PTZ head.',
        ),
        DeclareLaunchArgument(
            'arducam_horizontal_fov',
            default_value='0.78539816339',
            description='Horizontal FOV used to emulate Arducam zoom.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'rviz', 'mobile_robot_sensors.rviz'),
            description='RViz configuration file to load.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'arducam_pan_initial': LaunchConfiguration('arducam_pan_initial'),
                'arducam_tilt_initial': LaunchConfiguration('arducam_tilt_initial'),
                'arducam_horizontal_fov': LaunchConfiguration('arducam_horizontal_fov'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            launch_arguments={
                'rviz_config': LaunchConfiguration('rviz_config'),
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
