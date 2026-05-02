import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    package_name = 'mobile_robot'
    robot_name = 'differential_drive_robot'
    lidar_sensor_frame = f'{robot_name}/base_footprint/lidar_sensor'

    model_path = os.path.join(
        get_package_share_directory(package_name),
        'model',
        'robot.xacro',
    )
    bridge_path = os.path.join(
        get_package_share_directory(package_name),
        'parameters',
        'bridge_parameters.yaml',
    )
    world_path = LaunchConfiguration('world').perform(context)

    robot_description = xacro.process_file(
        model_path,
        mappings={
            'arducam_pan_initial': LaunchConfiguration('arducam_pan_initial').perform(context),
            'arducam_tilt_initial': LaunchConfiguration('arducam_tilt_initial').perform(context),
            'arducam_horizontal_fov': LaunchConfiguration('arducam_horizontal_fov').perform(context),
        },
    ).toxml()

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            # Default to a small obstacle course so the 2D lidar has useful geometry to hit.
            'gz_args': f'-r -v4 {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawn directly from the generated URDF string so the launch path does not depend on ROS topics.
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-string', robot_description,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
    )

    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_path}',
        ],
    )

    # Gazebo's gpu_lidar point cloud uses the scoped sensor name as frame_id.
    # Publish a matching static transform so RViz can place /scan/points in odom.
    lidar_pointcloud_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_pointcloud_frame_broadcaster',
        output='screen',
        arguments=[
            '0.10', '0.0', '0.45',
            '0.0', '0.0', '0.0',
            'base_footprint',
            lidar_sensor_frame,
        ],
    )

    return [
        gazebo_launch,
        robot_state_publisher,
        spawn_model,
        gazebo_bridge,
        lidar_pointcloud_tf,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('mobile_robot'),
                'worlds',
                'obstacle_course.sdf',
            ),
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
        OpaqueFunction(function=launch_setup),
    ])
