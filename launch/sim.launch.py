import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_pkg')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'robot.rviz')

    # Dünya seçimi: varsayılan sera. Eski dünya için: world:=obstacles.sdf
    world_arg = DeclareLaunchArgument(
        'world', default_value='greenhouse.sdf',
        description='worlds/ klasöründeki SDF dosyasının adı')
    world_path = PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])

    # Başlangıç konumu: koridor 1 (rampalı) y=0.6, koridor 2 (engelli) y=-0.6
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='0.6',
        description='Robotun başlangıç y konumu (m)')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world_path]}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'my_robot',
                            '-x', '-1', '-y', LaunchConfiguration('spawn_y'), '-z', '0.1'])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': bridge_cfg}])

    rviz_args = ['-d', rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz = Node(package='rviz2', executable='rviz2', arguments=rviz_args,
                parameters=[{'use_sim_time': True}])

    return LaunchDescription([world_arg, spawn_y_arg, gz, rsp, spawn, bridge, rviz])
