import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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
    behavior_cfg = os.path.join(pkg, 'config', 'behavior_params.yaml')
    
    # Doğrudan src klasöründeki tam yolu alıyoruz:
    home_dir = os.path.expanduser('~')
    tomato_script = os.path.join(home_dir, 'ws', 'src', 'my_robot_pkg', 'tools', 'tomato_counter.py')
    
    world_arg = DeclareLaunchArgument(
        'world', default_value='greenhouse.sdf',
        description='worlds/ klasöründeki SDF dosyasının adı')
    world_path = PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='0.0',
        description='Robotun başlangıç x konumu (m)')
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
                            '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'), '-z', '0.1'])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': bridge_cfg}])

    rviz_args = ['-d', rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz = Node(package='rviz2', executable='rviz2', arguments=rviz_args,
                parameters=[{'use_sim_time': True}])

    # Python scriptini doğrudan tam dosya yolundan çalıştırıyoruz
    tomato_counter = ExecuteProcess(
        cmd=['python3', tomato_script],
        output='screen'
    )

    # Uye C: tek /cmd_vel cikisi (bumper > tilt > lidar > varsayilan)
    behavior = Node(package='my_robot_pkg', executable='behavior_node',
                    name='behavior_node', output='screen', parameters=[behavior_cfg])

    return LaunchDescription([world_arg, spawn_x_arg, spawn_y_arg, gz, rsp, spawn, bridge, rviz,
                              tomato_counter, behavior])
