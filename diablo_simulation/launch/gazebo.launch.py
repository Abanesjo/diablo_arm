import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = FindPackageShare(package='diablo_simulation').find('diablo_simulation')

    world_file_name = 'diablo.world'
    world_path = os.path.join(share_dir, 'worlds', world_file_name)

    gazebo_models_path = os.path.join(share_dir, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    plugins_dir = FindPackageShare(package='diablo_gazebo_plugin').find('diablo_gazebo_plugin')
    gazebo_plugins_path = os.path.join(plugins_dir, 'diablo_gazebo_plugin')
    os.environ["GAZEBO_PLUGIN_PATH"] = gazebo_plugins_path

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true',
            'world': world_path,
            'verbose': 'true'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    teleop_node = Node(
        name='teleop_keyboard',
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        remappings=[
            ('/cmd_vel', '/diablo/diablo/vel_cmd')
        ],
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        teleop_node
    ])