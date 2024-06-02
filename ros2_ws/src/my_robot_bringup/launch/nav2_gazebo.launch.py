import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def print_paths(context):
    map_dir = LaunchConfiguration('map').perform(context)
    param_dir = LaunchConfiguration('params_file').perform(context)
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'rviz',
        'rviz_config.rviz')

    print("Map directory: ", map_dir)
    print("Param directory: ", param_dir)
    print("Nav2 launch file directory: ", nav2_launch_file_dir)
    print("RViz config directory: ", rviz_config_dir)


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default='/workspaces/auto_race/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/map_gazebo/map_gazebo.yaml')

    param_file_name = 'my_robot.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default='/workspaces/auto_race/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/param/my_robot.yaml')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'rviz',
        'rviz_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        OpaqueFunction(function=print_paths),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])

if __name__ == '__main__':
    generate_launch_description()
