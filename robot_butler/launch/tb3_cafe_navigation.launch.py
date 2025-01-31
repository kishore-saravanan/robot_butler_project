from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('robot_butler'),'config')
    rviz_config= os.path.join(config_dir,'tb3_nav.rviz')
    map_file = os.path.join(config_dir,'cafe_map.yaml')
    params_file = os.path.join(config_dir,'tb3_nav_params.yaml')
    map_config= os.path.join(config_dir,'mapping.rviz')
    nav_config= os.path.join(config_dir,'tb3_nav.rviz')

    # Bringing our Robot
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('turtlebot3_gazebo'),'/launch','/turtlebot3_cafe.launch.py']),
        launch_arguments={
        'x_pose':'-3.0',
        'y_pose': '6.0'}.items(),
    )
     # Integerating Nav2 Stack
    robot_navigation_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
        launch_arguments={
        'map':map_file,
        'params_file': params_file}.items(),

    )

    hotel_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),'launch', 'online_async_launch.py')
        ),
    )

    hotel_nav=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
        launch_arguments={
        'map':map_file,
        'params_file': params_file}.items(),

    )

    

    # Rviz2 bringup
    rviz=Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        # arguments=['-d',map_config]
        arguments=['-d',nav_config]
    )

    ld = LaunchDescription()

    ld.add_action(robot_bringup)
    #ld.add_action(robot_navigation_bringup)

    #ld.add_action(hotel_mapping)
    ld.add_action(rviz)
    ld.add_action(hotel_nav)

    return ld