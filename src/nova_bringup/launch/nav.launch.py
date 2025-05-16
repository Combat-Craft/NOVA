import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_project_bringup = get_package_share_directory('nova_bringup')
    pkg_project_gazebo = get_package_share_directory('nova_gazebo')
    pkg_project_description = get_package_share_directory('nova_description')
    
    #find the world
    world_path=os.path.join(pkg_project_gazebo, 'worlds/my_world.sdf')
    
    #bring up the robot
    xacro_file = os.path.join(pkg_project_description,'urdf','nova_gps.urdf')
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_config}
    nav2_params = os.path.join(pkg_project_bringup, 'config', 'josh_default_params.yaml')

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'gazebo.launch.py'))
        )
    
     # Static Transform Publisher: Publish transform between odom and base_link
    odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ),
        launch_arguments={'use_sim_time': 'True'}.items() 
    )

    nav_stack_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'True',
                          'params_file': nav2_params}.items()
    )

    costmap = Node(
        package='nav2_costmap_2d',
        executable = 'nav2_costmap_2d_markers',
        output='screen',
        parameters=[{'voxel_grid': '/local_costmap/voxel_grid'},
                    {'visualization_marker': '/my_marker'}]

    )

    rviz_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py'))
    )

    return launch.LaunchDescription([
       robot_bringup,
       odom_to_base_link, 
       TimerAction(
            period=2.0,  # Add a 2-second delay to allow time for transforms to initialize
            actions=[slam_bringup, nav_stack_bringup, rviz_bringup]
        ),
        costmap
    ])