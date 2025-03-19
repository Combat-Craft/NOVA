import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('encoder_cpr', default_value='500', description='Encoder CPR'),
        DeclareLaunchArgument('loop_rate', default_value='50', description='Loop rate'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0', description='Serial port'),
        DeclareLaunchArgument('baud_rate', default_value='57600', description='Baud rate'),
        DeclareLaunchArgument('serial_debug', default_value='False', description='Enable serial debugging'),

        # Node action to launch motor_driver with parameters
        Node(
            package='controller',  
            executable='motor_driver',  
            name='motor_driver',
            output='screen',
            parameters=[{
                'encoder_cpr': 766,  
                'loop_rate': 30,     
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 57600,
                'serial_debug': False
            }],
        ),
    ])
