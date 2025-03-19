from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    joy_params = os.path.join(get_package_share_directory('controller'), 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',  
        executable='joy_node',  
        parameters=[joy_params],  
    )

    teleop_node = Node(
        package='teleop_twist_joy',  
        executable='teleop_node', 
        name='teleop_node',  
        parameters=[joy_params], 
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    joystick_to_motor_node = Node(
        package='controller',  # Name of your package
        executable='joystick_to_motor',  # This is the new script for converting Twist -> MotorCommand
        name='joystick_to_motor',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/cmd_vel'),  # Assuming /cmd_vel is the correct input topic
                    ('/motor_command', '/motor_command')]  # Output topic for motor commands
    )
      
    return LaunchDescription([
        joy_node,
        teleop_node,
        joystick_to_motor_node
              
    ])
