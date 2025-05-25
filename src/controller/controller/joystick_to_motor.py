import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals

class JoystickToMotorCommand(Node):
    def __init__(self):
        super().__init__('joystick_to_motor_command')
        
        
        self.twist_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  
            self.twist_callback,
            10  # Queue size
        )
        
        
        self.motor_command_pub = self.create_publisher(
            MotorCommand,
            '/motor_command',  # Topic to send motor command
            10  # Queue size
        )
    
    def twist_callback(self, msg: Twist):
        # Extract linear and angular velocities from the Twist message
        linear_velocity = msg.linear.x  
        angular_velocity = msg.angular.z 
         
        # Scale joystick input to velocity range
        linear_velocity *= 250
        angular_velocity *= 250

        # Ensure velocity is a float
        angular_velocity = float(angular_velocity)
        linear_velocity = float(linear_velocity)

        
        # Create a MotorCommand message
        motor_command = MotorCommand()

        motor_command.linear_velocity = linear_velocity
        motor_command.angular_velocity = angular_velocity

        motor_command.is_pwm = False
        
        # Publish the motor command
        self.motor_command_pub.publish(motor_command)
        self.get_logger().info(f"Publishing motor command: {motor_command}")

def main(args=None):
    rclpy.init(args=args)
    
    node = JoystickToMotorCommand()
    
    # Spin the node to keep it running
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
