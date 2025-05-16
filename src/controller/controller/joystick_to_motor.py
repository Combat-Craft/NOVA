import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from serial_motor_demo_msgs.msg import MotorCommand


class JoystickToMotor(Node):

    def __init__(self):
        super().__init__('joystick_to_motor')

        # Declare parameters
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('max_velocity', 250)
        self.declare_parameter('turbo_button', 5)
        self.declare_parameter('turbo_scale', 2.0)  # Turbo scale factor

        # Get parameters
        self.deadzone = self.get_parameter('deadzone').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.turbo_button = self.get_parameter('turbo_button').value
        self.turbo_scale = self.get_parameter('turbo_scale').value

        # Log parameters for debugging
        self.get_logger().info(f"Deadzone: {self.deadzone}, Max Velocity: {self.max_velocity}, "
                               f"Turbo Button: {self.turbo_button}, Turbo Scale: {self.turbo_scale}")

        # Setup Joystick Subscription
        self.joystick_sub = self.create_subscription(
            Joy,
            '/joy',  # Topic for joystick input
            self.joystick_callback,
            10
        )

        # Setup Motor Command Publisher
        self.motor_pub = self.create_publisher(MotorCommand, 'motor_command', 10)

    def joystick_callback(self, msg):
        # Extract joystick axes for control
        linear_velocity = msg.axes[1]  # Left stick vertical axis (forward/backward)
        angular_velocity = msg.axes[0]  # Right stick horizontal axis (turning left/right)

        # Apply deadzone
        if abs(linear_velocity) < self.deadzone:
            linear_velocity = 0
        if abs(angular_velocity) < self.deadzone:
            angular_velocity = 0

        # Reverse angular direction for consistency
        angular_velocity = -angular_velocity

        # Check if the turbo button is pressed
        turbo_active = msg.buttons[self.turbo_button]

        # Adjust max velocity for turbo mode
        max_velocity = self.max_velocity * (self.turbo_scale if turbo_active else 1)

        # Scale joystick input to velocity range
        linear_velocity *= max_velocity
        angular_velocity *= max_velocity

        # Ensure angular_velocity is a float
        angular_velocity = float(angular_velocity)
        linear_velocity = float(linear_velocity)

        # Create and publish the MotorCommand message
        motor_command = MotorCommand()
        motor_command.is_pwm = False
        motor_command.linear_velocity = linear_velocity
        motor_command.angular_velocity = angular_velocity

        self.motor_pub.publish(motor_command)


def main(args=None):
    rclpy.init(args=args)

    joystick_to_motor = JoystickToMotor()

    # Spin to keep the node alive and handle subscriptions
    rclpy.spin(joystick_to_motor)

    joystick_to_motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()