#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64  # For gripper if needed

class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')
        self.declare_parameter('linear_scale', 0.3)  # Matches servo.yaml linear_scale
        self.declare_parameter('angular_scale', 0.5)  # Matches servo.yaml angular_scale
        self.declare_parameter('gripper_increment', 0.005)  # Small steps for gripper open/close
        self.declare_parameter('gripper_min', 0.0)
        self.declare_parameter('gripper_max', 0.024)
        self.declare_parameter('ee_frame', 'gripper_ee')  # Matches your servo.yaml ee_frame_name

        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.gripper_increment = self.get_parameter('gripper_increment').value
        self.gripper_min = self.get_parameter('gripper_min').value
        self.gripper_max = self.get_parameter('gripper_max').value
        self.ee_frame = self.get_parameter('ee_frame').value

        self.current_gripper_pos = self.gripper_min  # Start assuming open

        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)  # For unitless deltas (relative small moves)
        self.gripper_pub = self.create_publisher(Float64, '/gripper_controller/commands', 10)  # Direct position command for gripper_controller (prismatic)

        self.get_logger().info('Joy to Servo node ready. Use PS4 axes for arm Twist (delta), buttons for gripper.')

    def joy_callback(self, msg):
        # PS4 mapping (adjust if needed; typical: axes[0]=left_x, axes[1]=left_y, axes[3]=right_x, axes[4]=right_y, axes[2]=L2, axes[5]=R2)
        # Buttons: [0]=X, [1]=O, [2]=□, [3]=△, [4]=L1, [5]=R1, [6]=L2_btn, [7]=R2_btn, [8]=share, [9]=options, [10]=PS, [11]=left_stick_btn, [12]=right_stick_btn
        # Dpad: axes[6]=left/right (-1/1), axes[7]=up/down (-1/1) but often hats as buttons

        # Map to Twist: linear x (forward/back): left_y (axes[1]), y (left/right): left_x (axes[0]), z (up/down): L2/R2 (axes[2]/5, scale)
        # Angular: x (roll): right_y (axes[4]), y (pitch): right_x (axes[3]), z (yaw): dpad left/right or L1/R1 buttons
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.ee_frame  # EE frame for Cartesian deltas

        # Linear (unitless deltas: small relative moves, scaled by servo params)
        twist.twist.linear.x = msg.axes[1] * self.linear_scale  # Forward/back
        twist.twist.linear.y = -msg.axes[0] * self.linear_scale  # Left/right (invert if needed)
        twist.twist.linear.z = (msg.axes[5] - msg.axes[2]) * self.linear_scale / 2.0  # Up/down with triggers (R2 down, L2 up; scale half for sensitivity)

        # Angular
        twist.twist.angular.x = msg.axes[4] * self.angular_scale  # Roll (right_y)
        twist.twist.angular.y = msg.axes[3] * self.angular_scale  # Pitch (right_x)
        twist.twist.angular.z = (msg.buttons[4] - msg.buttons[5]) * self.angular_scale  # Yaw with L1/R1 (left positive, right negative; buttons are 0/1)

        # Publish only if non-zero (to avoid constant small noise)
        if any(abs(v) > 0.01 for v in [twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z,
                                       twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z]):
            self.twist_pub.publish(twist)

        # Gripper: Use buttons, e.g., X (0) to close increment, O (1) to open increment
        if msg.buttons[0] == 1:  # X: close
            self.current_gripper_pos = min(self.current_gripper_pos + self.gripper_increment, self.gripper_max)
            self.publish_gripper()
        if msg.buttons[1] == 1:  # O: open
            self.current_gripper_pos = max(self.current_gripper_pos - self.gripper_increment, self.gripper_min)
            self.publish_gripper()

    def publish_gripper(self):
        gripper_cmd = Float64()
        gripper_cmd.data = self.current_gripper_pos
        self.gripper_pub.publish(gripper_cmd)
        self.get_logger().info(f'Gripper command sent: {self.current_gripper_pos:.3f} m')

def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()