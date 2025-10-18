import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
import time
import math

class TestAngleClient(Node):
    def __init__(self):
        super().__init__('test_angle_client')
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def send_large_angle(self, angle_rad):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return

        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['base_link_Revolute-1']  # Only base
        point = JointTrajectoryPoint()
        point.positions = [angle_rad]  # Send large rad (e.g., 6*math.pi ~1080° arm, ~3240°/12288 ticks servo)
        point.time_from_start.sec = 5  # Slow move for safety
        trajectory.points = [point]
        goal.trajectory = trajectory

        self.get_logger().info(f'Sending goal: {angle_rad} rad to base')
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

def main():
    rclpy.init()
    client = TestAngleClient()

    # Temporarily enable torque on base for test (add to your node init or run separately)
    # self.driver.write("Torque_Enable", [1], ["base"])  # Uncomment in arm_driver_node if needed

    # Test small angle first
    client.send_large_angle(2 * math.pi)  # ~1 arm turn, ~3 servo turns / ~12288 ticks
    time.sleep(6)  # Wait for move

    # Test larger
    #client.send_large_angle(4 * math.pi)  # ~2 arm turns, should go to ~24576 ticks if multi-turn works (but clip to ±28672)

    # Monitor /joint_states or your print(f"Base raw ticks: ...") for continuous increase without jump
    # After test, disable torque if needed: self.driver.write("Torque_Enable", [0], ["base"])

    rclpy.shutdown()

if __name__ == '__main__':
    main()