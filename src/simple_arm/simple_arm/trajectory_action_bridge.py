# simple_arm/src/trajectory_action_bridge.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory

class TrajectoryActionBridge(Node):
    def __init__(self):
        super().__init__('trajectory_action_bridge')
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.traj_sub = self.create_subscription(
            JointTrajectory, '/arm_controller/joint_trajectory', self.traj_callback, 10
        )
        self.get_logger().info('Bridge node started, waiting for trajectories...')

    def traj_callback(self, msg: JointTrajectory):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available!')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg
        # Add small duration if not set (for "instant" feel)
        if not goal.trajectory.points:
            self.get_logger().warn('Empty trajectory received!')
            return
        for point in goal.trajectory.points:
            if point.time_from_start.sec == 0 and point.time_from_start.nanosec == 0:
                point.time_from_start.nanosec = 50000000  # 0.05s for quick move

        self.action_client.send_goal_async(goal)
        self.get_logger().info('Sent trajectory goal to action server')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryActionBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()