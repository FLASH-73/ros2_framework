import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from .sts_driver import FeetechMotorsBus  # Your driver
import numpy as np
import math
import time
import threading

class ArmDriverNode(Node):
    def __init__(self):
        super().__init__('arm_driver_node')

        # Motors config (from your code)
        self.motors_config = {
            "base": (1, "sts3215"),
            "shoulder_leader": (2, "sts3215"),
            "shoulder_follower": (3, "sts3215"),
            "elbow_leader": (4, "sts3215"),
            "elbow_follower": (5, "sts3215"),
            "link3": (6, "sts3215"),
            "link4": (7, "sts3215"),
            "link5": (8, "sts3215"),
            "gripper": (9, "sts3215"),
        }
        self.motor_names = list(self.motors_config.keys())
        self.motor_ids = [conf[0] for conf in self.motors_config.values()]
        self.motor_models = [conf[1] for conf in self.motors_config.values()]

        # Joint mappings
        self.single_servo_joints = {
            "base_link_Revolute-1": 1,
            "link3_Revolute-5": 6,
            "link4_Revolute-6": 7,
            "link5_Revolute-7": 8,
            "link6_Slider-8": 9,
        }
        self.dual_servo_joints = {
            "link1_Revolute-3": [2, 3],  # Leader, follower
            "link2_Revolute-4": [4, 5],
        }

        # Calibration
        self.calibration = {
            "base_link_Revolute-1": (644.9, 3453),
            "link1_Revolute-3": (651, 3072),
            "link2_Revolute-4": (647.2, 985),
            "link3_Revolute-5": (628, 3112),
            "link4_Revolute-6": (640, 702),
            "link5_Revolute-7": (654, 99),
            "link6_Slider-8": (14427, 2908),
        }

        # Joint names (from URDF)
        self.joint_names = [
            "base_link_Revolute-1", "link1_Revolute-3", "link2_Revolute-4",
            "link3_Revolute-5", "link4_Revolute-6", "link5_Revolute-7", "link6_Slider-8"
        ]
        self.hw_positions = np.zeros(len(self.joint_names))
        self.revolutions = {name: 0 for name in self.joint_names}
        self.last_wrapped_angles = {name: None for name in self.joint_names}

        # Initialize driver
        serial_port = self.declare_parameter('serial_port', '/dev/ttyUSB0').value
        self.driver = FeetechMotorsBus(port=serial_port, motors=self.motors_config)
        self.driver.connect()
        self.get_logger().info('Connected to motors.')

        # Enable torque
        self.driver.write("Torque_Enable", [1] * len(self.motor_names), self.motor_names)

        # Action server for FollowJointTrajectory
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',  # Change if needed
            execute_callback=self.execute_trajectory_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Publisher for /joint_states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.state_timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_trajectory_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        feedback = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        for point in trajectory.points:
            # Execute point (convert positions to ticks and write)
            self.execute_point(point.positions)

            # Feedback (publish current positions)
            feedback.desired.positions = point.positions
            goal_handle.publish_feedback(feedback)

            # Sleep for duration if timed
            if point.time_from_start.sec > 0:
                time.sleep(point.time_from_start.sec + point.time_from_start.nanosec / 1e9)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def execute_point(self, positions_rad):
        commands_to_sync = {}  # {servo_id: ticks}

        positions_rad = list(positions_rad)  # Convert to list for modification
        if len(positions_rad) == 6:  # Arm-only trajectory; append current gripper position
            gripper_idx = self.joint_names.index("link6_Slider-8")
            positions_rad.append(self.hw_positions[gripper_idx])
            self.get_logger().info("Arm-only trajectory received; keeping gripper at current position.")

        if len(positions_rad) != len(self.joint_names):
            self.get_logger().error(f"Mismatched positions length: expected {len(self.joint_names)}, got {len(positions_rad)}")
            return  # Abort to prevent errors

        for i, name in enumerate(self.joint_names):
            cmd_rad = positions_rad[i]
            final_rad = cmd_rad

            # Shortest path for base
            if name == "base_link_Revolute-1":
                current_unwrapped = self.hw_positions[i]
                delta = cmd_rad - current_unwrapped
                final_rad = current_unwrapped + (delta + math.pi) % (2 * math.pi) - math.pi

            # Convert to ticks
            if name == "link6_Slider-8":
                position_ticks = self._gripper_dist_to_ticks(final_rad)
            else:
                position_ticks = self._rad_to_ticks(final_rad, name)

            # Map to servos
            if name in self.single_servo_joints:
                servo_id = self.single_servo_joints[name]
                commands_to_sync[servo_id] = position_ticks
            elif name in self.dual_servo_joints:
                ids = self.dual_servo_joints[name]
                commands_to_sync[ids[0]] = position_ticks
                commands_to_sync[ids[1]] = 4095 - position_ticks

        # Write to hardware
        if commands_to_sync:
            ids_to_write = list(commands_to_sync.keys())
            values_to_write = list(commands_to_sync.values())
            models_to_write = ["sts3215"] * len(ids_to_write)
            self.driver.write_with_motor_ids(models_to_write, ids_to_write, "Goal_Position", values_to_write)

    def publish_joint_states(self):
        # Read positions from hardware
        ticks = self.driver.read("Present_Position", self.motor_names)
        ticks_by_id = dict(zip(self.motor_ids, ticks))

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = [0.0] * len(self.joint_names)

        for i, name in enumerate(self.joint_names):
            position_rad = 0.0
            if name == "link6_Slider-8":
                ticks_val = ticks_by_id[self.single_servo_joints[name]]
                position_rad = self._ticks_to_gripper_dist(ticks_val)
            elif name in self.single_servo_joints:
                ticks_val = ticks_by_id[self.single_servo_joints[name]]
                position_rad = self._ticks_to_rad(ticks_val, name)
            elif name in self.dual_servo_joints:
                ids = self.dual_servo_joints[name]
                leader_ticks = ticks_by_id[ids[0]]
                follower_ticks = ticks_by_id[ids[1]]
                leader_rad = self._ticks_to_rad(leader_ticks, name)
                follower_rad = self._ticks_to_rad(4095 - follower_ticks, name)
                position_rad = (leader_rad + follower_rad) / 2.0

            # Unwrap base
            if name == "base_link_Revolute-1":
                if self.last_wrapped_angles[name] is not None:
                    delta = position_rad - self.last_wrapped_angles[name]
                    if delta > math.pi:
                        self.revolutions[name] -= 1
                    elif delta < -math.pi:
                        self.revolutions[name] += 1
                unwrapped_pos = position_rad + self.revolutions[name] * 2 * math.pi
                self.last_wrapped_angles[name] = position_rad
                self.hw_positions[i] = unwrapped_pos
            else:
                self.hw_positions[i] = position_rad

            joint_state.position[i] = self.hw_positions[i]

        self.joint_state_pub.publish(joint_state)

    # Helper functions (copy from your python_hw_inter.py)
    def _rad_to_ticks(self, rad, joint_name):
        scale, offset = self.calibration.get(joint_name, (2048 / math.pi, 2048))
        if joint_name == "base_link_Revolute-1":
            rad *= 3.0
        return int(offset + rad * scale)

    def _ticks_to_rad(self, ticks, joint_name):
        scale, offset = self.calibration.get(joint_name, (2048 / math.pi, 2048))
        rad = (ticks - offset) / scale
        if joint_name == "base_link_Revolute-1":
            rad /= 3.0
        return rad

    def _gripper_dist_to_ticks(self, dist):
        min_dist, max_dist = 0.0, 0.024
        min_ticks, max_ticks = 2903, 1518
        dist = max(min_dist, min(dist, max_dist))
        return int(min_ticks + ((dist - min_dist) / (max_dist - min_dist)) * (max_ticks - min_ticks))

    def _ticks_to_gripper_dist(self, ticks):
        min_dist, max_dist = 0.0, 0.024
        min_ticks, max_ticks = 2903, 1518
        low_tick, high_tick = min(min_ticks, max_ticks), max(min_ticks, max_ticks)
        ticks = max(low_tick, min(ticks, high_tick))
        position = min_dist + ((ticks - min_ticks) / (max_ticks - min_ticks)) * (max_dist - min_dist)
        return max(min_dist, min(position, max_dist))

def main(args=None):
    rclpy.init(args=args)
    node = ArmDriverNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()