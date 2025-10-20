import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from threading import Lock
import numpy as np
import math
import time

from .sts_driver import FeetechMotorsBus

class ArmDriverNode(Node):
    def __init__(self):
        super().__init__('arm_driver_node')

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
        self.default_speed = 500

        self.single_servo_joints = {
            "base_link_Revolute-1": 1,
            "link3_Revolute-5": 6,
            "link4_Revolute-6": 7,
            "link5_Revolute-7": 8,
            "link6_Slider-8": 9,
        }
        self.dual_servo_joints = {
            "link1_Revolute-3": [2, 3],
            "link2_Revolute-4": [4, 5],
        }

        self.calibration = {
            "base_link_Revolute-1": (644.9, 1190),
            "link1_Revolute-3": (651, 3072),
            "link2_Revolute-4": (647.2, 985),
            "link3_Revolute-5": (628, 3112),
            "link4_Revolute-6": (640, 702),
            "link5_Revolute-7": (654, 2480),
            "link6_Slider-8": (14427, 3500),
        }

        self.joint_names = [
            "base_link_Revolute-1", "link1_Revolute-3", "link2_Revolute-4",
            "link3_Revolute-5", "link4_Revolute-6", "link5_Revolute-7", "link6_Slider-8"
        ]
        self.hw_positions = np.zeros(len(self.joint_names))
        self.lock = Lock()

        serial_port = '/dev/ttyUSB0'
        self.driver = FeetechMotorsBus(port=serial_port, motors=self.motors_config)
        self.driver.connect()

        # Reset base servo to default values
        self.driver.write("CW_Dead_Zone", [0], ["base"])
        self.driver.write("CCW_Dead_Zone", [0], ["base"])
        self.driver.write("P_Coefficient", [32], ["base"])
        self.driver.write("D_Coefficient", [32], ["base"])
        self.driver.write("I_Coefficient", [0], ["base"])
        self.driver.write("Protective_Torque", [20], ["base"])
        self.driver.write("Protection_Time", [200], ["base"])
        self.driver.write("Acceleration", [0], ["base"])
        self.driver.write("Min_Angle_Limit", [0], ["base"])
        self.driver.write("Max_Angle_Limit", [4095], ["base"])
        self.driver.write("Angular_Resolution", [1], ["base"])
        self.driver.write("Response_Status_Level", [2], ["base"])
        self.driver.write("Mode", [0], ["base"])
        self.driver.write("Lock", [1], ["base"])

        self.driver.write("Torque_Enable", [0] * len(self.motor_names), self.motor_names)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_trajectory_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.gripper_action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            execute_callback=self.execute_gripper_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.state_timer = self.create_timer(0.02, self.publish_joint_states)

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.driver.write("Torque_Enable", [0] * len(self.motor_names), self.motor_names)
        time.sleep(0.1)
        self.driver.write("Torque_Enable", [1] * len(self.motor_names), self.motor_names)
        return CancelResponse.ACCEPT

    async def execute_trajectory_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        feedback = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        for point in trajectory.points:
            self.execute_point(point.positions, trajectory.joint_names)
            feedback.desired.positions = point.positions
            goal_handle.publish_feedback(feedback)
            duration = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            if duration > 0:
                time.sleep(duration)
            else:
                time.sleep(0.05)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    async def execute_gripper_callback(self, goal_handle):
        gripper_goal = goal_handle.request.command.position
        self.execute_point([gripper_goal], ["link6_Slider-8"])
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = gripper_goal
        return result

    def execute_point(self, positions_rad, joint_names=None):
        if joint_names is None:
            joint_names = self.joint_names

        commands_to_sync = {}
        positions_rad = list(positions_rad)
        try:
            for name, pos in zip(joint_names, positions_rad):
                if name == "link6_Slider-8":
                    ticks = self._gripper_dist_to_ticks(pos)
                else:
                    ticks = self._rad_to_ticks(pos, name)

                if name in self.single_servo_joints:
                    commands_to_sync[self.single_servo_joints[name]] = ticks
                elif name in self.dual_servo_joints:
                    ids = self.dual_servo_joints[name]
                    commands_to_sync[ids[0]] = ticks
                    commands_to_sync[ids[1]] = 4095 - ticks

            if commands_to_sync:
                ids = list(commands_to_sync.keys())
                models = ["sts3215"] * len(ids)
                with self.lock:
                    self.driver.write_with_motor_ids(models, ids, "Goal_Speed", [self.default_speed] * len(ids))
                    self.driver.write_with_motor_ids(models, ids, "Goal_Position", list(commands_to_sync.values()))

        except Exception as e:
            self.get_logger().error(f"Error in execute_point: {str(e)}")

    def publish_joint_states(self):
        with self.lock:
            ticks = self.driver.read("Present_Position", self.motor_names)
        if ticks is None or len(ticks) != len(self.motor_names):
            self.get_logger().warn("Failed to read positions")
            return

        ticks_by_id = dict(zip(self.motor_ids, ticks))

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = [0.0] * len(self.joint_names)

        for i, name in enumerate(self.joint_names):
            #if name == "base_link_Revolute-1":
                #print(f"Base raw ticks: {ticks_by_id[self.single_servo_joints[name]]}")
            if name == "link6_Slider-8":
                ticks_val = ticks_by_id[self.single_servo_joints[name]]
                pos = self._ticks_to_gripper_dist(ticks_val)
            elif name in self.single_servo_joints:
                ticks_val = ticks_by_id[self.single_servo_joints[name]]
                pos = self._ticks_to_rad(ticks_val, name)
            elif name in self.dual_servo_joints:
                ids = self.dual_servo_joints[name]
                leader = self._ticks_to_rad(ticks_by_id[ids[0]], name)
                follower = self._ticks_to_rad(4095 - ticks_by_id[ids[1]], name)
                pos = (leader + follower) / 2.0
            
            self.hw_positions[i] = pos

            joint_state.position[i] = self.hw_positions[i]

        self.joint_state_pub.publish(joint_state)

    def _rad_to_ticks(self, rad, joint_name):
        scale, offset = self.calibration.get(joint_name, (2048 / math.pi, 2048))
        return int(offset + rad * scale)

    def _ticks_to_rad(self, ticks, joint_name):
        scale, offset = self.calibration.get(joint_name, (2048 / math.pi, 2048))
        rad = (ticks - offset) / scale
        return rad

    def _gripper_dist_to_ticks(self, dist):
        min_dist, max_dist = 0.0, 0.024
        min_ticks, max_ticks = 2903, 1518
        dist = max(min_dist, min(dist, max_dist))
        return int(min_ticks + ((dist - min_dist) / (max_dist - min_dist)) * (max_ticks - min_ticks))

    def _ticks_to_gripper_dist(self, ticks):
        min_dist, max_dist = 0.0, 0.024
        min_ticks, max_ticks = 2903, 1518
        ticks = max(min_ticks, min(ticks, max_ticks))
        return min_dist + ((ticks - min_ticks) / (max_ticks - min_ticks)) * (max_dist - min_dist)

def main(args=None):
    rclpy.init(args=args)
    node = ArmDriverNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()