import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
# π0 imports
from openpi.pi_zero import PiZeroModel  # Adjust per repo
import torch
import numpy as np  # For clipping/scaling

class Pi0ControlNode(Node):
    def __init__(self):
        super().__init__('pi0_control_node')
        self.model = PiZeroModel.load_from_checkpoint('path/to/pi0_checkpoint.pth')
        self.model.eval()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)

        # Arm-specific config (from your joint_limits.yaml/URDF)
        self.joint_names = ['base_link_Revolute-1', 'link1_Revolute-3', 'link2_Revolute-4', 
                            'link3_Revolute-5', 'link4_Revolute-6', 'link5_Revolute-7', 'link6_Slider-8']
        self.min_limits = [-np.inf, -3.14, -3.14, -3.14, -3.14, -3.14, 0.0]  # Revolute ±π, base unlimited, gripper 0-0.024m
        self.max_limits = [np.inf, 3.14, 3.14, 3.14, 3.14, 3.14, 0.024]  # Customize from your limits
        self.action_dim = 7  # Matches your DOF

        # Subscribers
        self.gripper_cam_sub = self.create_subscription(Image, '/d405/color/image_raw', self.gripper_image_cb, 10)
        self.stationary_cam_sub = self.create_subscription(Image, '/d435/color/image_raw', self.stationary_image_cb, 10)
        self.prompt_sub = self.create_subscription(String, '/vla_prompt', self.prompt_cb, 10)

        # Publisher
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # State
        self.bridge = CvBridge()
        self.current_gripper_img = None
        self.current_stationary_img = None
        self.current_prompt = "idle"

        self.timer = self.create_timer(0.1, self.inference_loop)

    def gripper_image_cb(self, msg):
        self.current_gripper_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def stationary_image_cb(self, msg):
        self.current_stationary_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def prompt_cb(self, msg):
        self.current_prompt = msg.data
        self.get_logger().info(f'New prompt: {self.current_prompt}')

    def map_ai_to_arm_actions(self, raw_actions):
        """Custom mapping: Denormalize π0 outputs to your arm's space."""
        # Assume π0 outputs normalized [-1,1] vector of size 7 (your DOF)
        if len(raw_actions) != self.action_dim:
            self.get_logger().warn(f"Action dim mismatch: Expected {self.action_dim}, got {len(raw_actions)}")
            return None

        # Step 1: Denormalize to physical units (linear scale from model range)
        physical_actions = np.zeros(self.action_dim)
        for i in range(self.action_dim):
            if self.min_limits[i] == -np.inf or self.max_limits[i] == np.inf:  # Continuous like base
                physical_actions[i] = raw_actions[i] * np.pi  # Arbitrary scale, calibrate
            else:  # Bounded joints
                physical_actions[i] = self.min_limits[i] + (raw_actions[i] + 1.0) * (self.max_limits[i] - self.min_limits[i]) / 2.0

        # Step 2: Clip to limits/safety
        physical_actions = np.clip(physical_actions, self.min_limits, self.max_limits)

        # Step 3: Optional IK/unwrap (e.g., for base continuous: add to unwrapped_pos)
        # physical_actions[0] += self.unwrapped_base_pos  # From your hardware (track in node)

        # Step 4: Gripper-specific (e.g., if model outputs [0,1] for open/close)
        # physical_actions[6] = physical_actions[6] * 0.024  # Scale to m

        self.get_logger().info(f"Mapped AI actions: {physical_actions}")
        return physical_actions.tolist()

    def inference_loop(self):
        if self.current_gripper_img is not None and self.current_stationary_img is not None and self.current_prompt != "idle":
            # Preprocess images (e.g., resize to 224x224 for π0)
            gripper_tensor = torch.from_numpy(self.current_gripper_img).permute(2, 0, 1).unsqueeze(0).float() / 255.0
            stationary_tensor = torch.from_numpy(self.current_stationary_img).permute(2, 0, 1).unsqueeze(0).float() / 255.0
            gripper_tensor = gripper_tensor.to(self.device)
            stationary_tensor = stationary_tensor.to(self.device)

            # π0 inference
            with torch.no_grad():
                raw_actions = self.model.infer(
                    prompt=self.current_prompt,
                    gripper_image=gripper_tensor,
                    stationary_image=stationary_tensor,
                    num_steps=1  # Single step for now; loop for multi
                )  # Outputs: e.g., np.array of shape (1, 7) for actions

            # Map to your arm
            arm_actions = self.map_ai_to_arm_actions(raw_actions[0])  # Flatten to list
            if arm_actions is None:
                return

            # Publish as trajectory
            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = arm_actions
            point.velocities = [0.0] * self.action_dim  # Or compute from deltas
            point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s step
            traj.points = [point]
            self.traj_pub.publish(traj)
            self.get_logger().info(f'Published mapped actions for: {self.current_prompt}')

def main(args=None):
    rclpy.init(args=args)
    node = Pi0ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()