# In simple_arm/simple_arm/arm_hardware_interface.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from . import sts_driver
from std_srvs.srv import Trigger
import threading

class ArmHardwareInterface(Node):
    def __init__(self):
        super().__init__('arm_hardware_interface')

        self.motors_enabled = False
        self.system_primed = False
        # --- Joint Mappings ---
        self.single_servo_joints = {
            'base_link_Revolute-1': 1, 'link3_Revolute-5': 6,
            'link4_Revolute-6': 7, 'link5_Revolute-7': 8,
            'link6_Slider-8': 9
        }
        self.dual_servo_joints = {
            'link1_Revolute-3': [2, 3], 'link2_Revolute-4': [4, 5]
        }
        self.joint_names = list(self.single_servo_joints.keys()) + list(self.dual_servo_joints.keys())
        #self.joint_names.append('link7_Slider-9')
        # --- Calibration Data ---
        self.calibration = {
            'base_link_Revolute-1': {'scale': 644.9, 'offset': 3453},
            'link1_Revolute-3': {'scale': 651, 'offset': 3072},
            'link2_Revolute-4': {'scale': 647.2, 'offset': 985},
            'link3_Revolute-5': {'scale': 628, 'offset': 3112},
            'link4_Revolute-6': {'scale': 640, 'offset': 702},
            'link5_Revolute-7': {'scale': 654, 'offset': 99},
            'link6_Slider-8': {'scale': 14427, 'offset':2908}
        }

        self.joint_positions = [0.0] * len(self.joint_names)
        self.position_update_lock = threading.Lock()

        self.unwrapped_positions = {name: 0.0 for name in self.joint_names}
        self.revolutions = {name: 0 for name in self.joint_names}
        self.last_wrapped_angles = {name: None for name in self.joint_names}
        # Stores the last known tick value to detect wraparounds
        self.last_tick_values = {name: None for name in self.joint_names}

        # --- Parameters & Driver Initialization ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        try:
            self.driver = sts_driver.ServoController(self.serial_port, self.baudrate)
            self.get_logger().info(f"Successfully connected on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to servos: {e}")
            rclpy.shutdown(); return

        # --- ROS Communications ---
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_command_subscriber = self.create_subscription(JointState, 'joint_commands', self.command_callback, 10)
        self.enable_service = self.create_service(Trigger, 'enable_motors', self.enable_motors_callback)
        
        # --- FIX: Use a single, faster timer for reading from hardware and publishing the state ---
        self.read_and_publish_timer = self.create_timer(0.02, self.read_and_publish_states_callback) # Operate at 50Hz

        self.get_logger().info("Arm Hardware Interface started.")
        self.get_logger().info("RViz will sync with the real robot's position.")
        self.get_logger().info("Call /enable_motors service to allow physical movement.")

    def enable_motors_callback(self, request, response):
        self.motors_enabled = True
        self.system_primed = False
        self.get_logger().info("Motors have been enabled. Physical robot will now move.")
        response.success = True
        response.message = "Motors enabled."
        return response

    # --- Conversion Functions ---
    def _convert_rad_to_ticks(self, rad, joint_name):
        if joint_name == 'base_link_Revolute-1':
            # To move the joint by 'rad', the servo motor must move 3x that angle.
            rad *= 3.0

        if joint_name in self.calibration:
            cal = self.calibration[joint_name]
            return int(cal['offset'] + rad * cal['scale'])
        else:
            self.get_logger().warn(f"Using fallback calibration for {joint_name}", throttle_duration_sec=5)
            return int(2048 + (rad * (2048 / 3.14159)))

    def _convert_ticks_to_rad(self, ticks, joint_name):
        rad = 0.0 # Initialize
        if joint_name in self.calibration:
            cal = self.calibration[joint_name]
            rad = (ticks - cal['offset']) / cal['scale'] if cal['scale'] != 0 else 0.0
        else:
            # Fallback for uncalibrated joints
            rad = (ticks - 2048) * (3.14159 / 2048)
        
        # Apply the gearbox adjustment AFTER the initial conversion
        if joint_name == 'base_link_Revolute-1':
            rad /= 3.0

        return rad

    def _convert_gripper_dist_to_ticks(self, dist):
    # --- INSERT YOUR CALIBRATED VALUES HERE ---
        min_dist, max_dist = 0.0, 0.024
        min_ticks, max_ticks = 2903, 1518
        
        if max_ticks == min_ticks:
            self.get_logger().error("Gripper 'max_ticks' and 'min_ticks' cannot be the same value!", once=True)
            return min_ticks

        dist = max(min_dist, min(dist, max_dist))
        return int(min_ticks + ((dist - min_dist) / (max_dist - min_dist)) * (max_ticks - min_ticks))

    def _convert_ticks_to_gripper_dist(self, ticks):
    # --- INSERT YOUR CALIBRATED VALUES HERE ---
        min_dist, max_dist = 0.0, 0.024
        min_ticks, max_ticks = 2903, 1518
        
        if max_ticks == min_ticks:
            self.get_logger().error("Gripper 'max_ticks' and 'min_ticks' cannot be the same value!", once=True)
            return min_dist
        
        # --- FIX: This block correctly clamps the ticks even if the range is inverted ---
        low_tick = min(min_ticks, max_ticks)
        high_tick = max(min_ticks, max_ticks)
        ticks = max(low_tick, min(ticks, high_tick))
        
        # The mathematical formula works correctly with an inverted range
        position = min_dist + ((ticks - min_ticks) / (max_ticks - min_ticks)) * (max_dist - min_dist)
        position = max(min_dist, min(position, max_dist))
        return position

    # --- FIX: Renamed method that now handles both reading and publishing ---
    def read_and_publish_states_callback(self):
        """
        Reads servo positions, calculates a continuous unwrapped angle based on a
        calibrated zero, and publishes the state.
        """
        # This is the full angular range of the base joint after the gearbox
        # (4096 ticks / scale) / gear_ratio
        BASE_JOINT_RANGE = 4096 / self.calibration['base_link_Revolute-1']['scale'] / 3.0

        for name in self.joint_names:
            pos_ticks = None
            # Default to the last known unwrapped position if read fails
            unwrapped_pos = self.unwrapped_positions.get(name, 0.0)
            id_to_read = None
            # --- 1. Read Ticks ---
            if name in self.single_servo_joints:
                id_to_read = self.single_servo_joints[name]
                pos_ticks = self.driver.get_position(self.single_servo_joints[name])
            elif name in self.dual_servo_joints:
                id_to_read = self.dual_servo_joints[name][0] # Use the leader servo's ID
                pos_ticks = self.driver.get_position(self.dual_servo_joints[name][0])
            
            if id_to_read == 9:  # Check if the current servo is the gripper
                if pos_ticks is not None:
                    self.get_logger().info(
                        f"Servo ID {id_to_read} ('{name}') Ticks: {pos_ticks}",
                        throttle_duration_sec=1.0  # IMPORTANT: Prevents terminal spam
                    )

            if pos_ticks is None:
                continue # Use last known value if read fails

            # --- 2. Calculate the "wrapped" angle using your calibrated offset ---
            if name == 'link6_Slider-8':
                wrapped_pos = self._convert_ticks_to_gripper_dist(pos_ticks)
            else:
                wrapped_pos = self._convert_ticks_to_rad(pos_ticks, name)

            # --- 3. Initialize on first run ---
            if self.last_wrapped_angles[name] is None:
                self.last_wrapped_angles[name] = wrapped_pos
                unwrapped_pos = wrapped_pos # The first position is our baseline
            else:
                # --- 4. Detect and correct for wraparound on subsequent runs ---
                last_wrapped = self.last_wrapped_angles[name]
                diff = wrapped_pos - last_wrapped

                # Check for a jump larger than half the full range
                if name == 'base_link_Revolute-1':
                    if diff > (BASE_JOINT_RANGE / 2): # e.g., jump from 0.3 to -1.7 -> diff is large and negative
                        self.revolutions[name] -= 1
                    elif diff < (-BASE_JOINT_RANGE / 2): # e.g., jump from -1.7 to 0.3 -> diff is large and positive
                        self.revolutions[name] += 1
                
                # The final unwrapped position respects the calibrated zero AND tracks full rotations
                if name == 'base_link_Revolute-1':
                    unwrapped_pos = wrapped_pos + self.revolutions[name] * BASE_JOINT_RANGE
                else: # For other joints, we still use the basic wrapped position
                    unwrapped_pos = wrapped_pos
                
                self.last_wrapped_angles[name] = wrapped_pos

            # --- 5. Update state and log for debugging ---
            self.unwrapped_positions[name] = unwrapped_pos
            
            
        
        
        # --- 6. Publish final, smooth joint states ---
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = list(self.unwrapped_positions.keys())
        msg.position = list(self.unwrapped_positions.values())
        self.joint_state_publisher.publish(msg)

            
    def command_callback(self, msg: JointState):
        """Receives commands and intelligently calculates the shortest path to the target."""
        if not self.motors_enabled:
            return

        commands_to_sync = {}
        BASE_JOINT_RANGE = 4096 / self.calibration['base_link_Revolute-1']['scale'] / 3.0
        
        # The priming logic can be removed if the main logic is robust.
        # This simplifies the function.
        for i, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue

            target_wrapped_rad = msg.position[i]
            final_rad_for_conversion = target_wrapped_rad

            # --- FIX: New shortest-path command logic for the base link ---
            if name == 'base_link_Revolute-1':
                current_unwrapped_rad = self.unwrapped_positions[name]
                current_revolutions = self.revolutions[name]

                # Find the target position on the current, next, and previous revolution
                target_on_current_rev = target_wrapped_rad + current_revolutions * BASE_JOINT_RANGE
                target_on_next_rev = target_wrapped_rad + (current_revolutions + 1) * BASE_JOINT_RANGE
                target_on_prev_rev = target_wrapped_rad + (current_revolutions - 1) * BASE_JOINT_RANGE
                
                # Choose the target that is closest to our current unwrapped position
                final_rad_for_conversion = min(
                    [target_on_current_rev, target_on_next_rev, target_on_prev_rev],
                    key=lambda x: abs(x - current_unwrapped_rad)
                )

            # --- Convert the final calculated radian value to ticks ---
            if name == 'link6_Slider-8':
                position_ticks = self._convert_gripper_dist_to_ticks(final_rad_for_conversion)
            else:
                position_ticks = self._convert_rad_to_ticks(final_rad_for_conversion, name)

            # --- Assign ticks to servos (same as before) ---
            if name in self.single_servo_joints:
                servo_id = self.single_servo_joints[name]
                commands_to_sync[servo_id] = position_ticks
            elif name in self.dual_servo_joints:
                leader_id, follower_id = self.dual_servo_joints[name]
                commands_to_sync[leader_id] = position_ticks
                commands_to_sync[follower_id] = 4095 - position_ticks
        
        if commands_to_sync:
            self.driver.sync_write_positions(commands_to_sync)

    # --- FIX: The old 'publish_states_callback' is no longer needed and should be deleted ---

    def destroy_node(self):
        """Called automatically on shutdown to clean up resources."""
        self.get_logger().info("Shutdown signal received. Disabling torque on all servos.")
        
        all_servo_ids = set(self.single_servo_joints.values())
        for ids in self.dual_servo_joints.values():
            all_servo_ids.update(ids)
            
        for servo_id in all_servo_ids:
            try:
                self.driver.set_torque_enable(servo_id, False)
            except Exception as e:
                self.get_logger().error(f"Failed to disable torque for servo {servo_id}: {e}")
        
        self.driver.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmHardwareInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()