import rclpy
from rclpy.logging import get_logger
import math
import numpy as np

from hardware_interface import SystemInterface, CallbackReturn, HardwareInfo
from hardware_interface.types.hardware_interface_type_values import (
    HardwareInterfaceTypeValues as HW,
)

from sts_driver import FeetechMotorsBus # Assuming sts_driver.py is in the same package or python path

class ArmHardware(SystemInterface):
    """
    Python hardware interface for the Mark II robotic arm.
    This class translates the logic from the C++ version to be compatible with
    the provided Python FeetechMotorsBus driver.
    """

    def on_init(self, info: HardwareInfo) -> CallbackReturn:
        """
        This method is called upon loading the hardware interface plugin.
        It initializes the connection to the motor bus and sets up joint configurations.
        """
        if super().on_init(info) != CallbackReturn.SUCCESS:
            return CallbackReturn.ERROR

        self.logger = get_logger("ArmHardware")

        # -- Hardware Configuration --
        # Define all 9 physical servos that the FeetechMotorsBus will control.
        # We give each a unique name and specify its model.
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


        # -- Mappings from ros2_control joints to physical servos --
        # These map the joint names from your URDF to the servo IDs.
        self.single_servo_joints = {
            "base_link_Revolute-1": 1,
            "link3_Revolute-5": 6,
            "link4_Revolute-6": 7,
            "link5_Revolute-7": 8,
            "link6_Slider-8": 9,
        }
        self.dual_servo_joints = {
            "link1_Revolute-3": {"leader": 2, "follower": 3},
            "link2_Revolute-4": {"leader": 4, "follower": 5},
        }

        # -- Calibration Data --
        # This data is translated directly from your C++ code.
        self.calibration = {
            "base_link_Revolute-1": (644.9, 3453),
            "link1_Revolute-3": (651, 3072),
            "link2_Revolute-4": (647.2, 985),
            "link3_Revolute-5": (628, 3112),
            "link4_Revolute-6": (640, 702),
            "link5_Revolute-7": (654, 99),
            "link6_Slider-8": (14427, 2908), # Note: This is for the gripper, which has its own conversion logic.
        }

        # -- State and Command Storage --
        # Store joint names from the URDF.
        self.joint_names = [j.name for j in self.info.joints]
        num_joints = len(self.joint_names)
        self.hw_commands = np.zeros(num_joints)
        self.hw_positions = np.zeros(num_joints)
        
        # Data for unwrapping continuous joint angle (base joint)
        self.revolutions = {name: 0 for name in self.joint_names}
        self.last_wrapped_angles = {name: None for name in self.joint_names}

        # -- Driver Initialization --
        try:
            serial_port = self.info.hardware_parameters.get("serial_port", "/dev/ttyUSB0")
            # The python driver uses a hardcoded baudrate of 1,000,000
            self.logger.info(f"Connecting to motors on port {serial_port}...")
            self.driver = FeetechMotorsBus(port=serial_port, motors=self.motors_config)
            self.driver.connect()
            self.logger.info("Successfully connected to motors.")
        except Exception as e:
            self.logger.fatal(f"Failed to initialize motor driver: {e}")
            return CallbackReturn.ERROR

        # -- Interface Validation --
        # Check that the URDF is configured for a position-only interface.
        for joint in self.info.joints:
            if len(joint.command_interfaces) != 1 or joint.command_interfaces[0].name != HW.POSITION:
                self.logger.fatal(f"Joint '{joint.name}' has an invalid command interface configuration.")
                return CallbackReturn.ERROR
            if len(joint.state_interfaces) != 1 or joint.state_interfaces[0].name != HW.POSITION:
                self.logger.fatal(f"Joint '{joint.name}' has an invalid state interface configuration.")
                return CallbackReturn.ERROR

        return CallbackReturn.SUCCESS

    def export_state_interfaces(self):
        """Exports one state interface (position) for each joint."""
        return [
            (f"{name}", HW.POSITION, self.hw_positions[i])
            for i, name in enumerate(self.joint_names)
        ]

    def export_command_interfaces(self):
        """Exports one command interface (position) for each joint."""
        return [
            (f"{name}", HW.POSITION, self.hw_commands[i])
            for i, name in enumerate(self.joint_names)
        ]

    def on_activate(self, previous_state):
        """Enables torque on all servos when the controller is activated."""
        self.logger.info("Activating hardware... Enabling torque.")
        try:
            values = [1] * len(self.motor_ids)
            self.driver.write_with_motor_ids(self.motor_models, self.motor_ids, "Torque_Enable", values)
            # Initialize current positions to be the first commands
            self.read(None, None)
            self.hw_commands = np.copy(self.hw_positions)
        except Exception as e:
            self.logger.error(f"Failed to enable torque: {e}")
            return CallbackReturn.ERROR
        self.logger.info("Torque enabled for all servos.")
        return CallbackReturn.SUCCESS

    def on_deactivate(self, previous_state):
        """Disables torque on all servos when the controller is deactivated."""
        self.logger.info("Deactivating hardware... Disabling torque.")
        try:
            values = [0] * len(self.motor_ids)
            self.driver.write_with_motor_ids(self.motor_models, self.motor_ids, "Torque_Enable", values)
        except Exception as e:
            self.logger.error(f"Failed to disable torque: {e}")
            # Don't return error on deactivate, just log it.
        self.logger.info("Torque disabled for all servos.")
        return CallbackReturn.SUCCESS
        
    def read(self, time, period):
        """
        Reads the current position of all servos from the hardware.
        This method uses a batch read for efficiency and then processes the raw
        tick values into radians/meters for ros2_control.
        """
        try:
            # Batch read raw tick values from all physical servos
            raw_ticks = self.driver.read_with_motor_ids(
                self.motor_models, self.motor_ids, "Present_Position"
            )
            # Map raw tick values by servo ID for easy lookup
            ticks_by_id = {mid: tick for mid, tick in zip(self.motor_ids, raw_ticks)}

            for i, name in enumerate(self.joint_names):
                position_rad = 0.0

                if name in self.single_servo_joints:
                    servo_id = self.single_servo_joints[name]
                    ticks = ticks_by_id.get(servo_id)
                    if ticks is None: continue

                    if name == "link6_Slider-8":
                        position_rad = self._ticks_to_gripper_dist(ticks)
                    else:
                        position_rad = self._ticks_to_rad(ticks, name)

                elif name in self.dual_servo_joints:
                    ids = self.dual_servo_joints[name]
                    leader_ticks = ticks_by_id.get(ids["leader"])
                    follower_ticks = ticks_by_id.get(ids["follower"])
                    if leader_ticks is None or follower_ticks is None: continue

                    leader_rad = self._ticks_to_rad(leader_ticks, name)
                    # Follower servo is mounted opposite, so its value is inverted
                    follower_rad = self._ticks_to_rad(4095 - follower_ticks, name)
                    position_rad = (leader_rad + follower_rad) / 2.0
                
                # --- Continuous Joint Unwrapping (for base_link_Revolute-1) ---
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

        except Exception as e:
            self.logger.error(f"Error during read: {e}")
            return CallbackReturn.ERROR
            
        return CallbackReturn.SUCCESS

    def write(self, time, period):
        """
        Writes the commanded positions to all servos.
        This method calculates the target tick value for each physical servo
        and sends them all in a single synchronized command.
        """
        commands_to_sync = {} # { servo_id: ticks }

        for i, name in enumerate(self.joint_names):
            cmd_rad = self.hw_commands[i]
            final_rad = cmd_rad

            # --- Shortest Path Calculation (for base_link_Revolute-1) ---
            if name == "base_link_Revolute-1":
                current_unwrapped = self.hw_positions[i]
                delta = cmd_rad - current_unwrapped
                # This finds the shortest angle to the target, preserving the unwrapped position
                final_rad = current_unwrapped + (delta + math.pi) % (2 * math.pi) - math.pi

            # --- Convert command to ticks ---
            if name == "link6_Slider-8":
                position_ticks = self._gripper_dist_to_ticks(final_rad)
            else:
                position_ticks = self._rad_to_ticks(final_rad, name)

            # --- Populate sync command dictionary ---
            if name in self.single_servo_joints:
                servo_id = self.single_servo_joints[name]
                commands_to_sync[servo_id] = position_ticks
            elif name in self.dual_servo_joints:
                ids = self.dual_servo_joints[name]
                commands_to_sync[ids["leader"]] = position_ticks
                commands_to_sync[ids["follower"]] = 4095 - position_ticks

        if commands_to_sync:
            try:
                # Prepare arguments for the driver's batch write method
                ids_to_write = list(commands_to_sync.keys())
                values_to_write = list(commands_to_sync.values())
                models_to_write = ["sts3215"] * len(ids_to_write) # Assuming all are the same model
                
                self.driver.write_with_motor_ids(
                    models_to_write, ids_to_write, "Goal_Position", values_to_write
                )
            except Exception as e:
                self.logger.error(f"Error during write: {e}")
                return CallbackReturn.ERROR

        return CallbackReturn.SUCCESS

    # --- Helper Conversion Methods (translated from C++) ---
    def _rad_to_ticks(self, rad, joint_name):
        # Special handling for 3:1 gear ratio on the base joint
        if joint_name == "base_link_Revolute-1":
            rad *= 3.0
        
        scale, offset = self.calibration.get(joint_name, (2048 / math.pi, 2048))
        return int(offset + rad * scale)

    def _ticks_to_rad(self, ticks, joint_name):
        scale, offset = self.calibration.get(joint_name, (2048 / math.pi, 2048))
        rad = (ticks - offset) / scale
        
        # Special handling for 3:1 gear ratio on the base joint
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