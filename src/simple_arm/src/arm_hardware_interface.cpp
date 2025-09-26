#include "arm_hardware_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <optional>

namespace simple_arm {

// --- Helper functions (unchanged) ---
double convert_rad_to_ticks(double rad, const std::string& joint_name, const std::map<std::string, std::pair<double, double>>& calibration) {
    if (joint_name == "base_link_Revolute-1") rad *= 3.0;
    auto it = calibration.find(joint_name);
    return (it != calibration.end()) ? 
           static_cast<int>(it->second.second + rad * it->second.first) : 
           static_cast<int>(2048 + (rad * (2048 / M_PI)));
}

double convert_ticks_to_rad(int16_t ticks, const std::string& joint_name, const std::map<std::string, std::pair<double, double>>& calibration) {
    double rad = 0.0;
    auto it = calibration.find(joint_name);
    rad = (it != calibration.end()) ? 
          ((ticks - it->second.second) / it->second.first) : 
          ((ticks - 2048) * (M_PI / 2048));
    if (joint_name == "base_link_Revolute-1") rad /= 3.0;
    return rad;
}

int convert_gripper_dist_to_ticks(double dist) {
    double min_dist = 0.0, max_dist = 0.024;
    int min_ticks = 2903, max_ticks = 1518;
    dist = std::max(min_dist, std::min(dist, max_dist));
    return static_cast<int>(min_ticks + ((dist - min_dist) / (max_dist - min_dist)) * (max_ticks - min_ticks));
}

double convert_ticks_to_gripper_dist(int16_t ticks) {
    double min_dist = 0.0, max_dist = 0.024;
    int min_ticks = 2903, max_ticks = 1518;
    int low_tick = std::min(min_ticks, max_ticks);
    int high_tick = std::max(min_ticks, max_ticks);
    ticks = std::max(low_tick, std::min(static_cast<int>(ticks), high_tick));
    double position = min_dist + ((ticks - min_ticks) / static_cast<double>(max_ticks - min_ticks)) * (max_dist - min_dist);
    return std::max(min_dist, std::min(position, max_dist));
}

hardware_interface::CallbackReturn ArmHardware::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::string serial_port = info.hardware_parameters.count("serial_port") ? info.hardware_parameters.at("serial_port") : "/dev/ttyUSB0";
    uint32_t baudrate = info.hardware_parameters.count("baudrate") ? std::stoi(info.hardware_parameters.at("baudrate")) : 1000000;

    driver_ = std::make_unique<ServoController>(serial_port, baudrate);

    single_servo_joints_ = {{"base_link_Revolute-1", 1}, {"link3_Revolute-5", 6}, {"link4_Revolute-6", 7}, {"link5_Revolute-7", 8}, {"link6_Slider-8", 9}};
    dual_servo_joints_ = {{"link1_Revolute-3", {2, 3}}, {"link2_Revolute-4", {4, 5}}};
    calibration_ = {{"base_link_Revolute-1", {644.9, 3453}}, {"link1_Revolute-3", {651, 3072}}, {"link2_Revolute-4", {647.2, 985}}, {"link3_Revolute-5", {628, 3112}}, {"link4_Revolute-6", {640, 702}}, {"link5_Revolute-7", {654, 99}}, {"link6_Slider-8", {14427, 2908}}};

    for (const auto& joint : info.joints) {
        joint_names_.push_back(joint.name);
    }

    hw_commands_.resize(joint_names_.size(), 0.0);
    hw_positions_.resize(joint_names_.size(), 0.0);
    hw_velocities_.resize(joint_names_.size(), 0.0); // Kept for internal consistency, but not exported

    // *** MODIFICATION: Corrected the validation logic to match the position-only interface ***
    for (const auto& joint : info.joints) {
        if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmHardware"), "Joint '%s' has an invalid command interface configuration.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        // This check is now for ONE state interface: position
        if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmHardware"), "Joint '%s' has an invalid state interface configuration.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    for (const auto& name : joint_names_) {
        revolutions_[name] = 0;
        last_wrapped_angles_[name] = std::nullopt;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // *** MODIFICATION: Only export the POSITION state interface ***
        state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        // The velocity interface is no longer exported.
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn ArmHardware::on_activate(const rclcpp_lifecycle::State&) {
    for (int id = 1; id <= 9; ++id) {
        if (!driver_->enable_torque(static_cast<uint8_t>(id), true)) {
            RCLCPP_ERROR(rclcpp::get_logger("ArmHardware"), "Failed to enable torque for servo ID %d", id);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Torque enabled for all servos.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardware::on_deactivate(const rclcpp_lifecycle::State&) {
    for (int id = 1; id <= 9; ++id) {
        driver_->enable_torque(static_cast<uint8_t>(id), false);
    }
    RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "Torque disabled for all servos.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmHardware::read(const rclcpp::Time&, const rclcpp::Duration&) {
    // *** MODIFICATION: Simplified to only read positions. Velocity is not needed as it's not exported. ***
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string& name = joint_names_[i];
        double position_rad = 0.0;

        if (single_servo_joints_.count(name)) {
            auto ticks_opt = driver_->get_position(single_servo_joints_.at(name));
            if (!ticks_opt.has_value()) continue;
            if (name == "link6_Slider-8") {
                position_rad = convert_ticks_to_gripper_dist(ticks_opt.value());
            } else {
                position_rad = convert_ticks_to_rad(ticks_opt.value(), name, calibration_);
            }
        } else if (dual_servo_joints_.count(name)) {
            auto [leader_id, follower_id] = dual_servo_joints_.at(name);
            auto leader_ticks_opt = driver_->get_position(leader_id);
            auto follower_ticks_opt = driver_->get_position(follower_id);
            if (!leader_ticks_opt.has_value() || !follower_ticks_opt.has_value()) continue;
            double leader_rad = convert_ticks_to_rad(leader_ticks_opt.value(), name, calibration_);
            double follower_rad = convert_ticks_to_rad(4095 - follower_ticks_opt.value(), name, calibration_);
            position_rad = (leader_rad + follower_rad) / 2.0;
        }

        if (name == "base_link_Revolute-1") {
            if (last_wrapped_angles_[name].has_value()) {
                double delta = position_rad - last_wrapped_angles_[name].value();
                if (delta > M_PI) revolutions_[name]--; else if (delta < -M_PI) revolutions_[name]++;
            }
            double unwrapped_pos = position_rad + revolutions_[name] * 2 * M_PI;
            last_wrapped_angles_[name] = position_rad;
            hw_positions_[i] = unwrapped_pos;
        } else {
            hw_positions_[i] = position_rad;
        }
        // Velocity is not exported, so this value is unused by the rest of the system.
        hw_velocities_[i] = 0.0;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardware::write(const rclcpp::Time&, const rclcpp::Duration&) {
    std::map<uint8_t, uint16_t> commands_to_sync;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string& name = joint_names_[i];
        double cmd = hw_commands_[i];
        double final_rad = cmd;

        if (name == "base_link_Revolute-1") {
            // *** BUG FIX: The shortest path calculation was incorrect. This is the corrected version. ***
            double current_unwrapped = unwrapped_positions_[name]; // Use the stored unwrapped position
            double delta = cmd - current_unwrapped;
            final_rad = current_unwrapped + std::fmod(delta + M_PI, 2 * M_PI) - M_PI;
        }

        uint16_t position_ticks = (name == "link6_Slider-8") ? 
                                  convert_gripper_dist_to_ticks(final_rad) : 
                                  convert_rad_to_ticks(final_rad, name, calibration_);

        if (single_servo_joints_.count(name)) {
            commands_to_sync[single_servo_joints_.at(name)] = position_ticks;
        } else if (dual_servo_joints_.count(name)) {
            auto [leader_id, follower_id] = dual_servo_joints_.at(name);
            commands_to_sync[leader_id] = position_ticks;
            commands_to_sync[follower_id] = 4095 - position_ticks;
        }
    }

    if (!commands_to_sync.empty()) {
        driver_->sync_write_positions(commands_to_sync);
    }
    return hardware_interface::return_type::OK;
}
}  // namespace simple_arm

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(simple_arm::ArmHardware, hardware_interface::SystemInterface);