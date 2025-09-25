#ifndef ARM_HARDWARE_INTERFACE_HPP_
#define ARM_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include "sts_driver.hpp"
#include <vector>
#include <map>
#include <string>
#include <optional>
#include <memory>
#include <set>
#include <cmath>

namespace simple_arm {

class ArmHardware : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardware)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    std::unique_ptr<ServoController> driver_;
    std::vector<double> hw_commands_, hw_positions_, hw_velocities_;
    std::map<std::string, int> single_servo_joints_, revolutions_;
    std::map<std::string, std::pair<int, int>> dual_servo_joints_;
    std::map<std::string, std::pair<double, double>> calibration_;  // scale, offset
    std::map<std::string, double> unwrapped_positions_;
    std::map<std::string, std::optional<double>> last_wrapped_angles_;
    std::vector<std::string> joint_names_;
    const double BASE_JOINT_RANGE = 2 * M_PI;
    
};

}  // namespace simple_arm

#endif  // ARM_HARDWARE_INTERFACE_HPP_