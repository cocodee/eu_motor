#ifndef EYOU_SYSTEM_INTERFACE_HPP_
#define EYOU_SYSTEM_INTERFACE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Include the SDK header
#include "eu_motor.h"

namespace eyou_robot_control
{
class EyouSystemInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(EyouSystemInterface)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // CAN bus manager
    std::unique_ptr<CanNetworkManager> can_manager_;
    
    // Vector of motor node instances
    std::vector<std::shared_ptr<EuMotorNode>> motor_nodes_;

    // Store the commands and states for all joints
    std::vector<double> hw_commands_positions_;
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;

    // Parameters from URDF
    int can_device_index_;
    long long can_baud_rate_;
};

}  // namespace eyou_robot_control

#endif  // EYOU_SYSTEM_INTERFACE_HPP_