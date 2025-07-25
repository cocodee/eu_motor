#include "eyou_robot_control/eyou_system_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// This is required to export the class as a plugin
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  eyou_robot_control::EyouSystemInterface,
  hardware_interface::SystemInterface
)

namespace eyou_robot_control
{

hardware_interface::CallbackReturn EyouSystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "Initializing...");

    // Get hardware parameters from URDF
    try {
        can_device_index_ = std::stoi(info_.hardware_parameters.at("can_device_index"));
        can_baud_rate_ = std::stoll(info_.hardware_parameters.at("can_baud_rate"));
    } catch (const std::out_of_range& oor) {
        RCLCPP_ERROR(rclcpp::get_logger("EyouSystemInterface"), "Missing required hardware parameters 'can_device_index' or 'can_baud_rate'.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "CAN Device Index: %d, Baud Rate: %lld", can_device_index_, can_baud_rate_);

    // Initialize storage for commands and states
    hw_commands_positions_.resize(info_.joints.size(), 0.0);
    hw_states_positions_.resize(info_.joints.size(), 0.0);
    hw_states_velocities_.resize(info_.joints.size(), 0.0);
    hw_start_enabled_.resize(info_.joints.size(), true);

    // Initialize CAN bus
    can_manager_ = std::make_unique<CanNetworkManager>();
    try {
        can_manager_->initDevice(harmonic_DeviceType_Canable, can_device_index_, (harmonic_Baudrate)can_baud_rate_);
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("EyouSystemInterface"), "Failed to initialize CAN device: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize motor nodes
    motor_nodes_.resize(info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        // Assuming Node ID = joint index + 1. This should be configured in the URDF.
        huint8 node_id = std::stoi(info_.joints[i].parameters.at("node_id"));
        RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "Initializing motor for joint '%s' with Node ID %d", info_.joints[i].name.c_str(), node_id);
        motor_nodes_[i] = std::make_shared<EuMotorNode>(can_device_index_, node_id);

        // Check for start_enabled parameter
        if (info_.joints[i].parameters.find("start_enabled") != info_.joints[i].parameters.end()) {
            std::string start_enabled_str = info_.joints[i].parameters.at("start_enabled");
            std::transform(start_enabled_str.begin(), start_enabled_str.end(), start_enabled_str.begin(), ::tolower);
            if (start_enabled_str == "false") {
                hw_start_enabled_[i] = false;
                RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "Joint '%s' is configured to be disabled on start.", info_.joints[i].name.c_str());
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "Initialization successful.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EyouSystemInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EyouSystemInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn EyouSystemInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "Activating...");

    // Set some default values and set current position as command
    for (size_t i = 0; i < motor_nodes_.size(); ++i) {
        try {
            hw_states_positions_[i] = motor_nodes_[i]->getPosition();
            hw_states_velocities_[i] = motor_nodes_[i]->getVelocity();
            hw_commands_positions_[i] = hw_states_positions_[i];
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(rclcpp::get_logger("EyouSystemInterface"), "Failed to read initial state for joint %zu: %s", i, e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // Configure and enable motors
    for (size_t i = 0; i < motor_nodes_.size(); ++i) {
        if (hw_start_enabled_[i]) {
            RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "Enabling motor for joint %s...", info_.joints[i].name.c_str());
            if (!motor_nodes_[i]->clearFault()) {
                RCLCPP_ERROR(rclcpp::get_logger("EyouSystemInterface"), "Failed to clear fault for joint %s", info_.joints[i].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (!motor_nodes_[i]->configureCspMode()) {
                RCLCPP_ERROR(rclcpp::get_logger("EyouSystemInterface"), "Failed to configure CSP mode for joint %s", info_.joints[i].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            // Configure TPDO1 for feedback every 20ms
            if (!motor_nodes_[i]->startAutoFeedback(0, 255, 20)) {
                RCLCPP_ERROR(rclcpp::get_logger("EyouSystemInterface"), "Failed to start auto feedback for joint %s", info_.joints[i].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        } else {
            RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "Skipping activation for joint %s as it is disabled.", info_.joints[i].name.c_str());
            motor_nodes_[i]->disable();
            if (!motor_nodes_[i]->clearFault()) {
                RCLCPP_ERROR(rclcpp::get_logger("EyouSystemInterface"), "Failed to clear fault for joint %s", info_.joints[i].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }            
            // Configure TPDO1 for feedback every 10ms
            if (!motor_nodes_[i]->startAutoFeedback(0, 255, 10)) {
                RCLCPP_ERROR(rclcpp::get_logger("EyouSystemInterface"), "Failed to start auto feedback for joint %s", info_.joints[i].name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }
    
    // Register the global callback for receiving TPDO data
    MotorFeedbackManager::getInstance().registerCallback();

    RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "Activation successful.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EyouSystemInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "Deactivating...");

    // Stop processing CAN frames
    for (const auto& node : motor_nodes_) {
        node->disable();
    }

    RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "Deactivation successful.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EyouSystemInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    for (size_t i = 0; i < motor_nodes_.size(); ++i) {
        MotorFeedbackData feedback = motor_nodes_[i]->getLatestFeedback();
        
        // The feedback manager returns 0 if no data has been received yet.
        // Only update if the timestamp is not default.
        if (feedback.last_update_time.time_since_epoch().count() != 0) {
            hw_states_positions_[i] = feedback.position_deg;
            hw_states_velocities_[i] = feedback.velocity_dps;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type EyouSystemInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    bool any_motor_enabled = false;
    for (size_t i = 0; i < motor_nodes_.size(); ++i) {
        if (hw_start_enabled_[i]) {
            // The sendCspTargetPosition function does not send the SYNC message itself.
            motor_nodes_[i]->sendCspTargetPosition(hw_commands_positions_[i], 0, true);
            RCLCPP_INFO(rclcpp::get_logger("EyouSystemInterface"), "send motor position %f, node_id:%d",hw_commands_positions_[i],motor_nodes_[i]->getNodeId());
            any_motor_enabled = true;
        }
    }

    // After sending all position targets, send a single SYNC message
    // to make all motors execute their received command simultaneously.
    if (any_motor_enabled) {
        // Find the first enabled motor to send the SYNC command
        for (size_t i = 0; i < motor_nodes_.size(); ++i) {
            if (hw_start_enabled_[i]) {
                motor_nodes_[i]->sendSync();
                break;
            }
        }
    }

    return hardware_interface::return_type::OK;
}

}  // namespace eyou_robot_control