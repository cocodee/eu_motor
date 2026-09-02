#include "../include/eu_motor.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

huint8 parseArgument(const char* value, const char* name, unsigned long maximum) {
    try {
        const unsigned long parsed = std::stoul(value);
        if (parsed > maximum) throw std::out_of_range(name);
        return static_cast<huint8>(parsed);
    } catch (const std::exception&) {
        throw std::runtime_error(std::string("Invalid ") + name + ": " + value);
    }
}

hreal32 moveToward(hreal32 current_deg, hreal32 target_deg, hreal32 max_step_deg) {
    const hreal32 delta = target_deg - current_deg;
    return current_deg + std::max(-max_step_deg, std::min(delta, max_step_deg));
}

const char* stateName(GripperState state) {
    switch (state) {
        case GripperState::Disabled: return "Disabled";
        case GripperState::Approach: return "Approach";
        case GripperState::Hold: return "Hold";
        case GripperState::SafeStop: return "SafeStop";
    }
    return "Unknown";
}

void printProgress(EuMotorNode& motor, hreal32 command_target_deg, int elapsed_ms) {
    const MotorFeedbackData feedback = motor.getLatestFeedback();
    long long feedback_age_ms = -1;
    if (feedback.last_update_time.time_since_epoch().count() != 0) {
        feedback_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - feedback.last_update_time).count();
    }
    std::cout << "[t=" << std::setw(5) << elapsed_ms << " ms]"
              << " state=" << stateName(motor.getGripperState())
              << " cmd=" << std::fixed << std::setprecision(2) << command_target_deg << " deg"
              << " effective=" << motor.getLastCspEffectiveTargetPosition() << " deg"
              << " actual=" << feedback.position_deg << " deg"
              << " torque=" << feedback.torque_milli << " permille"
              << " hold_error=" << motor.getHoldTorqueError() << " permille"
              << " status=0x" << std::hex << feedback.status_word << std::dec
              << " fault=" << (feedback.in_fault ? "yes" : "no")
              << " feedback_age=";
    if (feedback_age_ms < 0) std::cout << "none";
    else std::cout << feedback_age_ms << " ms";
    std::cout << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
    huint8 device_index = 0;
    huint8 node_id = 31;
    if (argc > 3) {
        std::cerr << "Usage: " << argv[0] << " [device_index] [node_id]" << std::endl;
        return 2;
    }
    try {
        if (argc >= 2) device_index = parseArgument(argv[1], "device index", 255);
        if (argc == 3) node_id = parseArgument(argv[2], "node id", 127);
        if (node_id == 0) throw std::runtime_error("Node ID must be in the range 1..127.");
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 2;
    }

    constexpr huint16 kPeriodMs = 20;
    constexpr hreal32 kApproachStepDeg = 0.01f;  // 0.5 deg/s at 20 ms
    constexpr hreal32 kReleaseStepDeg = 0.10f;   // 5.0 deg/s at 20 ms
    constexpr int kHoldDurationMs = 10000;

    CanNetworkManager can_manager;
    can_manager.initDevice(harmonic_DeviceType_Canable, device_index, harmonic_Baudrate_1000);
    auto motor = std::make_shared<EuMotorNode>(device_index, node_id);
    std::cout << "Gripper test: device=" << static_cast<int>(device_index)
              << ", node=" << static_cast<int>(node_id) << std::endl;

    GripperConfig config;
    config.open_position_deg = -40.0f;
    config.close_position_deg = 20.0f;
    config.torque_limit_milli = 700;
    config.hold_torque_milli = 250;
    config.hold_torque_tolerance_milli = 20;
    config.contact_detect_threshold_milli = 150;
    config.overload_threshold_milli = 550;
    config.contact_detect_consecutive_samples = 1;
    config.force_kp_deg_per_milli = 0.001f;
    config.max_hold_step_deg = 0.05f;
    config.max_hold_target_offset_deg = 2.0f;
    config.position_tolerance_deg = 0.5f;
    config.feedback_timeout_ms = 100;

    if (!motor->setGripperConfig(config) || !motor->configureCspMode() ||
        !motor->startAutoFeedback(0, 255, kPeriodMs)) {
        std::cerr << "Failed to configure CSP gripper hold control." << std::endl;
        return 1;
    }
    MotorFeedbackManager::getInstance().registerCallback();

    // Start from the measured position and ramp toward close. Never jump directly to an end stop.
    hreal32 command_target_deg = motor->getPosition();
    std::cout << "Initial position=" << std::fixed << std::setprecision(2) << command_target_deg
              << " deg; approach: " << config.open_position_deg << " -> " << config.close_position_deg
              << " deg; target hold torque=" << config.hold_torque_milli << " permille" << std::endl;
    const int max_approach_iterations = std::max(
        1, static_cast<int>(std::ceil(
               std::abs(config.close_position_deg - command_target_deg) / kApproachStepDeg)) + 1);
    const int hold_iterations = std::max(1, kHoldDurationMs / static_cast<int>(kPeriodMs));
    int completed_hold_iterations = 0;
    GripperState previous_state = motor->getGripperState();
    for (int i = 0;; ++i) {
        if (motor->getGripperState() != GripperState::Hold) {
            command_target_deg = moveToward(command_target_deg, config.close_position_deg, kApproachStepDeg);
        }
        motor->sendCspTargetPosition(command_target_deg, 0, true);
        const GripperState current_state = motor->getGripperState();
        if (current_state != previous_state) {
            std::cout << "State transition: " << stateName(previous_state) << " -> "
                      << stateName(current_state) << std::endl;
            previous_state = current_state;
        }
        if (i % 25 == 0) printProgress(*motor, command_target_deg, i * kPeriodMs);
        if (current_state == GripperState::SafeStop) {
            std::cerr << "Gripper entered safe stop." << std::endl;
            break;
        }
        if (current_state == GripperState::Hold) {
            // Keep requesting closure. The SDK owns the internal hold target; sending a
            // contact-point target here could be interpreted as an opening command once
            // the hold controller has advanced its target.
            command_target_deg = config.close_position_deg;
            if (++completed_hold_iterations >= hold_iterations) {
                std::cout << "Configured hold duration completed." << std::endl;
                break;
            }
        } else if (command_target_deg == config.close_position_deg) {
            std::cout << "Reached configured close position without contact." << std::endl;
            break;
        } else if (i + 1 >= max_approach_iterations) {
            std::cerr << "Approach iteration limit reached before the configured close position."
                      << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
    }

    std::cout << "Releasing gripper." << std::endl;
    for (int i = 0; i < 500; ++i) {
        command_target_deg = moveToward(command_target_deg, config.open_position_deg, kReleaseStepDeg);
        motor->sendCspTargetPosition(command_target_deg, 0, true);
        if (i % 25 == 0) printProgress(*motor, command_target_deg, i * kPeriodMs);
        std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
    }
    motor->disable();
    return 0;
}
