#include "../include/eu_motor.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

int main() {
    constexpr huint8 kDeviceIndex = 0;
    constexpr huint8 kNodeId = 31;
    constexpr huint16 kPeriodMs = 20;

    CanNetworkManager can_manager;
    can_manager.initDevice(harmonic_DeviceType_Canable, kDeviceIndex, harmonic_Baudrate_1000);
    auto motor = std::make_shared<EuMotorNode>(kDeviceIndex, kNodeId);

    GripperConfig config;
    config.open_position_deg = 0.0f;
    config.close_position_deg = 90.0f;
    config.torque_limit_milli = 500;
    config.hold_torque_milli = 300;
    config.contact_detect_threshold_milli = 180;
    config.overload_threshold_milli = 450;
    config.contact_detect_consecutive_samples = 5;
    config.feedback_timeout_ms = 100;

    if (!motor->setGripperConfig(config) || !motor->configureCspMode() ||
        !motor->startAutoFeedback(0, 255, kPeriodMs)) {
        std::cerr << "Failed to configure CSP gripper hold control." << std::endl;
        return 1;
    }
    MotorFeedbackManager::getInstance().registerCallback();

    // Keep calling CSP while holding: the SDK adjusts its internal target from torque feedback.
    for (int i = 0; i < 3000; ++i) {
        motor->sendCspTargetPosition(config.close_position_deg, 0, true);
        if (motor->getGripperState() == GripperState::SafeStop) {
            std::cerr << "Gripper entered safe stop." << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
    }

    for (int i = 0; i < 500; ++i) {
        motor->sendCspTargetPosition(config.open_position_deg, 0, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
    }
    motor->disable();
    return 0;
}
