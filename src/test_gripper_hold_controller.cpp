#include "../include/eu_motor.h"

#include <cassert>
#include <chrono>

namespace {

MotorFeedbackData feedback(float position, hint16 torque, int ms) {
    MotorFeedbackData value;
    value.position_deg = position;
    value.torque_milli = torque;
    value.last_update_time = std::chrono::steady_clock::time_point(std::chrono::milliseconds(ms));
    return value;
}

GripperConfig config() {
    GripperConfig value;
    value.open_position_deg = 0.0f;
    value.close_position_deg = 90.0f;
    value.torque_limit_milli = 500;
    value.hold_torque_milli = 300;
    value.hold_torque_tolerance_milli = 20;
    value.contact_detect_threshold_milli = 150;
    value.overload_threshold_milli = 450;
    value.contact_detect_consecutive_samples = 2;
    value.force_kp_deg_per_milli = 0.01f;
    value.max_hold_step_deg = 0.5f;
    value.max_hold_target_offset_deg = 2.0f;
    value.feedback_timeout_ms = 100;
    return value;
}

void test_disabled_passthrough() {
    GripperHoldController controller;
    const auto result = controller.process(70.0f, feedback(10.0f, 0, 1),
                                           std::chrono::steady_clock::time_point(std::chrono::milliseconds(1)));
    assert(result.target_position_deg == 70.0f);
    assert(result.state == GripperState::Disabled);
}

void test_contact_requires_distinct_samples() {
    GripperHoldController controller;
    assert(controller.configure(config()));
    const auto now = std::chrono::steady_clock::time_point(std::chrono::milliseconds(20));
    controller.process(90.0f, feedback(40.0f, 180, 20), now);
    controller.process(90.0f, feedback(40.0f, 180, 20), now);
    assert(controller.state() == GripperState::Approach);
    controller.process(90.0f, feedback(40.0f, 180, 40),
                       std::chrono::steady_clock::time_point(std::chrono::milliseconds(40)));
    assert(controller.state() == GripperState::Hold);
    assert(controller.gripPosition() == 40.0f);
}

void test_contact_detects_when_target_is_slightly_ahead_of_position() {
    GripperHoldController controller;
    assert(controller.configure(config()));

    // Position tolerance is for release/arrival decisions.  It must not make
    // a slow closing command lose its closing intent before contact detection.
    controller.process(40.05f, feedback(40.0f, 180, 20),
                       std::chrono::steady_clock::time_point(std::chrono::milliseconds(20)));
    controller.process(40.05f, feedback(40.0f, 180, 40),
                       std::chrono::steady_clock::time_point(std::chrono::milliseconds(40)));

    assert(controller.state() == GripperState::Hold);
    assert(controller.gripPosition() == 40.0f);
}

void test_hold_adjusts_target_in_force_direction() {
    GripperHoldController controller;
    assert(controller.configure(config()));
    controller.process(90.0f, feedback(40.0f, 180, 20),
                       std::chrono::steady_clock::time_point(std::chrono::milliseconds(20)));
    controller.process(90.0f, feedback(40.0f, 180, 40),
                       std::chrono::steady_clock::time_point(std::chrono::milliseconds(40)));
    const auto low_force = controller.process(90.0f, feedback(40.0f, 250, 60),
                                              std::chrono::steady_clock::time_point(std::chrono::milliseconds(60)));
    assert(low_force.target_position_deg > 40.0f);
    const auto high_force = controller.process(90.0f, feedback(40.0f, 400, 80),
                                               std::chrono::steady_clock::time_point(std::chrono::milliseconds(80)));
    assert(high_force.target_position_deg < low_force.target_position_deg);
}

void test_stale_feedback_enters_safe_stop() {
    GripperHoldController controller;
    assert(controller.configure(config()));
    const auto result = controller.process(90.0f, feedback(40.0f, 200, 1),
                                           std::chrono::steady_clock::time_point(std::chrono::milliseconds(200)));
    assert(result.state == GripperState::SafeStop);
    assert(result.target_position_deg == 40.0f);
}

void test_open_command_releases_safe_stop_from_safe_position() {
    GripperHoldController controller;
    assert(controller.configure(config()));
    controller.process(90.0f, feedback(40.0f, 200, 1),
                       std::chrono::steady_clock::time_point(std::chrono::milliseconds(200)));
    assert(controller.state() == GripperState::SafeStop);

    const auto released = controller.process(0.0f, feedback(40.0f, 200, 201),
                                             std::chrono::steady_clock::time_point(std::chrono::milliseconds(201)));
    assert(released.state == GripperState::Approach);
    assert(released.target_position_deg == 0.0f);
}

void test_release_ignores_residual_overload_torque() {
    GripperHoldController controller;
    assert(controller.configure(config()));
    controller.process(90.0f, feedback(40.0f, 200, 1),
                       std::chrono::steady_clock::time_point(std::chrono::milliseconds(200)));
    assert(controller.state() == GripperState::SafeStop);

    controller.process(0.0f, feedback(40.0f, 480, 201),
                       std::chrono::steady_clock::time_point(std::chrono::milliseconds(201)));
    const auto releasing = controller.process(0.0f, feedback(39.0f, 480, 202),
                                              std::chrono::steady_clock::time_point(std::chrono::milliseconds(202)));
    assert(releasing.state == GripperState::Approach);
    assert(releasing.target_position_deg == 0.0f);
}

}  // namespace

int main() {
    test_disabled_passthrough();
    test_contact_requires_distinct_samples();
    test_contact_detects_when_target_is_slightly_ahead_of_position();
    test_hold_adjusts_target_in_force_direction();
    test_stale_feedback_enters_safe_stop();
    test_open_command_releases_safe_stop_from_safe_position();
    test_release_ignores_residual_overload_torque();
    return 0;
}
