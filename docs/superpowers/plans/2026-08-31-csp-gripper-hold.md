# CSP Gripper Hold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add CSP-only, feedback-driven gripper hold-force control with a hardware torque ceiling and Python bindings.

**Architecture:** A pure C++ `GripperHoldController` owns the state machine and is unit-tested without CAN hardware. `EuMotorNode` feeds it cached TPDO feedback and sends its resulting CSP position target; pybind11 exposes the configuration and read-only state. No code below `ros2_ws/` changes.

**Tech Stack:** C++17, CMake, existing CANopen SDK, pybind11.

**Spec:** `docs/design_gripper_stall_protection.md`

## Global Constraints

- Keep the drive in CSP; do not call or configure CST for gripper hold.
- Do not modify `ros2_ws/**`.
- Public Python/SDK position units are degrees; torque is per-mille of rated torque.
- The Python loop must continue calling CSP target sending while holding.
- `0x6072` is written and read back before CSP enablement.

### Task 1: Pure controller and deterministic tests

**Files:**
- Create: `include/gripper_hold_controller.h`
- Create: `src/gripper_hold_controller.cpp`
- Create: `src/test_gripper_hold_controller.cpp`
- Modify: `src/CMakeLists.txt`

**Interfaces:**
- Produces `GripperConfig`, `GripperState`, `GripperControlResult`, and `GripperHoldController::configure/process/clearSafeStop`.
- Consumes no CAN or vendor API; `process(target_deg, MotorFeedbackData, now)` returns the CSP target and state.

- [ ] Write failing tests for invalid configuration, contact debounce by distinct timestamps, force-low/force-high correction, direction reversal, stale feedback safe stop, and disabled passthrough.
- [ ] Run the controller test and verify it fails because the controller does not exist.
- [ ] Implement the minimum pure state machine: validation, approach, hold P-control with deadband/step/offset limits, and safe stop.
- [ ] Run the controller test and verify it passes.

### Task 2: Integrate controller in `EuMotorNode`

**Files:**
- Modify: `include/eu_motor.h`
- Modify: `src/eu_motor.cpp`
- Modify: `src/CMakeLists.txt`

**Interfaces:**
- Consumes Task 1 controller.
- Produces `setGripperConfig`, gripper state queries, safe-stop clear, and CSP target interception.

- [ ] Write a failing compile-level integration test for the public gripper API.
- [ ] Add the API and controller member, passing cached feedback to `process()` in `sendCspTargetPosition()`.
- [ ] In `configureCspMode()`, set and verify `0x6072` before mode setup; fail safely on mismatch.
- [ ] Build and run controller tests.

### Task 3: Python surface and example

**Files:**
- Modify: `src/bindings.cpp`
- Create: `example/test_gripper_csp_mode.cpp`
- Modify: `example/CMakeLists.txt`

**Interfaces:**
- Consumes Task 2 public API.
- Produces `GripperState`, `GripperConfig`, configuration, state and safe-stop Python methods.

- [ ] Write a failing pybind compile/build check for the new symbols.
- [ ] Bind the API and add the 20 ms CSP-only example loop.
- [ ] Build the project and run all no-hardware tests.

### Task 4: Final verification

**Files:**
- Modify: `docs/design_gripper_stall_protection.md` only if implementation names deviate.

- [ ] Run the complete CMake configure/build and controller test executable.
- [ ] Inspect the diff to verify no `ros2_ws/**` paths changed.
- [ ] Record exact results and hardware-test limitations in the final handoff.
