#include "../include/eu_motor.h"
#include <cstddef>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

/**
 * @brief 将 harmonic 库的错误码转换为可读的字符串
 * 
 * @param code 错误码
 * @return std::string 错误描述
 */
std::string harmonicErrorToString(int code) {
    switch (code) {
        case HARMONIC_SUCCESS: return "Success";
        case HARMONIC_FAILED_DEVICEDISABLED: return "Device disabled or does not exist";
        case HARMONIC_FAILED_OPENFAILED: return "Device open failed";
        case HARMONIC_FAILED_CANSEND: return "CAN send failed";
        case HARMONIC_FAILED_CANRECEIVE: return "CAN receive failed";
        case HARMONIC_FAILED_ReadLocalDict: return "Read local dictionary failed";
        case HARMONIC_FAILED_WriteLocalDict: return "Write local dictionary failed";
        case HARMONIC_FAILED_NoRespondR: return "SDO read request timed out (no response)";
        case HARMONIC_FAILED_NoRespondW: return "SDO write request timed out (no response)";
        case HARMONIC_FAILED_UNKNOWN: return "Unknown failure";
        default:
            std::stringstream ss;
            ss << "Unknown error code: " << code;
            return ss.str();
    }
}

bool GripperHoldController::configure(const GripperConfig& config) {
    if (config.open_position_deg == config.close_position_deg ||
        config.torque_limit_milli <= 0 || config.hold_torque_milli <= 0 ||
        config.contact_detect_threshold_milli <= 0 || config.overload_threshold_milli <= 0 ||
        config.hold_torque_tolerance_milli < 0 || config.contact_detect_threshold_milli > config.hold_torque_milli ||
        config.hold_torque_milli + config.hold_torque_tolerance_milli >= config.overload_threshold_milli ||
        config.overload_threshold_milli >= config.torque_limit_milli ||
        config.contact_detect_consecutive_samples <= 0 || config.force_kp_deg_per_milli <= 0.0f ||
        config.max_hold_step_deg <= 0.0f || config.max_hold_target_offset_deg < 0.0f ||
        config.position_tolerance_deg < 0.0f || config.feedback_timeout_ms == 0) {
        return false;
    }

    config_ = config;
    closing_sign_ = config.close_position_deg > config.open_position_deg ? 1.0f : -1.0f;
    enabled_ = true;
    state_ = GripperState::Approach;
    grip_position_deg_ = config.open_position_deg;
    hold_target_position_deg_ = config.open_position_deg;
    hold_torque_error_milli_ = 0;
    contact_sample_count_ = 0;
    last_feedback_time_ = {};
    return true;
}

void GripperHoldController::disable() {
    enabled_ = false;
    state_ = GripperState::Disabled;
    contact_sample_count_ = 0;
    hold_torque_error_milli_ = 0;
    last_feedback_time_ = {};
}

void GripperHoldController::clearSafeStop() {
    if (enabled_ && state_ == GripperState::SafeStop) {
        state_ = GripperState::Approach;
        contact_sample_count_ = 0;
        last_feedback_time_ = {};
    }
}

bool GripperHoldController::isEnabled() const { return enabled_; }
GripperState GripperHoldController::state() const { return state_; }
hreal32 GripperHoldController::gripPosition() const { return grip_position_deg_; }
hint16 GripperHoldController::holdTorqueError() const { return hold_torque_error_milli_; }
const GripperConfig& GripperHoldController::config() const { return config_; }

bool GripperHoldController::isOpeningCommand(hreal32 target_deg) const {
    const hreal32 reference = (state_ == GripperState::Hold || state_ == GripperState::SafeStop)
        ? hold_target_position_deg_
        : grip_position_deg_;
    return (target_deg - reference) * closing_sign_ < -config_.position_tolerance_deg;
}

bool GripperHoldController::isClosingCommand(hreal32 target_deg, hreal32 position_deg) const {
    return (target_deg - position_deg) * closing_sign_ > config_.position_tolerance_deg;
}

hreal32 GripperHoldController::clampPosition(hreal32 position_deg) const {
    return std::max(std::min(config_.open_position_deg, config_.close_position_deg),
                    std::min(position_deg, std::max(config_.open_position_deg, config_.close_position_deg)));
}

void GripperHoldController::enterSafeStop(hreal32 position_deg) {
    state_ = GripperState::SafeStop;
    contact_sample_count_ = 0;
    hold_target_position_deg_ = clampPosition(position_deg);
}

GripperControlResult GripperHoldController::process(
    hreal32 user_target_deg,
    const MotorFeedbackData& feedback,
    std::chrono::steady_clock::time_point now) {
    if (!enabled_) return {user_target_deg, GripperState::Disabled};

    // A caller's opening command always wins over SafeStop. Compare it to the
    // measured position, rather than an internally frozen target, so the exact
    // CSP command supplied by the caller is forwarded for release.
    const bool opening_from_actual = (user_target_deg - feedback.position_deg) * closing_sign_ <
        -config_.position_tolerance_deg;
    if (state_ == GripperState::SafeStop && opening_from_actual) {
        state_ = GripperState::Approach;
        contact_sample_count_ = 0;
        hold_torque_error_milli_ = 0;
        return {user_target_deg, state_};
    }

    if (state_ == GripperState::Hold && isOpeningCommand(user_target_deg)) {
        state_ = GripperState::Approach;
        contact_sample_count_ = 0;
        hold_torque_error_milli_ = 0;
        return {user_target_deg, state_};
    }

    const bool has_feedback = feedback.last_update_time.time_since_epoch().count() != 0;
    const bool fresh = has_feedback && now >= feedback.last_update_time &&
        now - feedback.last_update_time <= std::chrono::milliseconds(config_.feedback_timeout_ms);
    const bool new_sample = fresh && feedback.last_update_time > last_feedback_time_;

    if (!fresh || feedback.in_fault) {
        enterSafeStop(feedback.position_deg);
        return {hold_target_position_deg_, state_};
    }

    if (state_ == GripperState::SafeStop) {
        state_ = GripperState::Approach;
        contact_sample_count_ = 0;
    }

    const int actual_torque = std::abs(static_cast<int>(feedback.torque_milli));
    const bool closing_command = isClosingCommand(user_target_deg, feedback.position_deg);
    // A released gripper can report a high residual torque for several TPDO
    // cycles. Only treat overload as a stop condition while closing or holding;
    // otherwise the release command would immediately re-enter SafeStop.
    if (actual_torque >= config_.overload_threshold_milli &&
        (state_ == GripperState::Hold || closing_command)) {
        enterSafeStop(feedback.position_deg);
        return {hold_target_position_deg_, state_};
    }

    if (state_ == GripperState::Approach) {
        if (new_sample) {
            last_feedback_time_ = feedback.last_update_time;
            if (closing_command &&
                actual_torque >= config_.contact_detect_threshold_milli) {
                ++contact_sample_count_;
                if (contact_sample_count_ >= config_.contact_detect_consecutive_samples) {
                    state_ = GripperState::Hold;
                    grip_position_deg_ = feedback.position_deg;
                    hold_target_position_deg_ = feedback.position_deg;
                    hold_torque_error_milli_ = static_cast<hint16>(config_.hold_torque_milli - actual_torque);
                }
            } else {
                contact_sample_count_ = 0;
            }
        }
        return {state_ == GripperState::Hold ? hold_target_position_deg_ : user_target_deg, state_};
    }

    if (state_ == GripperState::Hold && new_sample) {
        last_feedback_time_ = feedback.last_update_time;
        const int error = static_cast<int>(config_.hold_torque_milli) - actual_torque;
        hold_torque_error_milli_ = static_cast<hint16>(error);
        if (std::abs(error) > config_.hold_torque_tolerance_milli) {
            const hreal32 raw_step = config_.force_kp_deg_per_milli * static_cast<hreal32>(error);
            const hreal32 step = std::max(-config_.max_hold_step_deg, std::min(raw_step, config_.max_hold_step_deg));
            const hreal32 max_close_target = grip_position_deg_ + closing_sign_ * config_.max_hold_target_offset_deg;
            const hreal32 candidate = clampPosition(hold_target_position_deg_ + closing_sign_ * step);
            const bool exceeds_offset = (candidate - max_close_target) * closing_sign_ > 0.0f;
            hold_target_position_deg_ = exceeds_offset ? max_close_target : candidate;
            if (exceeds_offset && error > 0) enterSafeStop(hold_target_position_deg_);
        }
    }
    return {hold_target_position_deg_, state_};
}
// --- CanNetworkManager Implementation ---

void CanNetworkManager::initDevice(harmonic_DeviceType devType, huint8 devIndex, harmonic_Baudrate baudrate) {
    // 现在访问的是成员变量，用法不变
    std::lock_guard<std::mutex> lock(mutex_); 
    if (initialized_devices_.find(devIndex) == initialized_devices_.end() || !initialized_devices_[devIndex]) {
        std::cout << "CanNetworkManager: Initializing CAN device " << (int)devIndex << "..." << std::endl;
        if (harmonic_initDLL(devType, devIndex, baudrate) != HARMONIC_SUCCESS) {
            throw std::runtime_error("Failed to initialize CAN device index " + std::to_string(devIndex));
        }
        initialized_devices_[devIndex] = true;
    }
}

CanNetworkManager::~CanNetworkManager() {
    // 现在访问的是成员变量，用法不变
    std::lock_guard<std::mutex> lock(mutex_); 
    for (auto const& [dev_idx, is_init] : initialized_devices_) {
        if (is_init) {
            //std::cout << "CanNetworkManager: Freeing CAN device " << (int)dev_idx << "..." << std::endl;
            //std::cout << "CanNetworkManager: Freeing CAN device " << (int)dev_idx << "..." << std::endl;
            harmonic_freeDLL(dev_idx);
            //std::cout << "CanNetworkManager: CAN device " << (int)dev_idx << " freed." << std::endl;
        }
    }
    //std::cout << "CanNetworkManager: All CAN devices freed." << std::endl;
    //initialized_devices_.clear();
    //std::cout << "CanNetworkManager: All CAN devices cleared." << std::endl;
}


// --- EuMotorNode Implementation ---
int EuMotorNode::getNodeId(){
    return (int)node_id_;
}

bool EuMotorNode::moveTo(hreal32 target_angle_deg, huint32 velocity_dps, huint32 acceleration_dpss, huint32 deceleration_dpss) {
    //TODO: Check if we're already in the correct mode
    //if (current_mode_ != harmonic_OperateMode_ProfilePosition && !switchMode(harmonic_OperateMode_ProfilePosition)) return false;
    bool isRelative = false;   // 是否是相对位置
    bool isImmediately = true; // 是否立即生效
    bool isUpdate = false;     // 是否采用更新位置模式   
    return check(harmonic_profilePositionControl(
        dev_index_, node_id_,
        angleToPulses(target_angle_deg), velocityToPulses(velocity_dps),
        accelerationToPulses(acceleration_dpss), decelerationToPulses(deceleration_dpss), isRelative, isImmediately,isUpdate), "Move To Position");
}

bool EuMotorNode::moveAt(hreal32 target_velocity_dps, huint32 acceleration_dpss,huint32 deceleration_dpss) {
    //TODO: Check if we're already in the correct mode
    //if (current_mode_ != harmonic_OperateMode_ProfileVelocity && !switchMode(harmonic_OperateMode_ProfileVelocity)) return false;
    bool isUpdate = false; // 是否采用更新位置模式
    return check(harmonic_profileVelocityControl(
        dev_index_, node_id_,
        velocityToPulses(target_velocity_dps),
        accelerationToPulses(acceleration_dpss), decelerationToPulses(deceleration_dpss),isUpdate), "Move At Velocity");
}

bool EuMotorNode::applyTorque(hint16 target_torque_milli, huint32 torque_slope) {
    //TODO: Check if we're already in the correct mode
    //if (current_mode_ != harmonic_OperateMode_ProfileTorque && !switchMode(harmonic_OperateMode_ProfileTorque)) return false;
    bool isUpdate = false; // 是否采用更新位置模式
    return check(harmonic_profileTorqueControl(
        dev_index_, node_id_,
        target_torque_milli, static_cast<hint16>(torque_slope),isUpdate), "Apply Torque");
}

EuMotorNode::EuMotorNode(huint8 devIndex, huint8 nodeId, huint32 default_timeout_ms)
    : dev_index_(devIndex), node_id_(nodeId), timeout_ms_(default_timeout_ms) {
    // Read the gear ratio on construction.
    // 老电机: 0x6091-2 (shaft revolutions) 直接就是每轴转脉冲数 (如 5308416)，非 1 即采用。
    // 新电机: 0x6091-2 为 1，改用 0x2025(编码器单圈) × (0x26A2/0x26A3 减速比)。
    motor_id_ = {dev_index_, node_id_};
    huint32 shaft_rev = 0;
    bool gear_ok = false;

    // 1) Old-motor path: read shaft revolutions (0x6091-2).
    if (check(harmonic_getGearRatioShaftRevolutions(dev_index_, node_id_, &shaft_rev, timeout_ms_), "Read Shaft Revolutions")
        && shaft_rev != 1 && shaft_rev != 0) {
        pulses_per_rev_ = shaft_rev;   // 老电机直接采用 shaft_rev
        gear_ok = true;
    } else {
        // 2) New-motor path: encoder single turn (0x2025) × gear ratio (0x26A2/0x26A3).
        huint32 enc_single_turn = 0;
        huint16 new_motor_rev = 0;
        huint16 new_shaft_rev = 0;
        int rc = harmonic_readDirectory(dev_index_, node_id_, 0x2025, 0, harmonic_DataType_uint32, &enc_single_turn, timeout_ms_);
        if (rc == HARMONIC_SUCCESS) {
            rc = harmonic_readDirectory(dev_index_, node_id_, 0x26A2, 0, harmonic_DataType_uint16, &new_motor_rev, timeout_ms_);
        }
        if (rc == HARMONIC_SUCCESS) {
            rc = harmonic_readDirectory(dev_index_, node_id_, 0x26A3, 0, harmonic_DataType_uint16, &new_shaft_rev, timeout_ms_);
        }
        if (rc == HARMONIC_SUCCESS && new_shaft_rev != 0) {
            pulses_per_rev_ = static_cast<huint32>((hreal32)enc_single_turn * (hreal32)new_motor_rev / (hreal32)new_shaft_rev);
            gear_ok = true;
        }
    }

    if (!gear_ok) {
        pulses_per_rev_ = 360000; // Fallback to a sensible default
        MotorFeedbackManager::getInstance().setGearRatio(motor_id_, pulses_per_rev_);
        std::cerr << "WARNING [Motor " << (int)node_id_ << "]: Failed to read gear ratio. Using default "
                  << pulses_per_rev_ << ". Call setGearRatio() for accuracy." << std::endl;
    }else {
        MotorFeedbackManager::getInstance().setGearRatio(motor_id_, pulses_per_rev_);
        std::cout << "INFO [Motor " << (int)node_id_ << "]: Gear ratio set to "
                  << pulses_per_rev_ << " (pulses per rev)." << std::endl;
    }
    huint32 posWindow;
    harmonic_getPositionWindow(devIndex, nodeId, &posWindow);
    std::cout << "INFO [Motor] " << (int)node_id_ << "]: original position window: " << posWindow << std::endl;
    std::cout << "INFO [Motor] " << (int)node_id_ << "]: Position window set to 0xFFFF." << std::endl;
    harmonic_setPositionWindow(devIndex, nodeId, 0x64);
    huint32 eWindow;
    harmonic_getFollowingErrorWindow(devIndex, nodeId, &eWindow);
    std::cout << "INFO [Motor] " << (int)node_id_ << "]: original error window: " << eWindow << std::endl;
    std::cout << "INFO [Motor] " << (int)node_id_ << "]: Error window set to 0xFFFFFF." << std::endl;
    harmonic_setFollowingErrorWindow(devIndex, nodeId, 0xFFFFFF);
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Initialized." << "pulses_per_rev_:"<<pulses_per_rev_<< std::endl;
}
bool EuMotorNode::check(int return_code, const std::string& operation_name) const {
    if (return_code != HARMONIC_SUCCESS) {
        std::cerr << "ERROR [Motor " << (int)node_id_ << "]: " << operation_name 
                  << " failed with code " << return_code << "." << std::endl;
        return false;
    }
    return true;
}

bool EuMotorNode::enable(harmonic_OperateMode mode) {
    if (!clearFault()) return false;
    if (!switchMode(mode)) return false;
    return true;
}

bool EuMotorNode::disable() {
    return check(harmonic_setControlword(dev_index_, node_id_, 0x06, timeout_ms_), "Disable (Shutdown)");
}

bool EuMotorNode::clearFault() {
    try {
        huint16 status = getStatusWord();
        if (status & 0x0008) {
            std::cout << "INFO [Motor " << (int)node_id_ << "]: Fault detected, attempting to reset..." << std::endl;
            check(harmonic_setControlword(dev_index_, node_id_, 0x80, timeout_ms_), "Fault Reset");
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 等待
            
            status = getStatusWord(); // 再次检查
            if (status & 0x0008) {
                std::cerr << "ERROR [Motor " << (int)node_id_ << "]: Failed to clear fault." << std::endl;
                return false;
            }
            std::cout << "INFO [Motor " << (int)node_id_ << "]: Fault cleared." << std::endl;
        }
    } catch (const std::runtime_error& e) {
        std::cerr << "ERROR [Motor " << (int)node_id_ << "]: Exception in clearFault: " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool EuMotorNode::switchMode(harmonic_OperateMode new_mode) {
    if (current_mode_ == new_mode) return true;
    
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Switching mode to " << new_mode << "..." << std::endl;
    //if (!disable()) return false; // Go to a safe, non-operational state
    
    //if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Enter_PreOperational), "Enter Pre-Op State")) return false;
    //if (!check(harmonic_setOperateMode(dev_index_, node_id_, new_mode, timeout_ms_), "Set Operate Mode")) return false;
    //if (!enableStateMachine()) return false;
    //if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Start_Node), "Enter Operational State")) return false;
    
    current_mode_ = new_mode;
    return true;
}

int EuMotorNode::resetAndStartNode(){
    if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Reset_Node),"Reset Node")) return -1;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Start_Node),"Start Node")) return -1;
    return HARMONIC_SUCCESS;
}
int EuMotorNode::enableStateMachine() {
    if (!check(harmonic_setControlword(dev_index_, node_id_, 0x06, timeout_ms_), "State Machine: Shutdown")) return -1;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!check(harmonic_setControlword(dev_index_, node_id_, 0x07, timeout_ms_), "State Machine: Switch On")) return -1;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!check(harmonic_setControlword(dev_index_, node_id_, 0x0F, timeout_ms_), "State Machine: Enable Operation")) return -1;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return 0;
}

bool EuMotorNode::setGearRatio(huint32 pulses_per_revolution) {
    if (!check(harmonic_setGearRatioShaftRevolutions(dev_index_, node_id_, pulses_per_revolution, timeout_ms_), "Set Gear Ratio")) return false;
    pulses_per_rev_ = pulses_per_revolution;
    return true;
}

bool EuMotorNode::setAsHome() {
    hint32 current_pos_pulses = angleToPulses(getPosition());
    return check(harmonic_setHomeOffset(dev_index_, node_id_, -current_pos_pulses, timeout_ms_), "Set Home Offset");
}



bool EuMotorNode::stop() {
    return check(harmonic_stopControl(dev_index_, node_id_), "Stop Control");
}


hreal32 EuMotorNode::getPosition(){ // <-- 标记为 const
    // 从缓存中获取最新的反馈数据
    MotorFeedbackData feedback = getLatestFeedback();
    
    // 可以在这里增加一个检查，看数据是否过时
    auto time_since_update = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - feedback.last_update_time
    ).count();

    // 如果数据超过 500ms 没有更新，可能意味着反馈已丢失，抛出异常
    if (time_since_update > 500) {
        //throw std::runtime_error("Feedback data for Node " + std::to_string(node_id_) + " is stale!");
        hint32 pulses;
        if (!check(harmonic_getActualPos(dev_index_, node_id_, &pulses, timeout_ms_), "Get Position")) {
            throw std::runtime_error("Failed to read position for Node " + std::to_string(node_id_));
        }
        return pulsesToAngle(pulses);
    }

    return feedback.position_deg;
}

hreal32 EuMotorNode::getVelocity() { // <-- 标记为 const
    // 从缓存中获取最新的反馈数据
    MotorFeedbackData feedback = getLatestFeedback();

    // 同样可以增加超时检查
    auto time_since_update = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - feedback.last_update_time
    ).count();

    if (time_since_update > 500) {
        //throw std::runtime_error("Feedback data for Node " + std::to_string(node_id_) + " is stale!");
        hint32 pps;
        if (!check(harmonic_getActualVelocity(dev_index_, node_id_, &pps, timeout_ms_), "Get Velocity")) {
            throw std::runtime_error("Failed to read velocity for Node " + std::to_string(node_id_));
        }
        return pulsesToVelocity(pps);
    }

    return feedback.velocity_dps;
}

hint16 EuMotorNode::getTorque() {
    hint16 torque;
    if (!check(harmonic_getActualTorque(dev_index_, node_id_, &torque, timeout_ms_), "Get Torque")) {
        throw std::runtime_error("Failed to read torque for Node " + std::to_string(node_id_));
    }
    return torque;
}

huint16 EuMotorNode::getStatusWord() {
    huint16 word;
    if (!check(harmonic_getStatusWord(dev_index_, node_id_, &word, timeout_ms_), "Get Status Word")) {
        throw std::runtime_error("Failed to read Status Word for Node " + std::to_string(node_id_));
    }
    return word;
}

huint16 EuMotorNode::getErrorCode() {
    huint16 err;
    // 注意：这里不再需要 check() 了，因为如果读取失败，下面的函数会直接抛出异常。
    // 我们用一个 try-catch 块来捕获它，并返回一个表示通信失败的值（或者重新抛出）。
    try {
        if (harmonic_getServoErrorCode(dev_index_, node_id_, &err, timeout_ms_) != HARMONIC_SUCCESS) {
            // 如果函数返回错误但没有抛出（这种情况不应该发生，但为了安全），我们手动抛出
            throw std::runtime_error("Failed to read Error Code (harmonic call failed)");
        }
    } catch (const std::runtime_error& e) {
        // 如果 check() （或者它内部的函数）抛出异常，我们在这里捕获
        std::cerr << "COMM_ERROR [Motor " << (int)node_id_ << "]: " << e.what() << std::endl;
        // 重新抛出一个更明确的异常，表明是通信问题
        throw std::runtime_error("Communication failed while getting error code for Node " + std::to_string(node_id_));
    }
    return err;
}

harmonic_OperateMode EuMotorNode::getOperationMode() {
    harmonic_OperateMode mode;
    if (!check(harmonic_getDisplayOperateMode(dev_index_, node_id_, &mode, timeout_ms_), "Get Operation Mode")) {
        throw std::runtime_error("Failed to read Operation Mode for Node " + std::to_string(node_id_));
    }
    current_mode_ = mode;
    return mode;
}

// Unit conversion implementations
hint32 EuMotorNode::angleToPulses(hreal32 angle_deg) const {
    return static_cast<hint32>((angle_deg / 360.0f) * pulses_per_rev_);
}
hreal32 EuMotorNode::pulsesToAngle(hint32 pulses) const {
    if (pulses_per_rev_ == 0) return 0.0f;
    return (static_cast<hreal32>(pulses) / pulses_per_rev_) * 360.0f;
}
hint32 EuMotorNode::velocityToPulses(hreal32 dps) const {
    return static_cast<hint32>((dps / 360.0f) * pulses_per_rev_);
}
hreal32 EuMotorNode::pulsesToVelocity(hint32 pps) const {
     if (pulses_per_rev_ == 0) return 0.0f;
    return (static_cast<hreal32>(pps) / pulses_per_rev_) * 360.0f;
}
huint32 EuMotorNode::accelerationToPulses(huint32 dpss) const {
    return static_cast<huint32>((static_cast<hreal32>(dpss) / 360.0f) * pulses_per_rev_);
}

huint32 EuMotorNode::decelerationToPulses(huint32 dpss) const {
    return static_cast<huint32>((static_cast<hreal32>(dpss) / 360.0f) * pulses_per_rev_);
}


// Explicit template instantiations to avoid linker errors if definitions are in .cpp
//template bool EuMotorNode::write<huint32>(huint16, huint8, huint32);
//template huint32 EuMotorNode::read<huint32>(huint16, huint8);
// ... add more instantiations as needed
bool EuMotorNode::configureCspMode(huint16 pdo_index, bool use_sync) {
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring for CSP mode..." << std::endl;
    int itpv = 4;        // 插补周期，单位ms
    // Determine RPDO transmit type based on sync flag
    // 0x01: Synchronous cyclic. 0xFF: Asynchronous, event-driven.
    huint8 rpdo_transmit_type = use_sync ? 0x01 : 0xFF;
    // Switch to pre-op for configuration
    //if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Enter_PreOperational), "CSP: Enter Pre-Op")) return false;
    if (!check(harmonic_setOperateMode(dev_index_, node_id_, harmonic_OperateMode_CyclicSyncPosition),"CSP: Switch to CSP")) return false;
    if (!check(harmonic_setInterpolationTimePeriodValue(dev_index_, node_id_, itpv),"CSP: Set ITPV")) return false;
    if (!check(harmonic_setSyncCounter(dev_index_, node_id_, 0),"CSP: Set Sync Counter")) return false;
    if (!check(harmonic_setRPDOCobId(dev_index_, node_id_, 0, (0x80 << 24) + 0x200 + node_id_),"CSP: Set RPDO COB-ID")) return false;

    // 2. Map the RPDO to the target position object (0x607A)
    // First, disable mapping by setting count to 0
    if (!check(harmonic_setRPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 0), "CSP: Clear RPDO Map")) return false;
        // 1. Configure RPDO communication type to be synchronous
    // 0x01 means synchronous cyclic
    if (!check(harmonic_setRPDOTransmitType(dev_index_, node_id_, pdo_index, rpdo_transmit_type), "CSP: Set RPDO Type")) return false;

    // Map Target Position (0x607A), 32 bits (0x20)
    huint32 mapping_value = (0x607A << 16) + 0x0020;
    if (!check(harmonic_setRPDOMapped(dev_index_, node_id_, pdo_index, 0, mapping_value), "CSP: Set RPDO Map")) return false;
    
    // Now, enable mapping by setting count to 1
    if (!check(harmonic_setRPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 1), "CSP: Set RPDO Map Count")) return false;

    if (!check(harmonic_setRPDOCobId(dev_index_, node_id_, 0, 0x200 + node_id_), "CSP: Set RPDO CobId")) return false;

    if (!check(resetAndStartNode(),"CSP: Reset and Start Node")) return false;

    // Reset may restore drive parameters, so apply and verify the safety ceiling last,
    // immediately before enabling operation.
    if (gripper_controller_.isEnabled()) {
        const hint16 requested_limit = gripper_controller_.config().torque_limit_milli;
        if (!setTorqueLimit(requested_limit)) return false;
        try {
            if (getTorqueLimit() != requested_limit) {
                std::cerr << "ERROR [Motor " << (int)node_id_ << "]: Gripper torque limit readback mismatch." << std::endl;
                return false;
            }
        } catch (const std::runtime_error&) {
            return false;
        }
    }

    if (!check(enableStateMachine(),"CSP: Enable State Machine")) return false;
    // Finally, set the mode
    current_mode_ = harmonic_OperateMode_CyclicSyncPosition;
    return true;
}

int EuMotorNode::sendCspTargetPosition(hreal32 target_angle_deg, huint16 pdo_index, bool isSync) {
    huint16 rpdo_base_cobid = 0x200; // RPDO1 base
    if (pdo_index > 0) {
        rpdo_base_cobid += (pdo_index * 0x100);
    }
    
    const GripperControlResult gripper_result = gripper_controller_.process(
        target_angle_deg, getLatestFeedback(), std::chrono::steady_clock::now());
    last_csp_effective_target_deg_ = gripper_result.target_position_deg;
    hint32 pos_pulses = angleToPulses(gripper_result.target_position_deg);
    
    huint8 data[4];
    data[0] = pos_pulses & 0xFF;
    data[1] = (pos_pulses >> 8) & 0xFF;
    data[2] = (pos_pulses >> 16) & 0xFF;
    data[3] = (pos_pulses >> 24) & 0xFF;
    
    int result = harmonic_writeCanData(dev_index_, rpdo_base_cobid + node_id_, data, 4);
    if (isSync){
        sendSync();
    }
    return result;
}

/**
 * @brief Configures the motor for Cyclic Sync Torque (CST) mode.
 * This function sets up the necessary PDOs for real-time torque control.
 * It follows the logic from the test_cst_mode.cpp example.
 */
bool EuMotorNode::configureCstMode(huint8 interpolation_period_ms, huint16 pdo_index,bool use_sync) {
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring for CST mode..." << std::endl;

    // Determine RPDO transmit type based on sync flag
    // 0x01: Synchronous cyclic. 0xFF: Asynchronous, event-driven.
    huint8 rpdo_transmit_type = use_sync ? 0x01 : 0xFF;
    // Switch to pre-operational state for configuration
    //if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Enter_PreOperational), "CST: Enter Pre-Op")) return false;
    if (!check(harmonic_setOperateMode(dev_index_, node_id_, harmonic_OperateMode_CyclicSyncTorque),"CST: Set Mode")) return false;
    // 1. Set interpolation time period (Object 0x60C2, Sub-index 1)
    if (!check(harmonic_setInterpolationTimePeriodValue(dev_index_, node_id_, interpolation_period_ms), "CST: Set Interpolation Time")) return false;
    if (!check(harmonic_setSyncCounter(dev_index_, node_id_, 0),"CSP: Set Sync Counter")) return false;
    if (!check(harmonic_setRPDOCobId(dev_index_, node_id_, 0, (0x80 << 24) + 0x200 + node_id_),"CSP: Set RPDO COB-ID")) return false;
    // First, disable mapping by setting the count to 0
    if (!check(harmonic_setRPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 0), "CST: Clear RPDO Map")) return false;
    
    // 2. Configure RPDO communication type to be synchronous cyclic (Type 1)
    // This makes the motor wait for a SYNC message before applying the received torque value.
    if (!check(harmonic_setRPDOTransmitType(dev_index_, node_id_, pdo_index, rpdo_transmit_type), "CST: Set RPDO Type to Sync")) return false;

    // 3. Map the RPDO to the target torque object (0x6071)

    // Map Target Torque (Object 0x6071), 16 bits long (0x10)
    // Note: The example code sends 4 bytes, but object 6071h is typically a 16-bit integer (hint16).
    // We will stick to the 16-bit standard which is more common.
    huint32 mapping_value = (0x6071 << 16) + 0x0010; 
    if (!check(harmonic_setRPDOMapped(dev_index_, node_id_, pdo_index, 0, mapping_value), "CST: Set RPDO Map to Target Torque")) return false;
    
    // Now, enable mapping by setting the count back to 1
    if (!check(harmonic_setRPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 1), "CST: Set RPDO Map Count")) return false;

    if (!check(harmonic_setRPDOCobId(dev_index_, node_id_, 0, 0x200 + node_id_), "CSP: Set RPDO CobId")) return false;

    if (!check(resetAndStartNode(),"CSP: Reset and Start Node")) return false;

    if (!check(enableStateMachine(),"CSP: Enable State Machine")) return false;
    // 4. Set the final operation mode and enable the state machine.
    // The switchMode function will handle setting the mode and enabling the controlword state machine.
    current_mode_ = harmonic_OperateMode_CyclicSyncTorque;
    return true;
}

/**
 * @brief Sends the target torque value for CST mode using a direct CAN write.
 * This should be called in a real-time loop, followed by a sendSync() call.
 */
void EuMotorNode::sendCstTargetTorque(hint16 target_torque, huint16 pdo_index, bool isSync) {
    // Determine the COB-ID for the RPDO.
    // Default RPDOs are at 0x200, 0x300, 0x400, 0x500 for RPDO 1, 2, 3, 4.
    // So, RPDO index 0 corresponds to RPDO1.
    huint32 rpdo_base_cobid = 0x200 + (pdo_index * 0x100);
    huint32 cob_id = rpdo_base_cobid + node_id_;

    // Prepare the 2-byte payload for the 16-bit torque value (little-endian)
    huint8 data[2];
    data[0] = target_torque & 0xFF;
    data[1] = (target_torque >> 8) & 0xFF;
    
    // Write the data directly to the CAN bus. This is a non-blocking, non-checked call
    // for real-time performance.
    harmonic_writeCanData(dev_index_, cob_id, data, 2);
    if (isSync){
        sendSync();
    }
}

/**
 * @brief Configures the motor for Cyclic Sync Velocity (CSV) mode.
 * This function sets up the necessary PDOs for real-time velocity control.
 * It follows the logic from the test_csv_mode.cpp example.
 */
bool EuMotorNode::configureCsvMode(huint8 interpolation_period_ms, huint16 pdo_index, bool use_sync) {
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring for CSV mode..." << std::endl;
    // Determine RPDO transmit type based on sync flag
    // 0x01: Synchronous cyclic. 0xFF: Asynchronous, event-driven.
    huint8 rpdo_transmit_type = use_sync ? 0x01 : 0xFF;
    // Switch to pre-operational state for configuration
    //if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Enter_PreOperational), "CSV: Enter Pre-Op")) return false;
    if (!check(harmonic_setOperateMode(dev_index_, node_id_, harmonic_OperateMode_CyclicSyncVelocity),"CSV: Set Mode")) return false;    
    // 1. Set interpolation time period (Object 0x60C2, Sub-index 1)
    if (!check(harmonic_setInterpolationTimePeriodValue(dev_index_, node_id_, interpolation_period_ms), "CSV: Set Interpolation Time")) return false;
    if (!check(harmonic_setSyncCounter(dev_index_, node_id_, 0),"CSP: Set Sync Counter")) return false;
    if (!check(harmonic_setRPDOCobId(dev_index_, node_id_, 0, (0x80 << 24) + 0x200 + node_id_),"CSV: Set RPDO COB-ID")) return false;
    // First, disable mapping by setting the count to 0
    if (!check(harmonic_setRPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 0), "CSV: Clear RPDO Map")) return false;
    
    // 2. Configure RPDO communication type to be synchronous cyclic (Type 1)
    if (!check(harmonic_setRPDOTransmitType(dev_index_, node_id_, pdo_index, rpdo_transmit_type), "CSV: Set RPDO Type to Sync")) return false;


    // Map Target Velocity (Object 0x60FF), 32 bits long (0x20)
    huint32 mapping_value = (0x60FF << 16) + 0x0020; 
    if (!check(harmonic_setRPDOMapped(dev_index_, node_id_, pdo_index, 0, mapping_value), "CSV: Set RPDO Map to Target Velocity")) return false;
    
    // Now, enable mapping by setting the count back to 1
    if (!check(harmonic_setRPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 1), "CSV: Set RPDO Map Count")) return false;

    if (!check(harmonic_setRPDOCobId(dev_index_, node_id_, 0, 0x200 + node_id_), "CSV: Set RPDO CobId")) return false;

    if (!check(resetAndStartNode(),"CSV: Reset and Start Node")) return false;

    if (!check(enableStateMachine(),"CSV: Enable State Machine")) return false;
    // 4. Set the final operation mode and enable the state machine.
    // The switchMode function handles setting the mode and enabling the controlword state machine.
    current_mode_ = harmonic_OperateMode_CyclicSyncVelocity;
    return true;
}

/**
 * @brief Sends the target velocity value for CSV mode using a direct CAN write.
 * This should be called in a real-time loop, followed by a sendSync() call.
 */
void EuMotorNode::sendCsvTargetVelocity(hreal32 target_velocity_dps, huint16 pdo_index,bool isSync) {
    // Determine the COB-ID for the RPDO.
    // Default RPDOs are at 0x200, 0x300, 0x400, 0x500 for RPDO 1, 2, 3, 4.
    huint32 rpdo_base_cobid = 0x200 + (pdo_index * 0x100);
    huint32 cob_id = rpdo_base_cobid + node_id_;

    // Convert the user-friendly degrees/sec to device-specific pulses/sec
    hint32 vel_pulses = velocityToPulses(target_velocity_dps);

    // Prepare the 4-byte payload for the 32-bit velocity value (little-endian)
    huint8 data[4];
    data[0] = vel_pulses & 0xFF;
    data[1] = (vel_pulses >> 8) & 0xFF;
    data[2] = (vel_pulses >> 16) & 0xFF;
    data[3] = (vel_pulses >> 24) & 0xFF;
    
    // Write the data directly to the CAN bus for real-time performance.
    harmonic_writeCanData(dev_index_, cob_id, data, 4);
    if (isSync){
        sendSync();
    }
}

/**
 * @brief Configures the motor for Interpolated Position (IP) mode.
 * This function sets up the necessary PDOs for real-time position control via interpolation.
 * It follows the logic from the test_ip_mode.cpp example.
 */
bool EuMotorNode::configureIpMode(huint8 interpolation_period_ms, huint16 pdo_index, bool use_sync) {
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring for IP mode..." << std::endl;

    // Determine RPDO transmit type based on sync flag
    // 0x01: Synchronous cyclic. 0xFF: Asynchronous, event-driven.
    huint8 rpdo_transmit_type = use_sync ? 0x01 : 0xFF;

    // The example code doesn't switch to Pre-Op, but it's good practice.
    // However, we will follow the example's flow which configures directly.
    if (!check(harmonic_setOperateMode(dev_index_, node_id_, harmonic_OperateMode_InterpolatedPosition), "IP: Set Mode")) return false;
    
    // 1. Set interpolation time period (Object 0x60C2, Sub-index 1)
    if (!check(harmonic_setInterpolationTimePeriodValue(dev_index_, node_id_, interpolation_period_ms), "IP: Set Interpolation Time")) return false;
    
    // The following settings might be optional or default, but we include them for completeness from the example
    if (!check(harmonic_setSyncCounter(dev_index_, node_id_, 0), "IP: Set Sync Counter")) return false;
    
    // 2. Temporarily disable the RPDO for configuration
    if (!check(harmonic_setRPDOCobId(dev_index_, node_id_, pdo_index, (0x80 << 24) + 0x200 + node_id_), "IP: Disable RPDO")) return false;
    
    // 3. Clear previous mappings
    if (!check(harmonic_setRPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 0), "IP: Clear RPDO Map")) return false;

    // 4. Set the RPDO transmission type
    if (!check(harmonic_setRPDOTransmitType(dev_index_, node_id_, pdo_index, rpdo_transmit_type), "IP: Set RPDO Type")) return false;

    // 5. Map the RPDO to the Interpolation data record (0x60C1, sub-index 1)
    // The example uses (0x60C1 << 16) + 0x0120. 0x01 is sub-index, 0x20 is length (32 bits).
    // The harmonic_setRPDOMapped function likely handles the sub-index internally based on mapIndex,
    // so we only need to provide the main index and length.
    // The object 0x60C1 sub 1 is "Interpolation data 1" which is a 32-bit position value.
    huint32 mapping_value = (0x60C1 << 16) + 0x0120; // Following example exactly
    if (!check(harmonic_setRPDOMapped(dev_index_, node_id_, pdo_index, 0, mapping_value), "IP: Set RPDO Map")) return false;
    
    // 6. Enable the mapping by setting the count to 1
    if (!check(harmonic_setRPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 1), "IP: Set RPDO Map Count")) return false;

    // 7. Reset and start the node to apply settings
    if (!check(resetAndStartNode(),"IP: Reset and Start Node")) return false;

    // 8. Re-enable the RPDO with the correct, active COB-ID
    if (!check(harmonic_setRPDOCobId(dev_index_, node_id_, pdo_index, 0x200 + node_id_), "IP: Enable RPDO")) return false;

    // 9. Go through the standard state machine
    if (!check(enableStateMachine(),"IP Enable State Machine")) return false;    
    
    // 10. IP mode requires an extra control word (0x1F) to start the interpolator
    if (!check(harmonic_setControlword(dev_index_, node_id_, 0x1F, timeout_ms_), "IP: Start Interpolator")) return false;

    current_mode_ = harmonic_OperateMode_InterpolatedPosition;
    return true;
}


/**
 * @brief Sends the target position for IP mode using a direct CAN write.
 */
int EuMotorNode::sendIpTargetPosition(hreal32 target_angle_deg, huint16 pdo_index,bool isSync) {
    // Determine the COB-ID for the RPDO.
    huint32 rpdo_base_cobid = 0x200 + (pdo_index * 0x100);
    huint32 cob_id = rpdo_base_cobid + node_id_;

    // Convert the user-friendly degrees to device-specific pulses
    hint32 pos_pulses = angleToPulses(target_angle_deg);

    // Prepare the 4-byte payload for the 32-bit position value (little-endian)
    huint8 data[4];
    data[0] = pos_pulses & 0xFF;
    data[1] = (pos_pulses >> 8) & 0xFF;
    data[2] = (pos_pulses >> 16) & 0xFF;
    data[3] = (pos_pulses >> 24) & 0xFF;
    
    // Write the data directly to the CAN bus.
    int result = harmonic_writeCanData(dev_index_, cob_id, data, 4);

    // If using synchronous mode, a separate SYNC message is required.
    // The example code's setPos function sends it, so we replicate that behavior.
    if (isSync) {
        sendSync();
    }
    return result;
}
void EuMotorNode::sendSync() {
    huint8 data[1] = {0};
    harmonic_writeCanData(dev_index_, 0x80, data, 1);
}

bool EuMotorNode::startAutoFeedback(huint16 pdo_index, huint8 transmit_type, huint16 event_timer_ms) {
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring automatic feedback (TPDO" << pdo_index + 1 << ")..." << std::endl;
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring automatic feedback transmit type "<< (int)transmit_type << " ..." << std::endl;

    // TPDOs must be configured in Pre-Operational state
    if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Enter_PreOperational), "Feedback: Enter Pre-Op")) return false;

    // 1. Disable the TPDO by setting its COB-ID's valid bit (bit 31) to 1
    huint32 tpd_cobid = (0x180 + (0x100 * pdo_index) + node_id_);
    if (!check(harmonic_setTPDOCobId(dev_index_, node_id_, pdo_index, tpd_cobid | 0x80000000), "Feedback: Disable TPDO")) return false;

    // 2. Clear existing mappings by setting the map count to 0
    if (!check(harmonic_setTPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 0), "Feedback: Clear TPDO Map")) return false;

    // 3. Map the desired objects (Total size must be <= 8 bytes)
    // Mapping format: (Index << 16) | (SubIndex << 8) | (DataLength in bits)
    huint32 pos_mapping = (0x6064 << 16) | (0x00 << 8) | 32;    // Actual Position (4 bytes)
    huint32 torque_mapping = (0x6077 << 16) | (0x00 << 8) | 16; // Actual Torque (2 bytes)
    huint32 status_mapping = (0x6041 << 16) | (0x00 << 8) | 16; // Status Word (2 bytes)

    if (!check(harmonic_setTPDOMapped(dev_index_, node_id_, pdo_index, 0, pos_mapping), "Feedback: Map Position")) return false;
    if (!check(harmonic_setTPDOMapped(dev_index_, node_id_, pdo_index, 1, torque_mapping), "Feedback: Map Torque")) return false;
    if (!check(harmonic_setTPDOMapped(dev_index_, node_id_, pdo_index, 2, status_mapping), "Feedback: Map Status Word")) return false;
    // 4. Set the number of mapped objects
    if (!check(harmonic_setTPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 3), "Feedback: Set TPDO Map Count")) return false;

    // 4b. Configure the master-side local RPDO to accept this TPDO (mirror of test_pdo.cpp).
    // 若不配置，主站（CANable）不会把 TPDO 帧上抛给接收回调，反馈将收不到。
    check(harmonic_setLocalRPDOCobId(pdo_index, tpd_cobid), "Feedback: Local RPDO COB-ID");
    check(harmonic_setLocalRPDOMaxMappedCount(pdo_index, 0), "Feedback: Clear Local RPDO Map");
    check(harmonic_setLocalRPDOMapped(pdo_index, 0, pos_mapping), "Feedback: Local RPDO Map Pos");
    check(harmonic_setLocalRPDOMapped(pdo_index, 1, torque_mapping), "Feedback: Local RPDO Map Torque");
    check(harmonic_setLocalRPDOMapped(pdo_index, 2, status_mapping), "Feedback: Local RPDO Map Status");
    check(harmonic_setLocalRPDOMaxMappedCount(pdo_index, 3), "Feedback: Set Local RPDO Map Count");

    // 5. Set the transmission type
    // 0 = Synchronous,event driven
    // 1 = Synchronous,periodic
    // 2-253 = Synchronous, send per n syncs
    // 254 = Event-driven, on change. 255 = Event-driven, asynchronous. 1-240 = Synchronous on SYNC.
    if (!check(harmonic_setTPDOTransmitType(dev_index_, node_id_, pdo_index, transmit_type), "Feedback: Set Transmit Type")) return false;

    // If event-driven, set the event timer (minimum time between transmissions)
    if (transmit_type >= 254) {
        if (!check(harmonic_setTPDOEventTimer(dev_index_, node_id_, pdo_index, event_timer_ms), "Feedback: Set Event Timer")) return false;
    }

    // 6. Re-enable the TPDO by clearing the valid bit
    if (!check(harmonic_setTPDOCobId(dev_index_, node_id_, pdo_index, tpd_cobid), "Feedback: Enable TPDO")) return false;

    // 7. Return to operational state
    if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Start_Node), "Feedback: Enter Operational")) return false;

    std::cout << "INFO [Motor " << (int)node_id_ << "]: Feedback configured successfully." << std::endl;
    return true;
}

// 在 eu_motor.cpp 中
bool EuMotorNode::startErrorFeedbackTPDO(huint16 pdo_index, huint8 transmit_type, huint16 event_timer_ms) {
    // 确保 pdo_index > 0，因为 TPDO1 (index 0) 通常用于反馈位置/速度
    if (pdo_index == 0) {
        std::cerr << "ERROR [Motor " << (int)node_id_ << "]: TPDO index for error feedback should be > 0 (e.g., 1 for TPDO2)." << std::endl;
        return false;
    }

    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring error feedback on TPDO" << pdo_index + 1 << "..." << std::endl;

    try {
        // TPDOs 必须在 Pre-Operational 状态下配置
        check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Enter_PreOperational), "ErrorFeedback: Enter Pre-Op");

        // 1. 禁用TPDO以便配置
        huint32 tpd_cobid = (0x180 + (0x100 * pdo_index) + node_id_);
        check(harmonic_setTPDOCobId(dev_index_, node_id_, pdo_index, tpd_cobid | 0x80000000), "ErrorFeedback: Disable TPDO");

        // 2. 清除现有映射
        check(harmonic_setTPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 0), "ErrorFeedback: Clear TPDO Map");

        // 3. 映射我们需要的对象: Statusword (6041h) 和 Error Code (603Fh)
        // 映射格式: (Index << 16) | (SubIndex << 8) | (数据长度，单位bit)
        huint32 statusword_mapping = (0x6041 << 16) | (0x00 << 8) | 16; // Statusword, 16 bits (2 bytes)
        huint32 errorcode_mapping  = (0x603F << 16) | (0x00 << 8) | 16; // Error Code, 16 bits (2 bytes)

        check(harmonic_setTPDOMapped(dev_index_, node_id_, pdo_index, 0, statusword_mapping), "ErrorFeedback: Map Statusword");
        check(harmonic_setTPDOMapped(dev_index_, node_id_, pdo_index, 1, errorcode_mapping), "ErrorFeedback: Map Error Code");

        // 4. 设置映射对象的数量
        check(harmonic_setTPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 2), "ErrorFeedback: Set TPDO Map Count");

        // 5. 设置传输类型 (254/255 是事件驱动，当数值变化时发送)
        check(harmonic_setTPDOTransmitType(dev_index_, node_id_, pdo_index, transmit_type), "ErrorFeedback: Set Transmit Type");

        // 6. 如果是事件驱动，设置事件定时器以限制发送频率
        if (transmit_type >= 254) {
            check(harmonic_setTPDOEventTimer(dev_index_, node_id_, pdo_index, event_timer_ms), "ErrorFeedback: Set Event Timer");
        }

        // 7. 重新启用TPDO
        check(harmonic_setTPDOCobId(dev_index_, node_id_, pdo_index, tpd_cobid), "ErrorFeedback: Enable TPDO");

        // 8. 返回操作状态
        check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Start_Node), "ErrorFeedback: Enter Operational");

    } catch (const std::runtime_error& e) {
        std::cerr << "FATAL [Motor " << (int)node_id_ << "]: Failed to configure error feedback TPDO. Reason: " << e.what() << std::endl;
        return false;
    }

    std::cout << "INFO [Motor " << (int)node_id_ << "]: Error feedback TPDO configured successfully." << std::endl;
    return true;
}

/**
 * @brief 打印当前 TPDO 配置（COB-ID、传输类型、事件定时器等）与映射表，用于调试。
 * 遍历 TPDO1-4，读取从站 0x1800+pdoIndex 与 0x1A00+pdoIndex 相关对象并打印到 stdout。
 */
void EuMotorNode::printTpdoConfig() {
    std::cout << "=== TPDO Configuration for Motor " << (int)node_id_ << " ===" << std::endl;
    for (huint16 pdo = 0; pdo < 4; ++pdo) {
        std::cout << "--- TPDO" << (pdo + 1) << " (comm 0x" << std::hex << std::uppercase << (0x1800 + pdo)
                  << ", map 0x" << (0x1A00 + pdo) << ")" << std::dec << std::nouppercase << " ---" << std::endl;

        huint8 paras_count = 0;
        if (check(harmonic_getTPDOMaxParasCount(dev_index_, node_id_, pdo, &paras_count, timeout_ms_), "TPDO Max Paras Count")) {
            std::cout << "  Max Paras Count: " << (int)paras_count << std::endl;
        }

        huint32 cob_id = 0;
        if (check(harmonic_getTPDOCobId(dev_index_, node_id_, pdo, &cob_id, timeout_ms_), "TPDO COB-ID")) {
            bool valid = !(cob_id & 0x80000000);
            std::cout << "  COB-ID: 0x" << std::hex << std::uppercase << (cob_id & 0x7FFFFFFF) << std::dec
                      << std::nouppercase << " (" << (valid ? "valid" : "invalid/disabled") << ")" << std::endl;
        }

        huint8 transmit_type = 0;
        if (check(harmonic_getTPDOTransmitType(dev_index_, node_id_, pdo, &transmit_type, timeout_ms_), "TPDO Transmit Type")) {
            std::cout << "  Transmit Type: " << (int)transmit_type << std::endl;
        }

        huint16 inhibit_time = 0;
        if (check(harmonic_getTPDOInhibitTime(dev_index_, node_id_, pdo, &inhibit_time, timeout_ms_), "TPDO Inhibit Time")) {
            std::cout << "  Inhibit Time: " << inhibit_time << " (x100us)" << std::endl;
        }

        huint16 event_timer = 0;
        if (check(harmonic_getTPDOEventTimer(dev_index_, node_id_, pdo, &event_timer, timeout_ms_), "TPDO Event Timer")) {
            std::cout << "  Event Timer: " << event_timer << " ms" << std::endl;
        }

        huint8 sync_start = 0;
        if (check(harmonic_getTPDOSYNCStartValue(dev_index_, node_id_, pdo, &sync_start, timeout_ms_), "TPDO SYNC Start")) {
            std::cout << "  SYNC Start Value: " << (int)sync_start << std::endl;
        }

        huint8 map_count = 0;
        if (check(harmonic_getTPDOMaxMappedCount(dev_index_, node_id_, pdo, &map_count, timeout_ms_), "TPDO Map Count")) {
            std::cout << "  Mapped Objects: " << (int)map_count << std::endl;
            for (huint8 i = 0; i < map_count; ++i) {
                huint32 mapping = 0;
                if (check(harmonic_getTPDOMapped(dev_index_, node_id_, pdo, i, &mapping, timeout_ms_), "TPDO Map Entry")) {
                    huint16 obj_index = static_cast<huint16>(mapping >> 16);
                    huint8  obj_sub   = static_cast<huint8>((mapping >> 8) & 0xFF);
                    huint8  obj_len   = static_cast<huint8>(mapping & 0xFF);
                    std::cout << "    [" << (int)i << "] 0x" << std::hex << std::uppercase << obj_index
                              << std::dec << std::nouppercase
                              << " sub " << (int)obj_sub << ", " << (int)obj_len << " bits" << std::endl;
                }
            }
        }
    }
    std::cout << "=== End TPDO Configuration ===" << std::endl;
}


// --- Position Loop Gains ---
bool EuMotorNode::setPositionGains(huint16 kp, huint16 ki) {
    if (!setPositionKp(kp)) return false;
    if (!setPositionKi(ki)) return false;
    return true;
}

bool EuMotorNode::setPositionKp(huint16 kp) {
    // 调用库函数设置位置环 Kp (对象 0x2013, 子索引 3)
    return check(harmonic_setServoPositionLoopKP(dev_index_, node_id_, kp, timeout_ms_), "Set Position Kp");
}

bool EuMotorNode::setPositionKi(huint16 ki) {
    // 调用库函数设置位置环 Ki (对象 0x2013, 子索引 4)
    return check(harmonic_setServoPositionLoopKI(dev_index_, node_id_, ki, timeout_ms_), "Set Position Ki");
}

// --- Velocity Loop Gains ---
bool EuMotorNode::setVelocityGains(huint16 kp, huint16 ki) {
    if (!setVelocityKp(kp)) return false;
    if (!setVelocityKi(ki)) return false;
    return true;
}

bool EuMotorNode::setVelocityKp(huint16 kp) {
    // 调用库函数设置速度环 Kp (对象 0x2012, 子索引 3)
    return check(harmonic_setServoVelocityLoopKP(dev_index_, node_id_, kp, timeout_ms_), "Set Velocity Kp");
}

bool EuMotorNode::setVelocityKi(huint16 ki) {
    // 调用库函数设置速度环 Ki (对象 0x2012, 子索引 4)
    return check(harmonic_setServoVelocityLoopKI(dev_index_, node_id_, ki, timeout_ms_), "Set Velocity Ki");
}

// --- Current Loop Gains ---
bool EuMotorNode::setCurrentGains(huint16 kp, huint16 ki) {
    if (!setCurrentKp(kp)) return false;
    if (!setCurrentKi(ki)) return false;
    return true;
}

bool EuMotorNode::setCurrentKp(huint16 kp) {
    // 调用库函数设置电流环 Kp (对象 0x2010, 子索引 3)
    return check(harmonic_setServoCurrentLoopKP(dev_index_, node_id_, kp, timeout_ms_), "Set Current Kp");
}

bool EuMotorNode::setCurrentKi(huint16 ki) {
    // 调用库函数设置电流环 Ki (对象 0x2010, 子索引 4)
    return check(harmonic_setServoCurrentLoopKI(dev_index_, node_id_, ki, timeout_ms_), "Set Current Ki");
}

bool EuMotorNode::setGripperConfig(const GripperConfig& config) {
    if (!gripper_controller_.configure(config)) {
        std::cerr << "ERROR [Motor " << (int)node_id_ << "]: Invalid gripper configuration." << std::endl;
        return false;
    }
    return true;
}

void EuMotorNode::disableGripperMode() { gripper_controller_.disable(); }
bool EuMotorNode::isGripperMode() const { return gripper_controller_.isEnabled(); }
GripperState EuMotorNode::getGripperState() const { return gripper_controller_.state(); }
bool EuMotorNode::isGripDetected() const { return gripper_controller_.state() == GripperState::Hold; }
hreal32 EuMotorNode::getGripPosition() const { return gripper_controller_.gripPosition(); }
hint16 EuMotorNode::getHoldTorqueError() const { return gripper_controller_.holdTorqueError(); }
hreal32 EuMotorNode::getLastCspEffectiveTargetPosition() const { return last_csp_effective_target_deg_; }
void EuMotorNode::clearGripperSafeStop() { gripper_controller_.clearSafeStop(); }

bool EuMotorNode::setTorqueLimit(hint16 torque_milli) {
    return check(harmonic_setTorqueLimit(dev_index_, node_id_, torque_milli, timeout_ms_), "Set Torque Limit");
}

hint16 EuMotorNode::getTorqueLimit() {
    hint16 limit = 0;
    if (!check(harmonic_getTorqueLimit(dev_index_, node_id_, &limit, timeout_ms_), "Get Torque Limit")) {
        throw std::runtime_error("Failed to read Torque Limit for Node " + std::to_string(node_id_));
    }
    return limit;
}

MotorFeedbackData EuMotorNode::getLatestFeedback(){
    MotorFeedbackManager& feedback_manager_= MotorFeedbackManager::getInstance();
    return feedback_manager_.getFeedback(motor_id_);
}

// --- MotorFeedbackManager Implementation ---

MotorFeedbackManager& MotorFeedbackManager::getInstance() {
    static MotorFeedbackManager instance;
    return instance;
}

void MotorFeedbackManager::registerCallback() {
    harmonic_setReceiveDataCallBack(validCanRecvCallback);
}

MotorFeedbackData MotorFeedbackManager::getFeedback(const MotorIdentifier& motor_id) {
    // Access member mutex
    std::lock_guard<std::mutex> lock(mutex_); 
    if (feedback_data_.count(motor_id)) {
        return feedback_data_[motor_id];
    }
    return MotorFeedbackData{};
}

void emptyCanRecvCallback(int devIndex, const harmonic_CanMsg* frame){
    //std::cout << "INFO [MotorFeedbackManager]: empty callback" << std::endl;
}

void validCanRecvCallback(int devIndex, const harmonic_CanMsg* frame){
    //std::cout << "INFO [MotorFeedbackManager]: valid callback" << std::endl;
    MotorFeedbackManager::canRecvCallback(devIndex, frame);
}
void MotorFeedbackManager::canRecvCallback(int devIndex, const harmonic_CanMsg* frame) {
    // This is a static function, so it needs to get the instance to access members
    MotorFeedbackManager& instance = getInstance();

    // === DEBUG: dump the raw CAN frame ===
    //std::cout << "DBG [CAN] dev=" << (int)devIndex
    //          << " cob_id=0x" << std::hex << std::uppercase << frame->cob_id
    //          << " len=" << std::dec << (int)frame->len << " data=";
    //for (huint8 i = 0; i < frame->len; ++i) {
    //    std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
    //              << (int)frame->data[i] << std::setfill(' ');
    //    if (i + 1 < frame->len) std::cout << " ";
    //}
    //std::cout << std::dec << std::nouppercase << std::endl;

    huint32 function_code = frame->cob_id & 0xFF80;
    huint8 node_id = frame->cob_id & 0x007F;
    MotorIdentifier motor_id = {static_cast<huint8>(devIndex), node_id};
    if (function_code == 0x80) {
        huint8 node_id = frame->cob_id & 0x0000007F;
        if (frame->len >= 3) { // 至少需要3个字节
            EmcyMessage msg;
            msg.node_id = node_id;
            msg.error_code = frame->data[0] | (frame->data[1] << 8);
            msg.error_register = frame->data[2];
            for (int i = 0; i < 5 && (i + 3) < frame->len; ++i) {
                msg.manufacturer_specific[i] = frame->data[i + 3];
            }

            // 打印紧急报文信息
            std::cerr << "!!! EMERGENCY [Motor " << (int)msg.node_id << "] !!! "
                      << "Code: 0x" << std::hex << msg.error_code << std::dec
                      << ", Register: 0x" << std::hex << (int)msg.error_register << std::dec << std::endl;

            // 如果有注册的回调函数，则调用它
            //std::lock_guard<std::mutex> lock(instance.mutex_);
            //if (instance.emcy_callback_) {
            //    instance.emcy_callback_(msg);
            //}
        }
        return; // 处理完EMCY后直接返回
    }

    if ((frame->cob_id >= 0x181 && frame->cob_id <= 0x1FF) && frame->len == 8) {
        //std::cout << "INFO [MotorFeedbackManager]: Received CAN frame with COB-ID: " << std::hex << frame->cob_id << std::dec << std::endl;
        huint8 node_id = frame->cob_id & 0x0000007F;
        // Lock the instance's mutex
        std::lock_guard<std::mutex> lock(instance.mutex_);
        
        // Access the instance's gear ratio map
        if (instance.node_gear_ratios_.count(motor_id) == 0) {
            std::cerr << "Warning: Ignoring TPDO1 from unregistered node " << (int)node_id << std::endl;
            return;
        }
        huint32 ppr = instance.node_gear_ratios_[motor_id];

        // Parse data according to the new mapping: Pos (4), Torque (2), Status (2)
        hint32 pos_pulses = (frame->data[3] << 24) | (frame->data[2] << 16) | (frame->data[1] << 8) | frame->data[0];
        hint16 torque_milli = (frame->data[5] << 8) | frame->data[4];
        huint16 status_word = (frame->data[7] << 8) | frame->data[6];

        // Update the feedback data structure
        auto& feedback = instance.feedback_data_[motor_id];
        feedback.position_deg = pulsesToAngle(pos_pulses, ppr);
        feedback.torque_milli = torque_milli;
        feedback.status_word = status_word;
        
        instance.feedback_data_[motor_id].last_update_time = std::chrono::steady_clock::now();
    }

    if (function_code == 0x280 && frame->len == 4) {
        std::lock_guard<std::mutex> lock(instance.mutex_);
        huint8 node_id = frame->cob_id & 0x0000007F;
        if (instance.node_gear_ratios_.count(motor_id) == 0) {
            std::cerr << "Warning: Ignoring TPDO1 from unregistered node " << (int)node_id << std::endl;
            return;
        }
        // 解析数据 (小端格式)
        huint16 status = (frame->data[1] << 8) | frame->data[0];
        huint16 error  = (frame->data[3] << 8) | frame->data[2];

        // 更新到共享数据结构中
        instance.feedback_data_[motor_id].status_word = status;
        instance.feedback_data_[motor_id].error_code = error;
        
        // 检查 Statusword 的 Fault 位 (Bit 3)
        bool fault_detected = (status & 0x0008) != 0;
        
        // 如果错误状态发生变化，则打印日志
        if (fault_detected && !instance.feedback_data_[motor_id].in_fault) {
             std::cerr << "ERROR [Motor " << (int)node_id << "]: Fault detected via TPDO! StatusWord: 0x" 
                       << std::hex << status << ", ErrorCode: 0x" << error << std::dec << std::endl;
        } else if (!fault_detected && instance.feedback_data_[motor_id].in_fault) {
             std::cout << "INFO [Motor " << (int)node_id << "]: Fault cleared via TPDO." << std::endl;
        }

        instance.feedback_data_[motor_id].in_fault = fault_detected;
    }
}

void MotorFeedbackManager::setGearRatio(const MotorIdentifier& motor_id, huint32 pulses_per_rev) {
    // Access member mutex
    std::lock_guard<std::mutex> lock(mutex_); 
    node_gear_ratios_[motor_id] = pulses_per_rev;
}
hreal32 MotorFeedbackManager::pulsesToAngle(hint32 pulses, huint32 pulses_per_rev) {
    if (pulses_per_rev == 0) return 0.0f;
    return (static_cast<hreal32>(pulses) / pulses_per_rev) * 360.0f;
}

hreal32 MotorFeedbackManager::pulsesToVelocity(hint32 pps, huint32 pulses_per_rev) {
    if (pulses_per_rev == 0) return 0.0f;
    return (static_cast<hreal32>(pps) / pulses_per_rev) * 360.0f;
}

MotorFeedbackManager::~MotorFeedbackManager() {
    harmonic_setReceiveDataCallBack(emptyCanRecvCallback);
}

// --- End of EuMotorNode Implementation ---
