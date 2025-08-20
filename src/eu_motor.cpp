#include "../include/eu_motor.h"
#include <cstddef>
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
    // Read the gear ratio on construction
    if (!check(harmonic_getGearRatioShaftRevolutions(dev_index_, node_id_, &pulses_per_rev_, timeout_ms_), "Read Initial Gear Ratio")) {
        pulses_per_rev_ = 360000; // Fallback to a sensible default
        MotorFeedbackManager::getInstance().setGearRatio(node_id_,pulses_per_rev_);
        std::cerr << "WARNING [Motor " << (int)node_id_ << "]: Failed to read gear ratio. Using default " 
                  << pulses_per_rev_ << ". Call setGearRatio() for accuracy." << std::endl;
    }else {
        MotorFeedbackManager::getInstance().node_gear_ratios_[node_id_] = pulses_per_rev_;
        std::cout << "INFO [Motor] " << (int)node_id_ << "]: Gear ratio set to " << pulses_per_rev_ << "." << std::endl;
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

// Generic read/write templates - require full implementation for all types or a helper map
template<typename T>
T EuMotorNode::read(huint16 index, huint8 subIndex) {
    // A full implementation requires mapping C++ types to harmonic_DataType
    // This is a simplified example.
    T value;
    harmonic_DataType dt;
    if (std::is_same<T, huint8>::value) dt = harmonic_DataType_uint8;
    else if (std::is_same<T, huint16>::value) dt = harmonic_DataType_uint16;
    else if (std::is_same<T, huint32>::value) dt = harmonic_DataType_uint32;
    else if (std::is_same<T, hint8>::value) dt = harmonic_DataType_int8;
    else if (std::is_same<T, hint16>::value) dt = harmonic_DataType_int16;
    else if (std::is_same<T, hint32>::value) dt = harmonic_DataType_int32;
    else throw std::invalid_argument("Unsupported type for read operation");

    if(!check(harmonic_readDirectory(dev_index_, node_id_, index, subIndex, dt, &value, timeout_ms_), "Generic Read")) {
        throw std::runtime_error("Failed to read SDO " + std::to_string(index));
    }
    return value;
}

template<typename T>
bool EuMotorNode::write(huint16 index, huint8 subIndex, T value) {
    harmonic_DataType dt;
    if (std::is_same<T, huint8>::value) dt = harmonic_DataType_uint8;
    else if (std::is_same<T, huint16>::value) dt = harmonic_DataType_uint16;
    else if (std::is_same<T, huint32>::value) dt = harmonic_DataType_uint32;
    else if (std::is_same<T, hint8>::value) dt = harmonic_DataType_int8;
    else if (std::is_same<T, hint16>::value) dt = harmonic_DataType_int16;
    else if (std::is_same<T, hint32>::value) dt = harmonic_DataType_int32;
    else throw std::invalid_argument("Unsupported type for write operation");

    return check(harmonic_writeDirectory(dev_index_, node_id_, index, subIndex, dt, &value, timeout_ms_), "Generic Write");
}

// Explicit template instantiations to avoid linker errors if definitions are in .cpp
template bool EuMotorNode::write<huint32>(huint16, huint8, huint32);
template huint32 EuMotorNode::read<huint32>(huint16, huint8);
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
    
    hint32 pos_pulses = angleToPulses(target_angle_deg);
    
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
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring automatic feedback transmit type "<< transmit_type << " ..." << std::endl;

    // TPDOs must be configured in Pre-Operational state
    if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Enter_PreOperational), "Feedback: Enter Pre-Op")) return false;

    // 1. Disable the TPDO by setting its COB-ID's valid bit (bit 31) to 1
    huint32 tpd_cobid = (0x180 + (0x100 * pdo_index) + node_id_);
    if (!check(harmonic_setTPDOCobId(dev_index_, node_id_, pdo_index, tpd_cobid | 0x80000000), "Feedback: Disable TPDO")) return false;

    // 2. Clear existing mappings by setting the map count to 0
    if (!check(harmonic_setTPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 0), "Feedback: Clear TPDO Map")) return false;

    // 3. Map the desired objects (Total size must be <= 8 bytes)
    // Mapping format: (Index << 16) | (SubIndex << 8) | (DataLength in bits)
    huint32 pos_mapping = (0x6064 << 16) | (0x00 << 8) | 32; // Actual Position (4 bytes)
    huint32 vel_mapping = (0x606C << 16) | (0x00 << 8) | 32; // Actual Velocity (4 bytes)

    if (!check(harmonic_setTPDOMapped(dev_index_, node_id_, pdo_index, 0, pos_mapping), "Feedback: Map Position")) return false;
    if (!check(harmonic_setTPDOMapped(dev_index_, node_id_, pdo_index, 1, vel_mapping), "Feedback: Map Velocity")) return false;

    // 4. Set the number of mapped objects
    if (!check(harmonic_setTPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 2), "Feedback: Set TPDO Map Count")) return false;

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

MotorFeedbackData EuMotorNode::getLatestFeedback(){
    MotorFeedbackManager& feedback_manager_= MotorFeedbackManager::getInstance();
    return feedback_manager_.getFeedback(node_id_);
}

// --- MotorFeedbackManager Implementation ---

MotorFeedbackManager& MotorFeedbackManager::getInstance() {
    static MotorFeedbackManager instance;
    return instance;
}

void MotorFeedbackManager::registerCallback() {
    harmonic_setReceiveDataCallBack(validCanRecvCallback);
}

MotorFeedbackData MotorFeedbackManager::getFeedback(huint8 nodeId) {
    // Access member mutex
    std::lock_guard<std::mutex> lock(mutex_); 
    if (feedback_data_.count(nodeId)) {
        return feedback_data_[nodeId];
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

    huint32 function_code = frame->cob_id & 0xFF80;
    
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
        if (instance.node_gear_ratios_.count(node_id) == 0) {
            std::cerr << "Warning: Ignoring TPDO1 from unregistered node " << (int)node_id << std::endl;
            return;
        }
        huint32 ppr = instance.node_gear_ratios_[node_id];

        hint32 pos_pulses = (frame->data[3] << 24) | (frame->data[2] << 16) | (frame->data[1] << 8) | frame->data[0];
        hint32 vel_pulses = (frame->data[7] << 24) | (frame->data[6] << 16) | (frame->data[5] << 8) | frame->data[4];

        // Update the instance's feedback data map
        instance.feedback_data_[node_id].position_deg = pulsesToAngle(pos_pulses, ppr);
        instance.feedback_data_[node_id].velocity_dps = pulsesToVelocity(vel_pulses, ppr);
        instance.feedback_data_[node_id].last_update_time = std::chrono::steady_clock::now();
    }

    if (function_code == 0x280 && frame->len == 4) {
        std::lock_guard<std::mutex> lock(instance.mutex_);
        huint8 node_id = frame->cob_id & 0x0000007F;
        if (instance.node_gear_ratios_.count(node_id) == 0) {
            std::cerr << "Warning: Ignoring TPDO1 from unregistered node " << (int)node_id << std::endl;
            return;
        }
        // 解析数据 (小端格式)
        huint16 status = (frame->data[1] << 8) | frame->data[0];
        huint16 error  = (frame->data[3] << 8) | frame->data[2];

        // 更新到共享数据结构中
        instance.feedback_data_[node_id].status_word = status;
        instance.feedback_data_[node_id].error_code = error;
        
        // 检查 Statusword 的 Fault 位 (Bit 3)
        bool fault_detected = (status & 0x0008) != 0;
        
        // 如果错误状态发生变化，则打印日志
        if (fault_detected && !instance.feedback_data_[node_id].in_fault) {
             std::cerr << "ERROR [Motor " << (int)node_id << "]: Fault detected via TPDO! StatusWord: 0x" 
                       << std::hex << status << ", ErrorCode: 0x" << error << std::dec << std::endl;
        } else if (!fault_detected && instance.feedback_data_[node_id].in_fault) {
             std::cout << "INFO [Motor " << (int)node_id << "]: Fault cleared via TPDO." << std::endl;
        }

        instance.feedback_data_[node_id].in_fault = fault_detected;
    }
}

void MotorFeedbackManager::setGearRatio(huint8 nodeId, huint32 pulses_per_rev) {
    // Access member mutex
    std::lock_guard<std::mutex> lock(mutex_); 
    node_gear_ratios_[nodeId] = pulses_per_rev;
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