#include "../include/eu_motor.h"

// --- CanNetworkManager Implementation ---

CanNetworkManager& CanNetworkManager::getInstance() {
    static CanNetworkManager instance;
    return instance;
}

void CanNetworkManager::initDevice(harmonic_DeviceType devType, huint8 devIndex, harmonic_Baudrate baudrate) {
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
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto const& [dev_idx, is_init] : initialized_devices_) {
        if (is_init) {
            std::cout << "CanNetworkManager: Freeing CAN device " << (int)dev_idx << "..." << std::endl;
            harmonic_freeDLL(dev_idx);
        }
    }
    initialized_devices_.clear();
}

// Static member definition
std::map<huint8, bool> CanNetworkManager::initialized_devices_;
std::mutex CanNetworkManager::mutex_;


// --- EuMotorNode Implementation ---
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
        std::cerr << "WARNING [Motor " << (int)node_id_ << "]: Failed to read gear ratio. Using default " 
                  << pulses_per_rev_ << ". Call setGearRatio() for accuracy." << std::endl;
    }
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
    huint16 status = getStatusWord();
    if (status & 0x0008) { // Bit 3 is the fault bit
        std::cout << "INFO [Motor " << (int)node_id_ << "]: Fault detected, attempting to reset..." << std::endl;
        return check(harmonic_setControlword(dev_index_, node_id_, 0x80, timeout_ms_), "Fault Reset");
    }
    return true;
}

bool EuMotorNode::switchMode(harmonic_OperateMode new_mode) {
    if (current_mode_ == new_mode) return true;
    
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Switching mode to " << new_mode << "..." << std::endl;
    if (!disable()) return false; // Go to a safe, non-operational state
    
    if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Enter_PreOperational), "Enter Pre-Op State")) return false;
    if (!check(harmonic_setOperateMode(dev_index_, node_id_, new_mode, timeout_ms_), "Set Operate Mode")) return false;
    if (!enableStateMachine()) return false;
    if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Start_Node), "Enter Operational State")) return false;
    
    current_mode_ = new_mode;
    return true;
}

bool EuMotorNode::resetAndStartNode(){
    if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Reset_Node),"Reset Node")) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Start_Node),"Start Node")) return false;
}
bool EuMotorNode::enableStateMachine() {
    if (!check(harmonic_setControlword(dev_index_, node_id_, 0x06, timeout_ms_), "State Machine: Shutdown")) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!check(harmonic_setControlword(dev_index_, node_id_, 0x07, timeout_ms_), "State Machine: Switch On")) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!check(harmonic_setControlword(dev_index_, node_id_, 0x0F, timeout_ms_), "State Machine: Enable Operation")) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return true;
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

hreal32 EuMotorNode::getPosition() {
    hint32 pulses;
    if (!check(harmonic_getActualPos(dev_index_, node_id_, &pulses, timeout_ms_), "Get Position")) {
        throw std::runtime_error("Failed to read position for Node " + std::to_string(node_id_));
    }
    return pulsesToAngle(pulses);
}

hreal32 EuMotorNode::getVelocity() {
    hint32 pps;
    if (!check(harmonic_getActualVelocity(dev_index_, node_id_, &pps, timeout_ms_), "Get Velocity")) {
        throw std::runtime_error("Failed to read velocity for Node " + std::to_string(node_id_));
    }
    return pulsesToVelocity(pps);
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
    if (!check(harmonic_getServoErrorCode(dev_index_, node_id_, &err, timeout_ms_), "Get Error Code")) {
        throw std::runtime_error("Failed to read Error Code for Node " + std::to_string(node_id_));
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


// --- MotorFeedbackManager Implementation ---

MotorFeedbackManager& MotorFeedbackManager::getInstance() {
    static MotorFeedbackManager instance;
    return instance;
}

void MotorFeedbackManager::registerCallback(huint8 devIndex) {
    harmonic_registerCanRecvCallback(devIndex, canRecvCallback);
}

MotorFeedbackData MotorFeedbackManager::getFeedback(huint8 nodeId) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (feedback_data_.count(nodeId)) {
        return feedback_data_[nodeId];
    }
    return MotorFeedbackData{}; // Return default-initialized struct if no data exists
}

void MotorFeedbackManager::canRecvCallback(huint8 devIndex, harmonic_CanRecvFrame* frame) {
    // This is a static callback, so we access static members.
    // We are interested in TPDOs, which have COB-IDs from 0x181 to 0x480 + node_id
    // TPDO1: 0x180, TPDO2: 0x280, TPDO3: 0x380, TPDO4: 0x480
    huint32 cob_id_base = frame->cob_id & 0xFFFFFF80;
    huint8 node_id = frame->cob_id & 0x0000007F;

    if ((cob_id_base >= 0x180 && cob_id_base <= 0x480) && frame->len == 8) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Check if we have gear ratio info for this node
        if (node_gear_ratios_.count(node_id) == 0) {
            return; // Cannot convert without gear ratio
        }
        huint32 ppr = node_gear_ratios_[node_id];

        // The data is mapped as:
        // bytes 0-3: Actual Position (hint32)
        // bytes 4-7: Actual Velocity (hint32)
        hint32 pos_pulses = (frame->data[3] << 24) | (frame->data[2] << 16) | (frame->data[1] << 8) | frame->data[0];
        hint32 vel_pulses = (frame->data[7] << 24) | (frame->data[6] << 16) | (frame->data[5] << 8) | frame->data[4];

        feedback_data_[node_id].position_deg = pulsesToAngle(pos_pulses, ppr);
        feedback_data_[node_id].velocity_dps = pulsesToVelocity(vel_pulses, ppr);
        feedback_data_[node_id].last_update_time = std::chrono::steady_clock::now();
    }
}

hreal32 MotorFeedbackManager::pulsesToAngle(hint32 pulses, huint32 pulses_per_rev) {
    if (pulses_per_rev == 0) return 0.0f;
    return (static_cast<hreal32>(pulses) / pulses_per_rev) * 360.0f;
}

hreal32 MotorFeedbackManager::pulsesToVelocity(hint32 pps, huint32 pulses_per_rev) {
    if (pulses_per_rev == 0) return 0.0f;
    return (static_cast<hreal32>(pps) / pulses_per_rev) * 360.0f;
}

// Static member definitions for MotorFeedbackManager
std::map<huint8, MotorFeedbackData> MotorFeedbackManager::feedback_data_;
std::map<huint8, huint32> MotorFeedbackManager::node_gear_ratios_;
std::mutex MotorFeedbackManager::mutex_;



bool EuMotorNode::configureCspMode(huint16 pdo_index) {
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring for CSP mode..." << std::endl;
    int itpv = 4;        // 插补周期，单位ms
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
    if (!check(harmonic_setRPDOTransmitType(dev_index_, node_id_, pdo_index, 1), "CSP: Set RPDO Type")) return false;

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

void EuMotorNode::sendCspTargetPosition(hreal32 target_angle_deg, huint16 pdo_index, bool isSync) {
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
    
    harmonic_writeCanData(dev_index_, rpdo_base_cobid + node_id_, data, 4);
    if (isSync){
        sendSync();
    }
}

/**
 * @brief Configures the motor for Cyclic Sync Torque (CST) mode.
 * This function sets up the necessary PDOs for real-time torque control.
 * It follows the logic from the test_cst_mode.cpp example.
 */
bool EuMotorNode::configureCstMode(huint8 interpolation_period_ms, huint16 pdo_index) {
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring for CST mode..." << std::endl;

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
    if (!check(harmonic_setRPDOTransmitType(dev_index_, node_id_, pdo_index, 1), "CST: Set RPDO Type to Sync")) return false;

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
bool EuMotorNode::configureCsvMode(huint8 interpolation_period_ms, huint16 pdo_index) {
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring for CSV mode..." << std::endl;

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
    if (!check(harmonic_setRPDOTransmitType(dev_index_, node_id_, pdo_index, 1), "CSV: Set RPDO Type to Sync")) return false;


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
    if (!resetAndStartNode()) return false;
    
    // 8. Re-enable the RPDO with the correct, active COB-ID
    if (!check(harmonic_setRPDOCobId(dev_index_, node_id_, pdo_index, 0x200 + node_id_), "IP: Enable RPDO")) return false;

    // 9. Go through the standard state machine
    if (!enableStateMachine()) return false;
    
    // 10. IP mode requires an extra control word (0x1F) to start the interpolator
    if (!check(harmonic_setControlword(dev_index_, node_id_, 0x1F, timeout_ms_), "IP: Start Interpolator")) return false;

    current_mode_ = harmonic_OperateMode_InterpolatedPosition;
    return true;
}


/**
 * @brief Sends the target position for IP mode using a direct CAN write.
 */
void EuMotorNode::sendIpTargetPosition(hreal32 target_angle_deg, huint16 pdo_index,bool isSync) {
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
    harmonic_writeCanData(dev_index_, cob_id, data, 4);

    // If using synchronous mode, a separate SYNC message is required.
    // The example code's setPos function sends it, so we replicate that behavior.
    if (isSync) {
        sendSync();
    }
}
void EuMotorNode::sendSync() {
    huint8 data[1] = {0};
    harmonic_writeCanData(dev_index_, 0x80, data, 1);
}

bool EuMotorNode::startAutoFeedback(huint16 pdo_index, huint8 transmit_type, huint16 event_timer_ms) {
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring automatic feedback (TPDO" << pdo_index + 1 << ")..." << std::endl;

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


// --- End of EuMotorNode Implementation ---