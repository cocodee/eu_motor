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

bool EuMotorNode::moveTo(hreal32 target_angle_deg, huint32 velocity_dps, huint32 acceleration_dpss) {
    if (current_mode_ != harmonic_OperateMode_ProfilePosition && !switchMode(harmonic_OperateMode_ProfilePosition)) return false;
    return check(harmonic_profilePositionControl(
        dev_index_, node_id_,
        angleToPulses(target_angle_deg), velocityToPulses(velocity_dps),
        accelerationToPulses(acceleration_dpss), accelerationToPulses(acceleration_dpss)), "Move To Position");
}

bool EuMotorNode::moveAt(hreal32 target_velocity_dps, huint32 acceleration_dpss) {
    if (current_mode_ != harmonic_OperateMode_ProfileVelocity && !switchMode(harmonic_OperateMode_ProfileVelocity)) return false;
    return check(harmonic_profileVelocityControl(
        dev_index_, node_id_,
        velocityToPulses(target_velocity_dps),
        accelerationToPulses(acceleration_dpss), accelerationToPulses(acceleration_dpss)), "Move At Velocity");
}

bool EuMotorNode::applyTorque(hint16 target_torque_milli, huint32 torque_slope) {
    if (current_mode_ != harmonic_OperateMode_ProfileTorque && !switchMode(harmonic_OperateMode_ProfileTorque)) return false;
    return check(harmonic_profileTorqueControl(
        dev_index_, node_id_,
        target_torque_milli, static_cast<hint16>(torque_slope)), "Apply Torque");
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


// Generic read/write templates - require full implementation for all types or a helper map
template<typename T>
T EuMotorNode::read(huint16 index, huint8 subIndex) {
    // A full implementation requires mapping C++ types to harmonic_DataType
    // This is a simplified example.
    T value;
    harmonic_DataType dt;
    if constexpr (std::is_same_v<T, huint8>) dt = harmonic_DataType_uint8;
    else if constexpr (std::is_same_v<T, huint16>) dt = harmonic_DataType_uint16;
    else if constexpr (std::is_same_v<T, huint32>) dt = harmonic_DataType_uint32;
    else if constexpr (std::is_same_v<T, hint8>) dt = harmonic_DataType_int8;
    else if constexpr (std::is_same_v<T, hint16>) dt = harmonic_DataType_int16;
    else if constexpr (std::is_same_v<T, hint32>) dt = harmonic_DataType_int32;
    else throw std::invalid_argument("Unsupported type for read operation");

    if(!check(harmonic_readDirectory(dev_index_, node_id_, index, subIndex, dt, &value, timeout_ms_), "Generic Read")) {
        throw std::runtime_error("Failed to read SDO " + std::to_string(index));
    }
    return value;
}

template<typename T>
bool EuMotorNode::write(huint16 index, huint8 subIndex, T value) {
    harmonic_DataType dt;
    if constexpr (std::is_same_v<T, huint8>) dt = harmonic_DataType_uint8;
    else if constexpr (std::is_same_v<T, huint16>) dt = harmonic_DataType_uint16;
    else if constexpr (std::is_same_v<T, huint32>) dt = harmonic_DataType_uint32;
    else if constexpr (std::is_same_v<T, hint8>) dt = harmonic_DataType_int8;
    else if constexpr (std::is_same_v<T, hint16>) dt = harmonic_DataType_int16;
    else if constexpr (std::is_same_v<T, hint32>) dt = harmonic_DataType_int32;
    else throw std::invalid_argument("Unsupported type for write operation");

    return check(harmonic_writeDirectory(dev_index_, node_id_, index, subIndex, dt, &value, timeout_ms_), "Generic Write");
}

// Explicit template instantiations to avoid linker errors if definitions are in .cpp
template bool EuMotorNode::write<huint32>(huint16, huint8, huint32);
template huint32 EuMotorNode::read<huint32>(huint16, huint8);
// ... add more instantiations as needed


bool EuMotorNode::configureCspMode(huint16 pdo_index) {
    std::cout << "INFO [Motor " << (int)node_id_ << "]: Configuring for CSP mode..." << std::endl;
    
    // Switch to pre-op for configuration
    if (!check(harmonic_setNodeState(dev_index_, node_id_, harmonic_NMTState_Enter_PreOperational), "CSP: Enter Pre-Op")) return false;

    // 1. Configure RPDO communication type to be synchronous
    // 0x01 means synchronous cyclic
    if (!check(harmonic_setRPDOTransmitType(dev_index_, node_id_, pdo_index, 1), "CSP: Set RPDO Type")) return false;

    // 2. Map the RPDO to the target position object (0x607A)
    // First, disable mapping by setting count to 0
    if (!check(harmonic_setRPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 0), "CSP: Clear RPDO Map")) return false;
    
    // Map Target Position (0x607A), 32 bits (0x20)
    huint32 mapping_value = (0x607A << 16) + 0x0020;
    if (!check(harmonic_setRPDOMapped(dev_index_, node_id_, pdo_index, 0, mapping_value), "CSP: Set RPDO Map")) return false;
    
    // Now, enable mapping by setting count to 1
    if (!check(harmonic_setRPDOMaxMappedCount(dev_index_, node_id_, pdo_index, 1), "CSP: Set RPDO Map Count")) return false;

    // Finally, set the mode
    return switchMode(harmonic_OperateMode_CyclicSyncPosition);
}

void EuMotorNode::sendCspTargetPosition(hreal32 target_angle_deg, huint16 pdo_index) {
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
}

void EuMotorNode::sendSync() {
    harmonic_writeCanData(dev_index_, 0x80, nullptr, 0);
}