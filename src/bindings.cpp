#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#include "../include/eu_motor.h" // 包含你要绑定的头文件

namespace py = pybind11;

// 根据 eu_harmonic.h 完整地绑定所有枚举
void bind_enums(py::module_ &m) {
    // 设备类型
    py::enum_<harmonic_DeviceType>(m, "DeviceType")
        .value("USB2CAN", harmonic_DeviceType_USB2CAN)
        .value("Canable", harmonic_DeviceType_Canable) // 修正了拼写错误
        .export_values();

    // 波特率
    py::enum_<harmonic_Baudrate>(m, "Baudrate")
        .value("BPS_10K", harmonic_Baudrate_10)
        .value("BPS_20K", harmonic_Baudrate_20)
        .value("BPS_50K", harmonic_Baudrate_50)
        .value("BPS_100K", harmonic_Baudrate_100)
        .value("BPS_250K", harmonic_Baudrate_250)
        .value("BPS_500K", harmonic_Baudrate_500)
        .value("BPS_1M", harmonic_Baudrate_1000)
        .export_values();

    // 操作模式 (核心修正)
    py::enum_<harmonic_OperateMode>(m, "OperateMode")
        .value("AutoTuning", harmonic_OperateMode_AutoTuning)
        .value("INLCalibration", harmonic_OperateMode_INLCalibration)
        .value("RotorAligning", harmonic_OperateMode_RotorAligning)
        .value("Reserve", harmonic_OperateMode_Reserve)
        .value("PP", harmonic_OperateMode_ProfilePosition)      // 映射到 ProfilePosition
        .value("Velocity", harmonic_OperateMode_Velocity)
        .value("PV", harmonic_OperateMode_ProfileVelocity)      // 映射到 ProfileVelocity
        .value("PT", harmonic_OperateMode_ProfileTorque)        // 映射到 ProfileTorque
        .value("HM", harmonic_OperateMode_Homing)               // 映射到 Homing
        .value("IP", harmonic_OperateMode_InterpolatedPosition) // 映射到 InterpolatedPosition
        .value("CSP", harmonic_OperateMode_CyclicSyncPosition)  // 映射到 CyclicSyncPosition
        .value("CSV", harmonic_OperateMode_CyclicSyncVelocity)  // 映射到 CyclicSyncVelocity
        .value("CST", harmonic_OperateMode_CyclicSyncTorque)    // 映射到 CyclicSyncTorque
        .value("TorquePositionFixed", harmonic_OperateMode_TorquePositionFixed)
        .export_values();

    // --- 新增的完整枚举绑定 ---

    // NMT状态
    py::enum_<harmonic_NMTState>(m, "NMTState")
        .value("Start_Node", harmonic_NMTState_Start_Node)
        .value("Stop_Node", harmonic_NMTState_Stop_Node)
        .value("Enter_PreOperational", harmonic_NMTState_Enter_PreOperational)
        .value("Reset_Node", harmonic_NMTState_Reset_Node)
        .value("Reset_Comunication", harmonic_NMTState_Reset_Comunication)
        .export_values();

    // 节点状态
    py::enum_<harmonic_NodeState>(m, "NodeState")
        .value("Initialisation", harmonic_NodeState_Initialisation)
        .value("Disconnected", harmonic_NodeState_Disconnected)
        .value("Connecting", harmonic_NodeState_Connecting)
        .value("Preparing", harmonic_NodeState_Preparing)
        .value("Stopped", harmonic_NodeState_Stopped)
        .value("Operational", harmonic_NodeState_Operational)
        .value("Pre_operational", harmonic_NodeState_Pre_operational)
        .value("Unknown_state", harmonic_NodeState_Unknown_state)
        .export_values();

    // 数据类型
    py::enum_<harmonic_DataType>(m, "DataType")
        .value("boolean", harmonic_DataType_boolean)
        .value("int8", harmonic_DataType_int8)
        .value("int16", harmonic_DataType_int16)
        .value("int32", harmonic_DataType_int32)
        .value("uint8", harmonic_DataType_uint8)
        .value("uint16", harmonic_DataType_uint16)
        .value("uint32", harmonic_DataType_uint32)
        .value("real32", harmonic_DataType_real32)
        .export_values();
    
    // 快速停止选项
    py::enum_<harmonic_QuickStopOption>(m, "QuickStopOption")
        .value("Disable_Drive", harmonic_QuickStopOption_Disable_Drive)
        .value("Slow_Down_On_Slow_Down_Ramp", harmonic_QuickStopOption_Slow_Down_On_Slow_Down_Ramp)
        .value("Slow_Down_On_Quick_Stop_Ramp", harmonic_QuickStopOption_Slow_Down_On_Quick_Stop_Ramp)
        .value("Slow_Down_On_the_Current_Limit", harmonic_QuickStopOption_Slow_Down_On_the_Current_Limit)
        .export_values(); // ...可以添加更多，如果需要的话

    // 关机选项
    py::enum_<harmonic_ShutdownOption>(m, "ShutdownOption")
        .value("Disable_Drive_Function", harmonic_ShutdownOption_Disable_Drive_Function)
        .value("Slown_With_Slow_Down_Ramp", harmonic_ShutdownOption_Slown_With_Slow_Down_Ramp)
        .export_values();
    
    // ... 类似地可以添加 harmonic_DisableOperationOption, harmonic_HaltOption, harmonic_FaultReactionOption
}

PYBIND11_MODULE(eu_motor_py, m) {
    m.doc() = "Python bindings for the EuMotor CANopen motor control library";

    // --- 绑定枚举 ---
    bind_enums(m);

    // --- 绑定数据结构 ---

    py::class_<MotorIdentifier>(m, "MotorIdentifier")
        .def(py::init<huint8, huint8>(), py::arg("dev_index"), py::arg("node_id"))
        .def_readwrite("dev_index", &MotorIdentifier::dev_index)
        .def_readwrite("node_id", &MotorIdentifier::node_id)
        .def("__repr__",
            [](const MotorIdentifier &id) {
                return "<MotorIdentifier dev_index=" + std::to_string(id.dev_index) +
                       ", node_id=" + std::to_string(id.node_id) + ">";
            });

    py::class_<MotorFeedbackData>(m, "MotorFeedbackData")
        .def(py::init<>())
        .def_readwrite("position_deg", &MotorFeedbackData::position_deg)
        .def_readwrite("velocity_dps", &MotorFeedbackData::velocity_dps)
        .def_readwrite("torque_milli", &MotorFeedbackData::torque_milli)
        .def_readwrite("status_word", &MotorFeedbackData::status_word)
        .def_readwrite("error_code", &MotorFeedbackData::error_code)
        .def_readwrite("in_fault", &MotorFeedbackData::in_fault)
        .def_readwrite("last_update_time", &MotorFeedbackData::last_update_time)
        .def("__repr__",
            [](const MotorFeedbackData &d) {
                return "<MotorFeedbackData pos=" + std::to_string(d.position_deg) +
                       " deg, vel=" + std::to_string(d.velocity_dps) + " dps>";
            });
    
    py::class_<EmcyMessage>(m, "EmcyMessage")
        .def(py::init<>())
        .def_readwrite("node_id", &EmcyMessage::node_id)
        .def_readwrite("error_code", &EmcyMessage::error_code)
        .def_readwrite("error_register", &EmcyMessage::error_register)
        .def_property_readonly("manufacturer_specific", [](const EmcyMessage &msg) {
            return py::make_tuple(msg.manufacturer_specific[0], msg.manufacturer_specific[1],
                                  msg.manufacturer_specific[2], msg.manufacturer_specific[3],
                                  msg.manufacturer_specific[4]);
        })
        .def("__repr__",
            [](const EmcyMessage &msg) {
                return "<EmcyMessage node=" + std::to_string(msg.node_id) +
                       ", code=0x" + std::to_string(msg.error_code) + ">";
            });

    // --- 绑定管理器类 ---

    py::class_<CanNetworkManager>(m, "CanNetworkManager", "Class to manage CAN hardware devices.")
        .def(py::init<>())
        .def("init_device", &CanNetworkManager::initDevice, 
             "Initializes a specific CAN device.",
             py::arg("dev_type"), py::arg("dev_index"), py::arg("baudrate"));

    py::class_<MotorFeedbackManager, std::unique_ptr<MotorFeedbackManager, py::nodelete>>(m, "MotorFeedbackManager", "Singleton to handle motor feedback.")        .def_static("get_instance", &MotorFeedbackManager::getInstance, 
                    py::return_value_policy::reference,
                    "Get the single instance of the MotorFeedbackManager.")
        .def("register_callback", &MotorFeedbackManager::registerCallback, "Register the global CAN receive callback.")
        .def("get_feedback", &MotorFeedbackManager::getFeedback, 
             "Get the latest feedback data for a motor.",
             py::arg("motor_id"))
        .def("set_gear_ratio", &MotorFeedbackManager::setGearRatio,
             "Set gear ratio for a motor to correctly parse feedback.",
             py::arg("motor_id"), py::arg("pulses_per_rev"));


    // --- 绑定核心类 EuMotorNode ---

    py::class_<EuMotorNode, std::shared_ptr<EuMotorNode>>(m, "EuMotorNode", "Represents a single CANopen motor node.")
        .def(py::init<huint8, huint8, huint32>(), 
             py::arg("dev_index"), py::arg("node_id"), py::arg("default_timeout_ms") = 100)
        
        .def("enable", &EuMotorNode::enable, "Enables the motor and sets it to the specified operating mode.", py::arg("mode"))
        .def("disable", &EuMotorNode::disable, "Disables the motor.")
        .def("clear_fault", &EuMotorNode::clearFault, "Clears any existing faults on the motor.")
        .def("switch_mode", &EuMotorNode::switchMode, "Switches the motor's operating mode.", py::arg("new_mode"))

        .def("set_gear_ratio", &EuMotorNode::setGearRatio, "Sets the electronic gear ratio.", py::arg("pulses_per_revolution"))
        .def("set_as_home", &EuMotorNode::setAsHome, "Sets the current physical position as the new logical zero point.")

        .def("move_to", &EuMotorNode::moveTo, "Moves to a target angle in Profile Position (PP) mode.",
             py::arg("target_angle_deg"), py::arg("velocity_dps"), py::arg("acceleration_dpss"), py::arg("deceleration_dpss"))
        .def("move_at", &EuMotorNode::moveAt, "Rotates at a constant velocity in Profile Velocity (PV) mode.",
             py::arg("target_velocity_dps"), py::arg("acceleration_dpss"), py::arg("deceleration_dpss"))
        .def("apply_torque", &EuMotorNode::applyTorque, "Applies a target torque in Profile Torque (PT) mode.",
             py::arg("target_torque_milli"), py::arg("torque_slope"))
        .def("stop", &EuMotorNode::stop, "Stops any ongoing motion.")

        .def("get_position", &EuMotorNode::getPosition, "Returns position in degrees.")
        .def("get_velocity", &EuMotorNode::getVelocity, "Returns velocity in degrees per second.")
        .def("get_torque", &EuMotorNode::getTorque, "Returns torque in per-mille of rated torque.")
        .def("get_status_word", &EuMotorNode::getStatusWord, "Returns the raw status word.")
        .def("get_error_code", &EuMotorNode::getErrorCode, "Returns the last error code.")
        .def("get_operation_mode", &EuMotorNode::getOperationMode, "Returns the current operation mode.")
        .def("get_latest_feedback", &EuMotorNode::getLatestFeedback, "Retrieves the most recent feedback data received via TPDO.")

        .def("read_u8", &EuMotorNode::read<huint8>)
        .def("read_u16", &EuMotorNode::read<huint16>)
        .def("read_u32", &EuMotorNode::read<huint32>)
        .def("read_s8", &EuMotorNode::read<hint8>)
        .def("read_s16", &EuMotorNode::read<hint16>)
        .def("read_s32", &EuMotorNode::read<hint32>)
        .def("read_float", &EuMotorNode::read<hreal32>)
        .def("write_u8", &EuMotorNode::write<huint8>)
        .def("write_u16", &EuMotorNode::write<huint16>)
        .def("write_u32", &EuMotorNode::write<huint32>)
        .def("write_s8", &EuMotorNode::write<hint8>)
        .def("write_s16", &EuMotorNode::write<hint16>)
        .def("write_s32", &EuMotorNode::write<hint32>)
        .def("write_float", &EuMotorNode::write<hreal32>)

        .def("configure_csp_mode", &EuMotorNode::configureCspMode, "Configures the motor for CSP mode.", 
             py::arg("pdo_index") = 0, py::arg("use_sync") = true)
        .def("configure_cst_mode", &EuMotorNode::configureCstMode, "Configures the motor for CST mode.", 
             py::arg("interpolation_period_ms"), py::arg("pdo_index") = 0, py::arg("use_sync") = true)
        .def("configure_csv_mode", &EuMotorNode::configureCsvMode, "Configures the motor for CSV mode.",
             py::arg("interpolation_period_ms"), py::arg("pdo_index") = 0, py::arg("use_sync") = true)
        .def("configure_ip_mode", &EuMotorNode::configureIpMode, "Configures the motor for IP mode.",
             py::arg("interpolation_period_ms"), py::arg("pdo_index") = 0, py::arg("use_sync") = true)

        .def("send_csp_target_position", &EuMotorNode::sendCspTargetPosition, "Sends target position for CSP mode.",
             py::arg("target_angle_deg"), py::arg("pdo_index") = 0, py::arg("is_sync") = true)
        .def("send_cst_target_torque", &EuMotorNode::sendCstTargetTorque, "Sends target torque for CST mode.",
             py::arg("target_torque"), py::arg("pdo_index") = 0, py::arg("is_sync") = true)
        .def("send_csv_target_velocity", &EuMotorNode::sendCsvTargetVelocity, "Sends target velocity for CSV mode.",
             py::arg("target_velocity_dps"), py::arg("pdo_index") = 0, py::arg("is_sync") = true)
        .def("send_ip_target_position", &EuMotorNode::sendIpTargetPosition, "Sends target position for IP mode.",
             py::arg("target_angle_deg"), py::arg("pdo_index") = 0, py::arg("is_sync") = true)
        .def("send_sync", &EuMotorNode::sendSync, "Broadcasts a SYNC message on the bus.")
        
        .def("start_auto_feedback", &EuMotorNode::startAutoFeedback, "Configures and enables automatic data feedback via TPDO.",
             py::arg("pdo_index") = 0, py::arg("transmit_type") = 254, py::arg("event_timer_ms") = 100)
        .def("start_error_feedback_tpdo", &EuMotorNode::startErrorFeedbackTPDO, "Configures a TPDO for status and error feedback.",
             py::arg("pdo_index"), py::arg("transmit_type") = 254, py::arg("event_timer_ms") = 10)

        .def("get_node_id", &EuMotorNode::getNodeId, "Gets the node ID of the motor.")

        .def("set_position_gains", &EuMotorNode::setPositionGains, py::arg("kp"), py::arg("ki"))
        .def("set_position_kp", &EuMotorNode::setPositionKp, py::arg("kp"))
        .def("set_position_ki", &EuMotorNode::setPositionKi, py::arg("ki"))
        .def("set_velocity_gains", &EuMotorNode::setVelocityGains, py::arg("kp"), py::arg("ki"))
        .def("set_velocity_kp", &EuMotorNode::setVelocityKp, py::arg("kp"))
        .def("set_velocity_ki", &EuMotorNode::setVelocityKi, py::arg("ki"))
        .def("set_current_gains", &EuMotorNode::setCurrentGains, py::arg("kp"), py::arg("ki"))
        .def("set_current_kp", &EuMotorNode::setCurrentKp, py::arg("kp"))
        .def("set_current_ki", &EuMotorNode::setCurrentKi, py::arg("ki"));
}