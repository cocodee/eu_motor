#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // 用于绑定 std::vector, std::map 等
#include <pybind11/functional.h> // 用于绑定 std::function
#include <pybind11/chrono.h> // 用于绑定 std::chrono

#include "eumotor.h" // 包含你要绑定的头文件

namespace py = pybind11;

// 为了让 pybind11 知道如何处理 eu_harmonic.h 中定义的枚举类型，
// 我们需要在这里声明它们。
// 假设这些枚举在 eu_harmonic.h 中定义。你需要根据实际定义来填充。
// py::enum_ 会自动处理底层的整数类型转换。
void bind_enums(py::module_ &m) {
    py::enum_<harmonic_OperateMode>(m, "OperateMode")
        .value("No_Mode", harmonic_OperateMode_No_Mode)
        .value("PP", harmonic_OperateMode_PP)
        .value("PV", harmonic_OperateMode_PV)
        .value("PT", harmonic_OperateMode_PT)
        .value("HM", harmonic_OperateMode_HM)
        .value("CSP", harmonic_OperateMode_CSP)
        .value("CSV", harmonic_OperateMode_CSV)
        .value("CST", harmonic_OperateMode_CST)
        .value("IP", harmonic_OperateMode_IP)
        .value("Reserve", harmonic_OperateMode_Reserve)
        .export_values();

    // 假设的设备类型和波特率，请根据 eu_harmonic.h 的实际内容修改
    py::enum_<harmonic_DeviceType>(m, "DeviceType")
        .value("USB2CAN", harmonic_DeviceType_USB2CAN)
        .value("Canable", harmonic_DeviceType_Cannable)
        .export_values();

    py::enum_<harmonic_Baudrate>(m, "Baudrate")
        .value("BPS_1M", harmonic_Baudrate_1000)
        .value("BPS_500K", harmonic_Baudrate_500)
        .value("BPS_250K", harmonic_Baudrate_250)
        .export_values();
}


PYBIND11_MODULE(eumotor_py, m) {
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
        .def_readwrite("status_word", &MotorFeedbackData::status_word)
        .def_readwrite("error_code", &MotorFeedbackData::error_code)
        .def_readwrite("in_fault", &MotorFeedbackData::in_fault)
        // std::chrono::time_point 会被自动转换成 Python 的 datetime.datetime 对象
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
        // 将 C 风格的数组绑定为一个只读属性，返回一个 Python 元组
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

    py::class_<CanNetworkManager>(m, "CanNetworkManager", "Singleton class to manage CAN hardware devices.")
        .def(py::init<>())
        .def("init_device", &CanNetworkManager::initDevice, 
             "Initializes a specific CAN device.",
             py::arg("dev_type"), py::arg("dev_index"), py::arg("baudrate"));
        // 注意：我们没有绑定 getInstance，因为这个实现允许直接构造。
        // 如果是严格的单例，需要用特殊方法绑定。

    py::class_<MotorFeedbackManager>(m, "MotorFeedbackManager", "Singleton to handle motor feedback.")
        // 对于单例，我们不暴露构造函数，而是暴露一个静态的 get_instance 方法
        .def_static("get_instance", &MotorFeedbackManager::getInstance, 
                    py::return_value_policy::reference, // 必须用引用策略
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
        
        // --- 生命周期和状态管理 ---
        .def("enable", &EuMotorNode::enable, "Enables the motor and sets it to the specified operating mode.", py::arg("mode"))
        .def("disable", &EuMotorNode::disable, "Disables the motor.")
        .def("clear_fault", &EuMotorNode::clearFault, "Clears any existing faults on the motor.")
        .def("switch_mode", &EuMotorNode::switchMode, "Switches the motor's operating mode.", py::arg("new_mode"))

        // --- 配置 ---
        .def("set_gear_ratio", &EuMotorNode::setGearRatio, "Sets the electronic gear ratio.", py::arg("pulses_per_revolution"))
        .def("set_as_home", &EuMotorNode::setAsHome, "Sets the current physical position as the new logical zero point.")

        // --- 运动指令 ---
        .def("move_to", &EuMotorNode::moveTo, "Moves to a target angle in Profile Position (PP) mode.",
             py::arg("target_angle_deg"), py::arg("velocity_dps"), py::arg("acceleration_dpss"), py::arg("deceleration_dpss"))
        .def("move_at", &EuMotorNode::moveAt, "Rotates at a constant velocity in Profile Velocity (PV) mode.",
             py::arg("target_velocity_dps"), py::arg("acceleration_dpss"), py::arg("deceleration_dpss"))
        .def("apply_torque", &EuMotorNode::applyTorque, "Applies a target torque in Profile Torque (PT) mode.",
             py::arg("target_torque_milli"), py::arg("torque_slope"))
        .def("stop", &EuMotorNode::stop, "Stops any ongoing motion.")

        // --- 数据获取 ---
        .def("get_position", &EuMotorNode::getPosition, "Returns position in degrees.")
        .def("get_velocity", &EuMotorNode::getVelocity, "Returns velocity in degrees per second.")
        .def("get_torque", &EuMotorNode::getTorque, "Returns torque in per-mille of rated torque.")
        .def("get_status_word", &EuMotorNode::getStatusWord, "Returns the raw status word.")
        .def("get_error_code", &EuMotorNode::getErrorCode, "Returns the last error code.")
        .def("get_operation_mode", &EuMotorNode::getOperationMode, "Returns the current operation mode.")
        .def("get_latest_feedback", &EuMotorNode::getLatestFeedback, "Retrieves the most recent feedback data received via TPDO.")

        // --- SDO 读写 (模板方法需要实例化) ---
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

        // --- 实时模式配置 ---
        .def("configure_csp_mode", &EuMotorNode::configureCspMode, "Configures the motor for CSP mode.", 
             py::arg("pdo_index") = 0, py::arg("use_sync") = true)
        .def("configure_cst_mode", &EuMotorNode::configureCstMode, "Configures the motor for CST mode.", 
             py::arg("interpolation_period_ms"), py::arg("pdo_index") = 0, py::arg("use_sync") = true)
        .def("configure_csv_mode", &EuMotorNode::configureCsvMode, "Configures the motor for CSV mode.",
             py::arg("interpolation_period_ms"), py::arg("pdo_index") = 0, py::arg("use_sync") = true)
        .def("configure_ip_mode", &EuMotorNode::configureIpMode, "Configures the motor for IP mode.",
             py::arg("interpolation_period_ms"), py::arg("pdo_index") = 0, py::arg("use_sync") = true)

        // --- 实时指令发送 ---
        .def("send_csp_target_position", &EuMotorNode::sendCspTargetPosition, "Sends target position for CSP mode.",
             py::arg("target_angle_deg"), py::arg("pdo_index") = 0, py::arg("is_sync") = true)
        .def("send_cst_target_torque", &EuMotorNode::sendCstTargetTorque, "Sends target torque for CST mode.",
             py::arg("target_torque"), py::arg("pdo_index") = 0, py::arg("is_sync") = true)
        .def("send_csv_target_velocity", &EuMotorNode::sendCsvTargetVelocity, "Sends target velocity for CSV mode.",
             py::arg("target_velocity_dps"), py::arg("pdo_index") = 0, py::arg("is_sync") = true)
        .def("send_ip_target_position", &EuMotorNode::sendIpTargetPosition, "Sends target position for IP mode.",
             py::arg("target_angle_deg"), py::arg("pdo_index") = 0, py::arg("is_sync") = true)
        .def("send_sync", &EuMotorNode::sendSync, "Broadcasts a SYNC message on the bus.")
        
        // --- 自动反馈配置 ---
        .def("start_auto_feedback", &EuMotorNode::startAutoFeedback, "Configures and enables automatic data feedback via TPDO.",
             py::arg("pdo_index") = 0, py::arg("transmit_type") = 254, py::arg("event_timer_ms") = 100)
        .def("start_error_feedback_tpdo", &EuMotorNode::startErrorFeedbackTPDO, "Configures a TPDO for status and error feedback.",
             py::arg("pdo_index"), py::arg("transmit_type") = 254, py::arg("event_timer_ms") = 10)

        .def("get_node_id", &EuMotorNode::getNodeId, "Gets the node ID of the motor.")

        // --- 增益设置 ---
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