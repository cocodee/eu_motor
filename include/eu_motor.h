#ifndef EUMOTOR_H
#define EUMOTOR_H

#include "eu_harmonic.h" // 包含官方C-API头文件
#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <map>
#include <condition_variable>

// --- Data Structures for Feedback ---

/**
 * @struct MotorFeedbackData
 * @brief Holds the latest feedback data received from a motor via TPDO.
 */
struct MotorFeedbackData {
    hreal32 position_deg = 0.0f;
    hreal32 velocity_dps = 0.0f;
    std::chrono::steady_clock::time_point last_update_time;
};

// Forward declaration
class EuMotorNode; 

// --- Managers ---

/**
 * @class MotorFeedbackManager
 * @brief Singleton to handle incoming CAN frames and parse motor feedback data.
 */
// in your MotorFeedbackManager class definition
class MotorFeedbackManager {
public:
    static MotorFeedbackManager& getInstance();
    void registerCallback();
    MotorFeedbackData getFeedback(huint8 nodeId);

    // Make this public so EuMotorNode can access it
    std::map<huint8, huint32> node_gear_ratios_;

    // 禁用拷贝和赋值
    MotorFeedbackManager(const MotorFeedbackManager&) = delete;
    MotorFeedbackManager& operator=(const MotorFeedbackManager&) = delete;

    void setGearRatio(huint8 nodeId, huint32 pulses_per_rev);

    static void canRecvCallback(int devIndex, const harmonic_CanMsg* frame);

private:
    MotorFeedbackManager() = default;
    ~MotorFeedbackManager();

    // The static callback function remains static

    // Helper functions can remain static as they don't depend on member state
    static hreal32 pulsesToAngle(hint32 pulses, huint32 pulses_per_rev);
    static hreal32 pulsesToVelocity(hint32 pps, huint32 pulses_per_rev);

    // Change static members to regular members
    std::map<huint8, MotorFeedbackData> feedback_data_;
    std::mutex mutex_;
};

/**
 * @class CanNetworkManager
 * @brief Singleton class to manage the lifecycle of CAN hardware devices.
 * This class ensures that each CAN device is initialized only once and
 * is properly freed upon program termination.
 */
class CanNetworkManager {
public:
    // Delete copy and move constructors/assignments
    CanNetworkManager(const CanNetworkManager&) = delete;
    CanNetworkManager& operator=(const CanNetworkManager&) = delete;

    /**
     * @brief Get the single instance of the CanNetworkManager.
     * @return Reference to the CanNetworkManager instance.
     */
    static CanNetworkManager& getInstance();

    /**
     * @brief Initializes a specific CAN device. Throws std::runtime_error on failure.
     * @param devType The type of the CAN device (e.g., USB2CAN).
     * @param devIndex The index of the CAN device (e.g., 0 for the first one).
     * @param baudrate The communication baud rate.
     */
    void initDevice(harmonic_DeviceType devType, huint8 devIndex, harmonic_Baudrate baudrate);

private:
    CanNetworkManager() = default;
    ~CanNetworkManager(); // Destructor will free all initialized devices.

    std::map<huint8, bool> initialized_devices_;
    std::mutex mutex_;
};


/**
 * @class EuMotorNode
 * @brief Represents a single CANopen motor node (e.g., an EUPH harmonic drive).
 * This class provides a high-level, object-oriented interface to control the motor.
 */
class EuMotorNode {
public:
    /**
     * @brief Constructor for a motor node.
     * @param devIndex The hardware index of the CAN bus this motor is on.
     * @param nodeId The CANopen Node ID of this motor (1-127).
     * @param default_timeout_ms Default timeout for SDO communications.
     */
    EuMotorNode(huint8 devIndex, huint8 nodeId, huint32 default_timeout_ms = 100);

    // --- Lifecycle and State Management ---
    
    /**
     * @brief Enables the motor and sets it to the specified operating mode.
     * This performs fault clearing and the full CiA 402 state machine transition.
     * @param mode The desired operating mode to start in.
     * @return True on success, false on failure.
     */
    bool enable(harmonic_OperateMode mode);

    /**
     * @brief Disables the motor, stopping power output.
     * @return True on success, false on failure.
     */
    bool disable();

    /**
     * @brief Clears any existing faults on the motor.
     * @return True on success, false on failure.
     */
    bool clearFault();

    /**
     * @brief Switches the motor's operating mode safely.
     * @param new_mode The new mode to switch to.
     * @return True on success, false on failure.
     */
    bool switchMode(harmonic_OperateMode new_mode);

    // --- Configuration ---

    /**
     * @brief Sets the electronic gear ratio for unit conversion.
     * @param pulses_per_revolution Number of pulses for one 360-degree revolution.
     * @return True on success, false on failure.
     */
    bool setGearRatio(huint32 pulses_per_revolution);

    /**
     * @brief Sets the current physical position as the new logical zero point.
     * @return True on success, false on failure.
     */
    bool setAsHome();

    // --- Motion Commands ---

    /**
     * @brief Moves to a target angle in Profile Position (PP) mode.
     * @param target_angle_deg Target angle in degrees.
     * @param velocity_dps Profile velocity in degrees per second.
     * @param acceleration_dpss Profile acceleration in degrees per second squared.
     * @return True on success, false on failure.
     */
    bool moveTo(hreal32 target_angle_deg, huint32 velocity_dps, huint32 acceleration_dpss,huint32 deceleration_dpss);

    /**
     * @brief Rotates at a constant velocity in Profile Velocity (PV) mode.
     * @param target_velocity_dps Target velocity in degrees per second.
     * @param acceleration_dpss Profile acceleration in degrees per second squared.
     * @return True on success, false on failure.
     */
    bool moveAt(hreal32 target_velocity_dps, huint32 acceleration_dpss,huint32 deceleration_dpss);

    /**
     * @brief Applies a target torque in Profile Torque (PT) mode.
     * @param target_torque_milli Target torque in thousandths of rated torque (1-1000).
     * @param torque_slope Rate of torque change (in thousandths of rated torque per second).
     * @return True on success, false on failure.
     */
    bool applyTorque(hint16 target_torque_milli, huint32 torque_slope);

    /**
     * @brief Stops any ongoing motion using the configured halt option.
     * @return True on success, false on failure.
     */
    bool stop();

    // --- Data Acquisition ---
    
    hreal32 getPosition();      // Returns position in degrees.
    hreal32 getVelocity();      // Returns velocity in degrees per second.
    hint16  getTorque();        // Returns torque in per-mille of rated torque.
    huint16 getStatusWord();    // Returns the raw status word.
    huint16 getErrorCode();     // Returns the last error code.
    harmonic_OperateMode getOperationMode(); // Returns the current operation mode.

    /**
     * @brief Retrieves the most recent feedback data received via TPDO.
     * This is a non-blocking call that returns cached data.
     * @return A struct containing the latest position, velocity, and timestamp.
     */
    MotorFeedbackData getLatestFeedback();

    // --- Low-level SDO Access ---

    template<typename T>
    T read(huint16 index, huint8 subIndex);

    template<typename T>
    bool write(huint16 index, huint8 subIndex, T value);

    /**
     * @brief Configures the motor for CSP mode, setting up the necessary PDOs.
     * @param pdo_index Which RPDO to use for position commands (0-3 for RPDO1-4).
     * @return True on success, false on failure.
     */
    bool configureCspMode(huint16 pdo_index = 0);

    bool configureCstMode(huint8 interpolation_period_ms, huint16 pdo_index = 0);  

    /**
     * @brief Configures the motor for Cyclic Sync Velocity (CSV) mode.
     * @param interpolation_period_ms The time period in milliseconds for interpolation.
     * @param pdo_index The RPDO index to use for receiving target velocity (default 0 for RPDO1).
     * @return True if configuration is successful, false otherwise.
     */
    bool configureCsvMode(huint8 interpolation_period_ms, huint16 pdo_index = 0);

    // --- Real-time Commands ---
    void sendCspTargetPosition(hreal32 target_angle_deg, huint16 pdo_index = 0, bool isSync=true);
    /**
     * @brief Sends the target torque value via a raw CAN frame for CST mode.
     * @param target_torque The target torque in device-specific units (usually 1/1000 of rated torque).
     * @param pdo_index The RPDO index that was configured (default 0 for RPDO1).
     */
    void sendCstTargetTorque(hint16 target_torque, huint16 pdo_index = 0, bool isSync=true);

     /**
     * @brief Sends the target velocity value via a raw CAN frame for CSV mode.
     * @param target_velocity_dps The target velocity in degrees per second.
     * @param pdo_index The RPDO index that was configured (default 0 for RPDO1).
     */
    void sendCsvTargetVelocity(hreal32 target_velocity_dps, huint16 pdo_index = 0, bool isSync=true);
   
    /**
     * @brief Configures the motor for Interpolated Position (IP) mode.
     * @param interpolation_period_ms The time period in milliseconds for interpolation.
     * @param use_sync Whether to use SYNC message to trigger the movement (true) or let it trigger asynchronously on PDO arrival (false).
     * @param pdo_index The RPDO index to use for receiving target position (default 0 for RPDO1).
     * @return True if configuration is successful, false otherwise.
     */
    bool configureIpMode(huint8 interpolation_period_ms, huint16 pdo_index = 0, bool use_sync=true);

    /**
     * @brief Sends the target position value via a raw CAN frame for IP mode.
     * @param target_angle_deg The target position in degrees.
     * @param use_sync If this is true, a SYNC message will NOT be sent automatically. You must call sendSync() manually.
     *                 If false, the PDO itself will trigger the motion.
     * @param pdo_index The RPDO index that was configured (default 0 for RPDO1).
     */
    void sendIpTargetPosition(hreal32 target_angle_deg, huint16 pdo_index = 0, bool isSync=true);   
    /**
     * @brief Broadcasts a SYNC message on the bus to trigger all synced motors.
     */
    void sendSync();

    /**
     * @brief Configures and enables automatic data feedback via TPDO.
     * The motor will start sending its position and velocity data (8 bytes total).
     * @param pdo_index The TPDO to use (0-3 for TPDO1-4).
     * @param transmit_type 254 for event-driven (on change), 255 for event-driven (asynchronous), 1-240 for synchronous (on SYNC).
     * @param event_timer_ms If transmit_type is 254/255, this sets the minimum time between transmissions in ms.
     * @return True on success, false on failure.
     */
    bool startAutoFeedback(huint16 pdo_index = 0, huint8 transmit_type = 254, huint16 event_timer_ms = 100);
    
private:
    huint8 dev_index_;
    huint8 node_id_;
    huint32 timeout_ms_;
    huint32 pulses_per_rev_;
    harmonic_OperateMode current_mode_ = harmonic_OperateMode_Reserve;
    
    // Internal helper for checking API return codes
    bool check(int return_code, const std::string& operation_name) const;

    // Unit conversion helpers
    hint32 angleToPulses(hreal32 angle_deg) const;
    hreal32 pulsesToAngle(hint32 pulses) const;
    hint32 velocityToPulses(hreal32 dps) const;
    hreal32 pulsesToVelocity(hint32 pps) const;
    huint32 accelerationToPulses(huint32 dpss) const;
    huint32 decelerationToPulses(huint32 dpss) const;
    // Helper for CiA 402 state machine transitions
    int enableStateMachine();
    int resetAndStartNode();
};

void validCanRecvCallback(int devIndex, const harmonic_CanMsg* frame);

#endif // EUMOTOR_H