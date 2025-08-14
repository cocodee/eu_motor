// 文件名: test_eu_motor_suite.cpp
#include "../include/eu_motor.h" // 假设头文件路径
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <functional>
#include <thread>
#include <chrono>
#include <cmath> // For std::sin
#include <iomanip>

// Helper function for printing test case headers
void print_header(const std::string& test_name) {
    std::cout << "\n==================================================" << std::endl;
    std::cout << "  RUNNING TEST: " << test_name << std::endl;
    std::cout << "==================================================" << std::endl;
}

// --- Test Case Functions ---
// We will wrap each test case in its own function.

void test_pp_mode(EuMotorNode& motor) {
    print_header("Profile Position (PP) Mode - Single Move");
    if (motor.enable(harmonic_OperateMode_ProfilePosition)) {
        std::cout << "Motor enabled in PP mode. Moving to 90 degrees." << std::endl;
        motor.moveTo(90.0f, 180, 500, 500);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "Current Position: " << motor.getPosition() << " degrees" << std::endl;

        std::cout << "Moving back to 0 degrees." << std::endl;
        motor.moveTo(0.0f, 180, 500, 500);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "Current Position: " << motor.getPosition() << " degrees" << std::endl;
    } else {
        std::cerr << "Failed to enable PP mode." << std::endl;
    }
}

void test_pv_mode(EuMotorNode& motor) {
    print_header("Profile Velocity (PV) Mode");
    if (motor.enable(harmonic_OperateMode_ProfileVelocity)) {
        std::cout << "Motor enabled in PV mode. Rotating at 90 dps for 3 seconds." << std::endl;
        motor.moveAt(90.0f, 500, 500);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "Stopping..." << std::endl;
        motor.stop();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Final Position: " << motor.getPosition() << " degrees" << std::endl;
    } else {
        std::cerr << "Failed to enable PV mode." << std::endl;
    }
}

void test_csp_mode(EuMotorNode& motor) {
    print_header("Cyclic Sync Position (CSP) Mode");
    if (motor.configureCspMode()) {
        std::cout << "Motor configured for CSP mode. Sending a sine wave trajectory." << std::endl;
        hreal32 start_pos = motor.getPosition();
        for (int i = 0; i <= 200; ++i) {
            hreal32 target_pos = start_pos + 25.0f * std::sin(2.0 * M_PI * i / 200.0);
            motor.sendCspTargetPosition(target_pos, 0, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } else {
        std::cerr << "Failed to configure CSP mode." << std::endl;
    }
}

void test_cst_mode(EuMotorNode& motor) {
    print_header("Cyclic Sync Torque (CST) Mode");
    if (motor.configureCstMode(4)) {
        std::cout << "Motor configured for CST mode. Applying small torque." << std::endl;
        std::cout << "WARNING: Motor may move freely. Be careful!" << std::endl;
        hint16 target_torque = 50;
        for(int i = 0; i < 200; ++i) {
             motor.sendCstTargetTorque(target_torque, 0, true);
             std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    } else {
         std::cerr << "Failed to configure CST mode." << std::endl;
    }
}

void test_ip_mode(EuMotorNode& motor) {
    print_header("Interpolated Position (IP) Mode");
    bool use_sync_for_ip = true;
    if (motor.configureIpMode(4, 0, use_sync_for_ip)) {
        std::cout << "Motor configured for IP mode. Sending position stream." << std::endl;
        hreal32 start_pos = motor.getPosition();
        for (int i = 0; i <= 100; ++i) {
            hreal32 target_pos = start_pos + (90.0f * i / 100.0f);
            motor.sendIpTargetPosition(target_pos, 0, use_sync_for_ip);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    } else {
        std::cerr << "Failed to configure IP mode." << std::endl;
    }
}

void test_feedback_mode(EuMotorNode& motor) {
    print_header("Automatic Feedback (TPDO)");
    MotorFeedbackManager& feedback_manager_= MotorFeedbackManager::getInstance();
    feedback_manager_.registerCallback();
    // Use a shorter event timer for more frequent updates
    if (motor.startAutoFeedback(0, 255, 20)) {
        std::cout << "Automatic feedback started. Moving motor to 180 degrees..." << std::endl;
        //motor.enable(harmonic_OperateMode_ProfilePosition);
        //motor.moveTo(90.0f, 90, 500, 500);

        // Poll for feedback for a few seconds while it moves
        for (int i = 0; i < 50; ++i) {
            MotorFeedbackData data = motor.getLatestFeedback();
            auto time_since_update = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - data.last_update_time
            ).count();

            std::cout << "Feedback: Pos=" << data.position_deg 
                      << " deg, Vel=" << data.velocity_dps 
                      << " dps (updated " << time_since_update << " ms ago)\r" << std::flush ;
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cout << std::endl; // Newline after the carriage return loop
        
        // Wait for move to complete
        std::this_thread::sleep_for(std::chrono::seconds(2));
        MotorFeedbackData final_data = motor.getLatestFeedback();
        std::cout << "Final position from feedback: " << final_data.position_deg << std::endl;
        std::cout << "Final position from SDO read: " << motor.getPosition() << std::endl;
    } else {
        std::cerr << "Failed to start feedback mode." << std::endl;
    }
}


// +++ NEW TEST CASE +++
// Helper function to decode and print the CiA 402 Statusword
void print_status_word(huint16 status) {
    std::cout << "Statusword: 0x" << std::hex << std::setw(4) << std::setfill('0') << status << std::dec 
              << " (";
    std::string flags;
    if (status & 0x0001) flags += "ReadyToSwitchOn, ";
    if (status & 0x0002) flags += "SwitchedOn, ";
    if (status & 0x0004) flags += "OpEnabled, ";
    if (status & 0x0008) flags += "FAULT, ";
    if (status & 0x0010) flags += "VoltageEnabled, ";
    if (status & 0x0020) flags += "QuickStop, ";
    if (status & 0x0040) flags += "SwitchOnDisabled, ";
    if (status & 0x0080) flags += "Warning, ";
    if (status & 0x0400) flags += "TargetReached, ";
    if (status & 0x1000) flags += "Remote, ";
    if (!flags.empty()) {
        flags.resize(flags.length() - 2); // Remove trailing comma and space
    }
    std::cout << flags << ")" << std::endl;
}

void test_status_and_errors(EuMotorNode& motor) {
    print_header("Status, Error & SDO Reading Test");

    try {
        harmonic_OperateMode mode = motor.getOperationMode();
        std::cout << "Current Operation Mode (from device): " << static_cast<int>(mode) << std::endl;

        huint16 status = motor.getStatusWord();
        print_status_word(status);

        huint16 error_code = motor.getErrorCode();
        std::cout << "Current Error Code: 0x" << std::hex << std::setw(4) << std::setfill('0') << error_code << std::dec << std::endl;

        // --- 2. Reading Error History from SDO ---
        std::cout << "\n--- 2. Reading Error History (SDO 0x1003) ---" << std::endl;
        // Try to read the first stored error (sub-index 1). This is a 32-bit value.
        // This demonstrates reading from the object dictionary.
        try {
            huint32 first_error = motor.read<huint32>(0x1003, 1);
            std::cout << "Latest error from history (0x1003, sub 1): 0x" << std::hex << first_error << std::dec << std::endl;
        } catch (const std::runtime_error& e) {
            std::cout << "Could not read error history SDO: " << e.what() << std::endl;
            std::cout << "(This is normal if no errors have ever occurred or SDO access is restricted)." << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "An error occurred during the status test for motor " 
                  << motor.getNodeId() << ": " << e.what() << std::endl;
    }

    motor.disable();
    std::cout << "\nStatus test for motor " << motor.getNodeId() << " finished." << std::endl;
}
// +++ END OF NEW TEST CASE +++


// Function to display help message
void print_usage(const char* prog_name, const std::map<std::string, std::function<void(EuMotorNode&)>>& tests) {
    std::cout << "Usage: " << prog_name << " --dev <id1> <id2>... --tests <test1> <test2>..." << std::endl;
    std::cout << "   or: " << prog_name << " --dev <id1>... --tests all" << std::endl;
    std::cout << "\nAvailable tests:" << std::endl;
    for (const auto& pair : tests) {
        std::cout << "  - " << pair.first << std::endl;
    }
    std::cout << "\nExample: " << prog_name << " --dev 1 --tests pp status" << std::endl;
}

void run_test_on_all_motors(const std::string& test_name, 
                           const std::function<void(EuMotorNode&)>& test_func,
                            std::vector<std::unique_ptr<EuMotorNode>>& motors) {
    print_header(test_name);
    
    std::vector<std::thread> test_threads;

    // Reset and prepare all motors before starting the test
    for (auto& motor : motors) {
        //motor->clearFault();
        motor->disable();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // Launch a thread for each motor to run the test function
    for (auto& motor : motors) {
        test_threads.emplace_back(test_func, std::ref(*motor));
    }

    // Special handling for synchronous modes (CSP, CST, etc.)
    // A master SYNC signal can be sent here if needed.
    if (test_name == "csp" || test_name == "cst") {
        std::cout << "Running synchronous test. Master SYNC would be sent from here." << std::endl;
        // Example: Send SYNC every 4ms for the duration of the test
        for(int i=0; i<500; ++i) { // 2 seconds duration
            if(!motors.empty()) motors[0]->sendSync();
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }

    // Wait for all threads to complete
    for (auto& t : test_threads) {
        if (t.joinable()) {
            t.join();
        }
    }
    
    std::cout << "\n--- " << test_name << " test completed for all motors. ---" << std::endl;
}
int main(int argc, char* argv[]) {
    // --- Test Suite Definition ---
    std::map<std::string, std::function<void(EuMotorNode&)>> test_suite;
    test_suite["pp"] = test_pp_mode;
    test_suite["pv"] = test_pv_mode;
    test_suite["csp"] = test_csp_mode;
    test_suite["cst"] = test_cst_mode;
    test_suite["ip"] = test_ip_mode;
    test_suite["feedback"] = test_feedback_mode;
    test_suite["status"] = test_status_and_errors;

    // --- Argument Parsing ---
    // --- Argument Parsing ---
    std::vector<huint8> node_ids;
    std::vector<std::string> tests_to_run;
    
    if (argc < 5) {
        print_usage(argv[0],test_suite);
        return 1;
    }

    int i = 1;
    while (i < argc) {
        std::string arg = argv[i];
        if (arg == "--dev") {
            i++;
            while (i < argc && argv[i][0] != '-') {
                node_ids.push_back(static_cast<huint8>(std::stoi(argv[i])));
                i++;
            }
        } else if (arg == "--tests") {
            i++;
            if (i < argc && std::string(argv[i]) == "all") {
                for (const auto& pair : test_suite) {
                    tests_to_run.push_back(pair.first);
                }
                i++;
            } else {
                while (i < argc && argv[i][0] != '-') {
                    if (test_suite.count(argv[i])) {
                        tests_to_run.push_back(argv[i]);
                    } else {
                        std::cerr << "Warning: Test '" << argv[i] << "' not found. Skipping." << std::endl;
                    }
                    i++;
                }
            }
        } else {
            i++;
        }
    }

    if (node_ids.empty() || tests_to_run.empty()) {
        std::cerr << "Error: You must specify at least one device ID and one test." << std::endl;
        print_usage(argv[0],test_suite);
        return 1;
    }

    try {
        // --- Setup ---
        huint8 devIndex = 1;
        CanNetworkManager canNetworkManager;
        canNetworkManager.initDevice(harmonic_DeviceType_Canable, devIndex, harmonic_Baudrate_1000);
        
        std::vector<std::unique_ptr<EuMotorNode>> motors;
        std::cout << "Creating motor nodes for IDs: ";
        for (huint8 id : node_ids) {
            std::cout << (int)id << " ";
            motors.emplace_back(std::make_unique<EuMotorNode>(devIndex, id));
        }
        std::cout << std::endl;

        // --- Run Selected Tests ---
        // --- Run Selected Tests ---
        for (const auto& test_name : tests_to_run) {
            run_test_on_all_motors(test_name, test_suite[test_name], motors);
            std::this_thread::sleep_for(std::chrono::seconds(2)); // Pause between tests
        }

        // --- Cleanup ---
        std::cout << "\n\nAll specified tests completed." << std::endl;
        for (auto& motor : motors) {
            motor->disable();
        }

        // --- Cleanup ---
        std::cout << "\n\nAll specified tests completed. Disabling all motors..." << std::endl;
        for (auto& motor_ptr : motors) { // 使用一个不同的循环变量名，比如 motor_ptr
            if (motor_ptr) { // 检查指针是否有效
                motor_ptr->disable();
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "\nAn unrecoverable error occurred: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}