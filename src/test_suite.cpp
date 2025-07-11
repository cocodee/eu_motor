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
            hreal32 target_pos = start_pos + 45.0f * std::sin(2.0 * M_PI * i / 200.0);
            motor.sendCspTargetPosition(target_pos, 0, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
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
    if (motor.startAutoFeedback(0, 254, 20)) {
        std::cout << "Automatic feedback started. Moving motor to 180 degrees..." << std::endl;
        motor.enable(harmonic_OperateMode_ProfilePosition);
        motor.moveTo(90.0f, 90, 500, 500);

        // Poll for feedback for a few seconds while it moves
        for (int i = 0; i < 50; ++i) {
            MotorFeedbackData data = motor.getLatestFeedback();
            auto time_since_update = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - data.last_update_time
            ).count();

            std::cout << "Feedback: Pos=" << data.position_deg 
                      << " deg, Vel=" << data.velocity_dps 
                      << " dps (updated " << time_since_update << " ms ago)";
            
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


// Function to display help message
void print_usage(const char* prog_name, const std::map<std::string, std::function<void(EuMotorNode&)>>& tests) {
    std::cout << "Usage: " << prog_name << " [test_name_1] [test_name_2] ..." << std::endl;
    std::cout << "   or: " << prog_name << " all" << std::endl;
    std::cout << "\nAvailable tests:" << std::endl;
    for (const auto& pair : tests) {
        std::cout << "  - " << pair.first << std::endl;
    }
    std::cout << "\nExample: " << prog_name << " pp pv" << std::endl;
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

    // --- Argument Parsing ---
    if (argc < 4 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
        print_usage(argv[0], test_suite);
        return 0;
    }
    
    int nodeId_ = 1;
    if (argc > 3 && std::string(argv[1]) == "--dev") {
        nodeId_ = std::stoi(argv[2]);
    }

    std::vector<std::string> tests_to_run;
    if (std::string(argv[3]) == "all") {
        for (const auto& pair : test_suite) {
            tests_to_run.push_back(pair.first);
        }
    } else {
        for (int i = 3; i < argc; ++i) {
            std::string test_name = argv[i];
            if (test_suite.find(test_name) != test_suite.end()) {
                tests_to_run.push_back(test_name);
            } else {
                std::cerr << "Warning: Test '" << test_name << "' not found. Skipping." << std::endl;
            }
        }
    }
    
    if (tests_to_run.empty()) {
        std::cerr << "No valid tests specified." << std::endl;
        print_usage(argv[0], test_suite);
        return 1;
    }


    try {
        // --- Setup ---
        huint8 devIndex = 0;
        huint8 nodeId = nodeId_;
        CanNetworkManager::getInstance().initDevice(harmonic_DeviceType_Canable, devIndex, harmonic_Baudrate_1000);
        
        EuMotorNode motor(devIndex, nodeId);
        std::cout << "Motor Node " << (int)nodeId << " created. Initializing..." << std::endl;

        // --- Run Selected Tests ---
        for (const auto& test_name : tests_to_run) {
            // Reset motor state before each test for consistency
            motor.clearFault();
            motor.disable();
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // give it a moment to settle

            // Run the test function
            test_suite[test_name](motor);
        }

        // --- Cleanup ---
        std::cout << "\n\nAll specified tests completed." << std::endl;
        motor.disable(); // Final disable

    } catch (const std::exception& e) {
        std::cerr << "\nAn unrecoverable error occurred: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}