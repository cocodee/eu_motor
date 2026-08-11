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
#include <csignal>   // <-- 新增: 用于信号处理
#include <atomic>    // <-- 新增: 用于线程安全的退出标志

// --- 全局退出标志 (用于 monitor 模式) ---
std::atomic<bool> keep_running(true);

// --- 信号处理函数 ---
void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cout << "\nCaught Ctrl+C. Requesting shutdown..." << std::endl;
        keep_running = false;
    }
}

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
        motor.moveTo(90.0f, 180, 1000, 1000);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "Current Position: " << motor.getPosition() << " degrees" << std::endl;

        std::cout << "Moving back to 0 degrees." << std::endl;
        //motor.moveTo(0.0f, 180, 500, 500);
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
    MotorFeedbackManager& feedback_manager_= MotorFeedbackManager::getInstance();
    //feedback_manager_.registerCallback();
    huint16 baudrate;
    //harmonic_getServoCanBaudrate(1,motor.getNodeId(),&baudrate);
    std::cout<<"node id:"<<motor.getNodeId()<<" baudrate:"<<baudrate<<std::endl;
    //motor.startAutoFeedback(0,255,20);
    if (motor.configureCspMode()) {
        std::cout << "Motor configured for CSP mode. Sending a sine wave trajectory." << std::endl;
        hreal32 start_pos = 0;
        for (int i = 0; i <= 20000; ++i) {
            hreal32 target_pos = start_pos + 10.0f * std::sin(2.0 * M_PI * i / 200.0);
            int result = motor.sendCspTargetPosition(target_pos, 0, true);
            std::cout << "Sent target position: " << target_pos << " degrees" << "motor id"<< motor.getNodeId()<<" result:"<< result<< std::endl;
            motor.sendSync();
            if(result != HARMONIC_SUCCESS){
                break;
            }
            if (i % 100 == 0) {
                std::cout << "  [Position check] i=" << i << " Motor position: " << motor.getPosition() << " degrees" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
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
        // 电机必须先使能（新版固件才会上报位置 TPDO）。
        // 注：EuMotorNode::enable() 目前只清故障+切模式，没真正写控制字，这里直接写 0x6040 走 402 状态机。
        motor.write<huint16>(0x6040, 0, 0x06); // Shutdown → ReadyToSwitchOn
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        motor.write<huint16>(0x6040, 0, 0x07); // Switch On → SwitchedOn
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        motor.write<huint16>(0x6040, 0, 0x0F); // Enable Operation
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::cout << "Motor enabled (controlword 0x0F). Statusword: 0x"
                  << std::hex << motor.getStatusWord() << std::dec << std::endl;
        // Print the current TPDO configuration for debugging
        motor.printTpdoConfig();
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

// 固定 SYNC 网格 + 按需 RPDO + TPDO 固定周期反馈 的通用实现。
// 由参数 rpdo_transmit_type 选择 RPDO 的同步方式：
//   0 = 非周期同步（acyclic）：收到 RPDO 先缓存，下一个固定 SYNC 到达时才刷入驱动层生效；
//   1 = 同步周期（cyclic）：每个 SYNC 都采样 RPDO 缓冲区。
// 其余一致：
// - 主站维持固定 10ms SYNC 周期（主循环内每周期 sendSync）。
// - TPDO Transmission Type = 1（同步周期）：每个 SYNC 上报一次反馈。
// - RPDO 仅在按需指令点投递一帧（不带 SYNC），由固定 SYNC 提交。
// - 关节运动安全：测试前读取位置 P0，测试期间只偏移 ±5°，结束后回到 P0。
//
// === 总线验证（核对"RPDO ≪ SYNC，且只在写入时发"）===
// 测试运行时另开终端抓帧：candump can1 > /tmp/bus.log （结束 Ctrl+C），再统计：
//   awk '{print $2}' /tmp/bus.log | sort | uniq -c
// 预期各 COB-ID（node 为从站节点号，本测试 node 26 → 0x1A）：
//   0x80          SYNC（固定 10ms 周期）        ~420 次
//   0x200+node    RPDO1 目标位置（仅按需写入）    ~4 次（预热 1 + 指令 3）
//   0x180+node    TPDO1 反馈（每个 SYNC 一帧）    ~420 次
//   0x080+node    EMCY（仅故障时出现）
// 结论：RPDO 帧数远小于 SYNC，且只出现在按需写入时刻。
void run_feedback_sync_test(EuMotorNode& motor, huint8 rpdo_transmit_type, const char* title) {
    print_header(title);

    MotorFeedbackManager& fb_mgr = MotorFeedbackManager::getInstance();
    fb_mgr.registerCallback();

    // 1. RPDO -> 0x607A 目标位置。use_sync=false: RPDO 发送与 SYNC 解耦（按需投递）。
    if (!motor.configureCspMode(0, false)) {
        std::cerr << "Failed to configure CSP mode." << std::endl;
        return;
    }
    // [诊断] configureCspMode 内部会 enableStateMachine，此处应已 OpEnabled。
    {
        huint16 sw = motor.getStatusWord();
        std::cout << "[diag] statusword after configureCspMode: 0x" << std::hex << sw << std::dec
                  << (sw & 0x0004 ? " (OpEnabled)" : " (NOT OpEnabled)") << std::endl;
    }
    // 2. TPDO 反馈 Transmission Type = 1（同步周期）-> 每个 SYNC 上报一次。
    if (!motor.startAutoFeedback(0, 1, 0)) {
        std::cerr << "Failed to start sync feedback (TPDO type 1)." << std::endl;
        return;
    }
    // [诊断] startAutoFeedback 走了 PreOp->Start NMT，402 状态可能被复位（不再 OpEnabled）。
    {
        huint16 sw = motor.getStatusWord();
        std::cout << "[diag] statusword after startAutoFeedback: 0x" << std::hex << sw << std::dec
                  << (sw & 0x0004 ? " (OpEnabled)" : " (NOT OpEnabled)") << std::endl;
    }
    // 3. RPDO1 Transmission Type（0x1400 sub2）：0 = 非周期同步，1 = 同步周期。
    //    必须在 configureCspMode 的 Reset-Node 之后写，否则会被复位清掉。
    if (!motor.write<huint8>(0x1400, 2, rpdo_transmit_type)) {
        std::cerr << "Failed to set RPDO1 transmit type to " << (int)rpdo_transmit_type << "." << std::endl;
        return;
    }
    // 4. 重新使能 CiA 402：状态感知，只前进不降级。
    //    诊断发现：startAutoFeedback 的 PreOp->Start NMT 后驱动处于 0x1333（低 8 位与可动的 0x333 相同，
    //    即已 SwitchedOn）。若盲目写 0x06(Shutdown) 会把状态降到 0x331(ReadyToSwitchOn)，反而动不了。
    //    因此先读状态字：已 SwitchedOn(bit1) 就直接 0x0F 升到 Operation Enabled；否则才走完整 0x06→0x07→0x0F。
    auto advance_cw = [&](huint16 cw, const char* tag) -> huint16 {
        bool ok = motor.write<huint16>(0x6040, 0, cw);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        huint16 s2 = motor.getStatusWord();
        std::cout << "[diag] 0x6040=0x" << std::hex << cw << std::dec
                  << " (" << tag << ") write " << (ok ? "ok" : "FAIL")
                  << " -> sw=0x" << std::hex << s2 << std::dec
                  << (s2 & 0x0004 ? " (OpEnabled)" : " (NOT OpEnabled)") << std::endl;
        return s2;
    };
    huint16 sw_en = motor.getStatusWord();
    if (!(sw_en & 0x0004)) {
        if (sw_en & 0x0002) {          // 已 SwitchedOn -> 直接 Enable Operation（不降级）
            sw_en = advance_cw(0x0F, "EnableOp");
        } else {                        // 低于 SwitchedOn -> 完整序列
            sw_en = advance_cw(0x06, "Shutdown");
            sw_en = advance_cw(0x07, "SwitchOn");
            sw_en = advance_cw(0x0F, "EnableOp");
        }
    }
    std::cout << "[diag] final statusword after state-aware enable: 0x" << std::hex << sw_en << std::dec
              << (sw_en & 0x0004 ? " (OpEnabled)" : " (NOT OpEnabled)") << std::endl;

    // 5. 配置读回校验
    std::cout << "RPDO1 transmit type = " << (int)motor.read<huint8>(0x1400, 2)
              << " (expect " << (int)rpdo_transmit_type
              << (rpdo_transmit_type == 0 ? " = acyclic synchronous" : " = synchronous cyclic") << ")"
              << std::endl;
    std::cout << "TPDO1 transmit type = " << (int)motor.read<huint8>(0x1800, 2)
              << " (expect 1 = synchronous periodic)" << std::endl;
    motor.printTpdoConfig();

    // 6. 测试前读取关节位置 P0（电机静止），测试期间只偏移 ±5°，结束后回到 P0。
    const hreal32 p0 = motor.getPosition();
    std::cout << "Pre-test joint position P0 = " << p0 << " deg" << std::endl;

    const int    sync_period_ms = 10;
    const int    kCmds = 3;
    const hreal32 step = 5.0f;
    const hreal32 targets_deg[kCmds] = { p0 + step, p0 - step, p0 }; // +5°, -5°, 回原位
    const int    cmd_at_sync[kCmds] = { 100, 200, 300 };             // 先发 1s 纯 SYNC 建立网格，再按需下指令（t = 1s / 2s / 3s）
    const float  tol_deg = 2.0f;
    int cmd_idx = 0, sync_count = 0, rpdo_count = 0;
    bool first_fb = true;
    MotorFeedbackData fb = motor.getLatestFeedback();
    auto prev_update = fb.last_update_time;
    huint32 gap_sum_ms = 0, gap_max_ms = 0;
    int gap_n = 0;

    // 7. 主循环：固定 10ms SYNC；RPDO 仅在指令点按需投递（不带 SYNC）。
    for (int i = 0; i < 420; ++i) { // 4.2s @ 10ms（前 1s 只发 SYNC 建立网格，之后才下 RPDO 指令）
        // 网格建立后先发一帧“原位”RPDO（target=P0，不会动）：吸收固件可能丢弃“第一条非周期 RPDO”的情况。
        if (sync_count == 50) {
            motor.sendCspTargetPosition(p0, 0, /*isSync=*/false);
            std::cout << "[t=" << sync_count * sync_period_ms << "ms] >> warm-up RPDO (target=P0, no-op)"
                      << std::endl;
        }
        if (cmd_idx < kCmds && sync_count == cmd_at_sync[cmd_idx]) {
            // 先校验上一条指令是否到位（end-to-end RPDO 验证）+ 诊断：SDO 读回目标/实际位置
            if (cmd_idx > 0) {
                float err = std::fabs(fb.position_deg - targets_deg[cmd_idx - 1]);
                std::cout << (err <= tol_deg ? "  [PASS]" : "  [FAIL]")
                          << " cmd#" << cmd_idx
                          << " target=" << targets_deg[cmd_idx - 1]
                          << " deg, TPDO pos=" << fb.position_deg << " deg"
                          << ", sw=0x" << std::hex << fb.status_word << std::dec
                          << (fb.status_word & 0x0004 ? " (OpEnabled)" : " (NOT OpEnabled)");
                // [诊断] 0x607A=目标位置(脉冲) 0x6064=实际位置(脉冲)：判断 RPDO 是否到达/被应用
                try {
                    hint32 tgt_pulses = motor.read<hint32>(0x607A, 0);
                    hint32 act_pulses = motor.read<hint32>(0x6064, 0);
                    std::cout << ", SDO 0x607A=" << tgt_pulses
                              << ", SDO 0x6064=" << act_pulses << " pulses";
                } catch (const std::runtime_error& e) {
                    std::cout << ", SDO read failed: " << e.what();
                }
                std::cout << " (err=" << err << ")" << std::endl;
            }
            motor.sendCspTargetPosition(targets_deg[cmd_idx], 0, /*isSync=*/false); // 按需 RPDO
            std::cout << "[t=" << sync_count * sync_period_ms << "ms] >> on-demand RPDO -> target="
                      << targets_deg[cmd_idx] << " deg" << std::endl;
            ++rpdo_count;
            ++cmd_idx;
        }
        motor.sendSync(); // 固定周期 SYNC（同步生效的提交信号）
        ++sync_count;

        fb = motor.getLatestFeedback();
        if (fb.last_update_time != prev_update) { // TPDO 到达间隔统计（应 ≈ 10ms）
            if (!first_fb) {
                huint32 gap = (huint32)std::chrono::duration_cast<std::chrono::milliseconds>(
                    fb.last_update_time - prev_update).count();
                if (gap > 0 && gap < 100) {
                    gap_sum_ms += gap;
                    ++gap_n;
                    if (gap > gap_max_ms) gap_max_ms = gap;
                }
            }
            first_fb = false;
            prev_update = fb.last_update_time;
        }
        if (sync_count % 25 == 0) { // 每 250ms 打印位置/sync/rpdo + 实时状态字
            std::cout << "[t=" << sync_count * sync_period_ms << "ms] Pos=" << fb.position_deg
                      << " deg, sw=0x" << std::hex << fb.status_word << std::dec
                      << (fb.status_word & 0x0004 ? " (OpEnabled)" : " (NOT OpEnabled)")
                      << ", syncs=" << sync_count
                      << ", rpdos=" << rpdo_count << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sync_period_ms));
    }

    // 8. 收尾：回到测试前角度 P0（最后一条指令目标即 P0）
    if (cmd_idx == kCmds) {
        float err_home = std::fabs(fb.position_deg - p0);
        std::cout << (err_home <= tol_deg ? "  [PASS]" : "  [FAIL]")
                  << " return-home P0=" << p0 << " deg, pos=" << fb.position_deg
                  << " deg (err=" << err_home << ")" << std::endl;
    }
    std::cout << "\nSummary: syncs=" << sync_count << ", on-demand RPDOs=" << rpdo_count
              << " (RPDO only at command times), avg TPDO gap=" << (gap_n ? gap_sum_ms / gap_n : 0)
              << "ms max=" << gap_max_ms << "ms (expect ~10ms)" << std::endl;
}

// RPDO Type 0（非周期同步）：收到 RPDO 缓存，下一个 SYNC 生效（"异步发送 + 同步生效"）。
void test_feedback_sync_mode(EuMotorNode& motor) {
    run_feedback_sync_test(motor, 0, "Feedback Sync Mode (SYNC 10ms | RPDO Type0 acyclic-sync | TPDO Type1 periodic)");
}

// RPDO Type 1（同步周期）+ TPDO Type 1（同步周期）：TPDO 与 RPDO 均为同步模式。
// 与 test_feedback_sync_mode(Type 0) 的区别：不使用 advance_cw，使能直接交给
// configureCspMode(use_sync=true) 内部自带的 enableStateMachine（即"按 csp configure 的方式使能"）。
void test_feedback_sync_cyclic(EuMotorNode& motor) {
    print_header("Feedback Sync Mode (SYNC 10ms | RPDO Type1 sync-cyclic | TPDO Type1 periodic)");

    MotorFeedbackManager& fb_mgr = MotorFeedbackManager::getInstance();
    fb_mgr.registerCallback();

    // 1. CSP 模式 + RPDO Type 1（同步周期）。configureCspMode(use_sync=true) 内部会：
    //    setOperateMode(CSP) -> RPDO 映射 0x607A 且 Type=1 -> Reset-Node -> Start
    //    -> enableStateMachine(0x06/0x07/0x0F 使能电机)。这就是"按 csp configure 的方式使能"，不需要 advance_cw。
    if (!motor.configureCspMode(0, true)) {
        std::cerr << "Failed to configure CSP mode." << std::endl;
        return;
    }
    // [诊断] configureCspMode 内部会 enableStateMachine，此处应已可动。
    {
        huint16 sw = motor.getStatusWord();
        std::cout << "[diag] statusword after configureCspMode: 0x" << std::hex << sw << std::dec
                  << (sw & 0x0004 ? " (OpEnabled)" : " (NOT OpEnabled)") << std::endl;
    }

    // 2. TPDO 反馈 Transmission Type = 1（同步周期）。startAutoFeedback 会走 PreOp->Start NMT，
    //    可能复位 402 状态；此处按需求不再用 advance_cw 重新使能，靠状态字确认是否仍可动。
    if (!motor.startAutoFeedback(0, 1, 0)) {
        std::cerr << "Failed to start sync feedback (TPDO type 1)." << std::endl;
        return;
    }
    // [诊断] 若此处 NOT OpEnabled，电机将不动（这正是 advance_cw 解决的问题）。
    {
        huint16 sw = motor.getStatusWord();
        std::cout << "[diag] statusword after startAutoFeedback: 0x" << std::hex << sw << std::dec
                  << (sw & 0x0004 ? " (OpEnabled)" : " (NOT OpEnabled)")
                  << " (若 NOT OpEnabled 电机不会动)" << std::endl;
    }

    // 3. 配置读回校验
    std::cout << "RPDO1 transmit type = " << (int)motor.read<huint8>(0x1400, 2)
              << " (expect 1 = synchronous cyclic)" << std::endl;
    std::cout << "TPDO1 transmit type = " << (int)motor.read<huint8>(0x1800, 2)
              << " (expect 1 = synchronous periodic)" << std::endl;
    motor.printTpdoConfig();

    // 4. 测试前读取关节位置 P0（电机静止），测试期间只偏移 ±5°，结束后回到 P0。
    const hreal32 p0 = motor.getPosition();
    std::cout << "Pre-test joint position P0 = " << p0 << " deg" << std::endl;

    const int    sync_period_ms = 10;
    const int    kCmds = 3;
    const hreal32 step = 5.0f;
    const hreal32 targets_deg[kCmds] = { p0 + step, p0 - step, p0 }; // +5°, -5°, 回原位
    const int    cmd_at_sync[kCmds] = { 100, 200, 300 };             // 先发 1s 纯 SYNC 建立网格（t = 1s / 2s / 3s）
    const float  tol_deg = 2.0f;
    int cmd_idx = 0, sync_count = 0, rpdo_count = 0;
    bool first_fb = true;
    MotorFeedbackData fb = motor.getLatestFeedback();
    auto prev_update = fb.last_update_time;
    huint32 gap_sum_ms = 0, gap_max_ms = 0;
    int gap_n = 0;

    // 5. 主循环：固定 10ms SYNC；RPDO 仅在指令点按需投递（不带 SYNC）。
    for (int i = 0; i < 420; ++i) { // 4.2s @ 10ms（前 1s 只发 SYNC 建立网格，之后才下 RPDO 指令）
        // 网格建立后先发一帧“原位”RPDO（target=P0，不会动）：与 Type 0 测试保持一致，吸收首条同步 RPDO 可能丢失的情况。
        if (sync_count == 50) {
            motor.sendCspTargetPosition(p0, 0, /*isSync=*/false);
            std::cout << "[t=" << sync_count * sync_period_ms << "ms] >> warm-up RPDO (target=P0, no-op)"
                      << std::endl;
        }
        if (cmd_idx < kCmds && sync_count == cmd_at_sync[cmd_idx]) {
            // 先校验上一条指令是否到位（end-to-end RPDO 验证）+ 诊断：SDO 读回目标/实际位置
            if (cmd_idx > 0) {
                float err = std::fabs(fb.position_deg - targets_deg[cmd_idx - 1]);
                std::cout << (err <= tol_deg ? "  [PASS]" : "  [FAIL]")
                          << " cmd#" << cmd_idx
                          << " target=" << targets_deg[cmd_idx - 1]
                          << " deg, TPDO pos=" << fb.position_deg << " deg"
                          << ", sw=0x" << std::hex << fb.status_word << std::dec
                          << (fb.status_word & 0x0004 ? " (OpEnabled)" : " (NOT OpEnabled)");
                // [诊断] 0x607A=目标位置(脉冲) 0x6064=实际位置(脉冲)：判断 RPDO 是否到达/被应用
                try {
                    hint32 tgt_pulses = motor.read<hint32>(0x607A, 0);
                    hint32 act_pulses = motor.read<hint32>(0x6064, 0);
                    std::cout << ", SDO 0x607A=" << tgt_pulses
                              << ", SDO 0x6064=" << act_pulses << " pulses";
                } catch (const std::runtime_error& e) {
                    std::cout << ", SDO read failed: " << e.what();
                }
                std::cout << " (err=" << err << ")" << std::endl;
            }
            motor.sendCspTargetPosition(targets_deg[cmd_idx], 0, /*isSync=*/false); // 按需 RPDO
            std::cout << "[t=" << sync_count * sync_period_ms << "ms] >> on-demand RPDO -> target="
                      << targets_deg[cmd_idx] << " deg" << std::endl;
            ++rpdo_count;
            ++cmd_idx;
        }
        motor.sendSync(); // 固定周期 SYNC（同步生效的提交信号）
        ++sync_count;

        fb = motor.getLatestFeedback();
        if (fb.last_update_time != prev_update) { // TPDO 到达间隔统计（应 ≈ 10ms）
            if (!first_fb) {
                huint32 gap = (huint32)std::chrono::duration_cast<std::chrono::milliseconds>(
                    fb.last_update_time - prev_update).count();
                if (gap > 0 && gap < 100) {
                    gap_sum_ms += gap;
                    ++gap_n;
                    if (gap > gap_max_ms) gap_max_ms = gap;
                }
            }
            first_fb = false;
            prev_update = fb.last_update_time;
        }
        if (sync_count % 25 == 0) { // 每 250ms 打印位置/sync/rpdo + 实时状态字
            std::cout << "[t=" << sync_count * sync_period_ms << "ms] Pos=" << fb.position_deg
                      << " deg, sw=0x" << std::hex << fb.status_word << std::dec
                      << (fb.status_word & 0x0004 ? " (OpEnabled)" : " (NOT OpEnabled)")
                      << ", syncs=" << sync_count
                      << ", rpdos=" << rpdo_count << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sync_period_ms));
    }

    // 6. 收尾：回到测试前角度 P0（最后一条指令目标即 P0）
    if (cmd_idx == kCmds) {
        float err_home = std::fabs(fb.position_deg - p0);
        std::cout << (err_home <= tol_deg ? "  [PASS]" : "  [FAIL]")
                  << " return-home P0=" << p0 << " deg, pos=" << fb.position_deg
                  << " deg (err=" << err_home << ")" << std::endl;
    }
    std::cout << "\nSummary: syncs=" << sync_count << ", on-demand RPDOs=" << rpdo_count
              << " (RPDO only at command times), avg TPDO gap=" << (gap_n ? gap_sum_ms / gap_n : 0)
              << "ms max=" << gap_max_ms << "ms (expect ~10ms)" << std::endl;
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

        // --- 1. Gear Ratio (SDO 0x6091) ---
        std::cout << "\n--- Gear Ratio (SDO 0x6091) ---" << std::endl;
        try {
            huint32 motor_rev = motor.read<huint32>(0x6091, 1);
            huint32 shaft_rev = motor.read<huint32>(0x6091, 2);
            std::cout << "Motor revolutions: " << motor_rev << std::endl;
            std::cout << "Shaft revolutions: " << shaft_rev << std::endl;
            if (shaft_rev != 0) {
                std::cout << "Gear ratio (motor/shaft): " << motor_rev << "/" << shaft_rev << std::endl;
            }
        } catch (const std::runtime_error& e) {
            std::cout << "Could not read gear ratio SDO: " << e.what() << std::endl;
        }

        // --- New Motor Gear Ratio (SDO 0x26A2 / 0x26A3) ---
        // 新版电机的减速比计算依据：两个字段都是 UINT16（2 字节无符号）。
        // 错误码 7 = SDO 读超时无响应，可能是子索引不对或本电机无此对象，因此 sub 0/1 都试并打印结果。
        std::cout << "\n--- New Motor Gear Ratio (SDO 0x26A2 / 0x26A3) ---" << std::endl;
        huint16 new_motor_rev = 0;
        huint16 new_shaft_rev = 0;
        bool motor_ok = false;
        bool shaft_ok = false;
        for (huint8 sub : {0, 1}) {
            if (!motor_ok) {
                try {
                    new_motor_rev = motor.read<huint16>(0x26A2, sub);
                    motor_ok = true;
                    std::cout << "  0x26A2 sub " << (int)sub << " = " << new_motor_rev << std::endl;
                } catch (const std::runtime_error& e) {
                    std::cout << "  0x26A2 sub " << (int)sub << " failed: " << e.what() << std::endl;
                }
            }
            if (!shaft_ok) {
                try {
                    new_shaft_rev = motor.read<huint16>(0x26A3, sub);
                    shaft_ok = true;
                    std::cout << "  0x26A3 sub " << (int)sub << " = " << new_shaft_rev << std::endl;
                } catch (const std::runtime_error& e) {
                    std::cout << "  0x26A3 sub " << (int)sub << " failed: " << e.what() << std::endl;
                }
            }
        }
        if (motor_ok && shaft_ok && new_shaft_rev != 0) {
            std::cout << "Gear ratio (motor/shaft): " << new_motor_rev << "/" << new_shaft_rev << std::endl;
        }

        // --- New Motor Encoder Resolution (SDO 0x2023 / 0x2025) ---
        // 新版固件：0x2023=编码器分辨率(LPR)，0x2025=单圈分辨率。都按 UINT32 读原始值。
        std::cout << "\n--- New Motor Encoder Resolution (SDO 0x2023 / 0x2025) ---" << std::endl;
        for (huint16 obj_idx : {0x2023, 0x2025}) {
            try {
                huint32 res = motor.read<huint32>(obj_idx, 0);
                std::cout << "  0x" << std::hex << std::uppercase << obj_idx << std::dec << std::nouppercase
                          << " sub 0 = " << res << std::endl;
            } catch (const std::runtime_error& e) {
                std::cout << "  0x" << std::hex << std::uppercase << obj_idx << std::dec << std::nouppercase
                          << " sub 0 failed: " << e.what() << std::endl;
            }
        }

        huint16 status = motor.getStatusWord();
        print_status_word(status);

        huint16 error_code = motor.getErrorCode();
        std::cout << "Current Error Code: 0x" << std::hex << std::setw(4) << std::setfill('0') << error_code << std::dec << std::endl;

        // --- Motor Position (SDO read) ---
        std::cout << "\n--- Motor Position (SDO read) ---" << std::endl;
        try {
            hreal32 position = motor.getPosition();
            std::cout << "Current Position: " << position << " degrees" << std::endl;
        } catch (const std::runtime_error& e) {
            std::cout << "Could not read motor position: " << e.what() << std::endl;
        }

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
void test_clear_fault(EuMotorNode& motor) {
    std::cout << "\nTesting clear fault for motor " << motor.getNodeId() << "..." << std::endl;
    try {
        if (!motor.clearFault()) {
            std::cout << "Failed to clear fault for motor " << motor.getNodeId() << "." << std::endl;
        } else {
            std::cout << "Fault cleared for motor " << motor.getNodeId() << "." << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "An error occurred during the fault clear test for motor ";
    }
}

// Function to display help message
void print_usage(const char* prog_name, const std::map<std::string, std::function<void(EuMotorNode&)>>& tests) {
    std::cout << "Usage: " << prog_name << " --dev <device_index> --motors <id1> <id2>... --tests <test1> <test2>..." << std::endl;
    std::cout << "   or: " << prog_name << " --dev <device_index> --motors <id1>... --tests all" << std::endl;
    std::cout << "\nParameters:" << std::endl;
    std::cout << "  --dev <index>        Specify the CAN device index (e.g., 0 for the first device)." << std::endl;
    std::cout << "  --motors <id...>     Specify one or more motor node IDs to test." << std::endl;
    std::cout << "  --tests <name...>    Specify one or more tests to run, or 'all' to run all tests." << std::endl;
    std::cout << "\nAvailable tests:" << std::endl;
    for (const auto& pair : tests) {
        std::cout << "  - " << pair.first << std::endl;
    }
    std::cout << "\nExample: " << prog_name << " --dev 0 --motors 11 12 --tests pp status" << std::endl;
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
    test_suite["feedback_sync"] = test_feedback_sync_mode;
    test_suite["feedback_sync_cyclic"] = test_feedback_sync_cyclic;
    test_suite["status"] = test_status_and_errors;
    test_suite["clearfault"] = test_clear_fault;

    // 新增 devIndex 变量并设置一个默认值
    huint8 devIndex = 0; 
    bool devIndexSet = false;
    std::vector<huint8> motor_node_ids; // 重命名变量以提高清晰度
    std::vector<std::string> tests_to_run;
    
    // 我们至少需要一个参数来显示帮助信息
    if (argc < 2) {
        print_usage(argv[0], test_suite);
        return 1;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dev") {
            // --dev 后面必须跟一个数字
            if (i + 1 < argc) {
                try {
                    devIndex = static_cast<huint8>(std::stoi(argv[++i]));
                    devIndexSet = true;
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Error: Invalid number for --dev: " << argv[i] << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "Error: --dev option requires one argument." << std::endl;
                return 1;
            }
        } else if (arg == "--motors") {
            // --motors 后面可以跟一个或多个数字
            i++; // 移动到第一个电机ID
            while (i < argc && argv[i][0] != '-') {
                try {
                    motor_node_ids.push_back(static_cast<huint8>(std::stoi(argv[i])));
                    i++;
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Error: Invalid number for --motors: " << argv[i] << std::endl;
                    return 1;
                }
            }
            i--; // 回退一步，因为外层 for 循环会 i++
        } else if (arg == "--tests") {
            // --tests 后面可以跟一个或多个测试名，或 'all'
            i++; // 移动到第一个测试名
            if (i < argc && std::string(argv[i]) == "all") {
                for (const auto& pair : test_suite) {
                    tests_to_run.push_back(pair.first);
                }
            } else {
                while (i < argc && argv[i][0] != '-') {
                    if (test_suite.count(argv[i])) {
                        tests_to_run.push_back(argv[i]);
                    } else {
                        std::cerr << "Warning: Test '" << argv[i] << "' not found. Skipping." << std::endl;
                    }
                    i++;
                }
                i--; // 回退一步
            }
        } else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0], test_suite);
            return 0;
        }
    }

    // 检查必需的参数是否已提供
    if (!devIndexSet || motor_node_ids.empty() || tests_to_run.empty()) {
        std::cerr << "Error: You must specify --dev, at least one motor ID with --motors, and at least one test with --tests." << std::endl;
        print_usage(argv[0], test_suite);
        return 1;
    }

    try {
        // --- Setup ---
        CanNetworkManager canNetworkManager;
        canNetworkManager.initDevice(harmonic_DeviceType_Canable, devIndex, harmonic_Baudrate_1000);
        
        std::vector<std::unique_ptr<EuMotorNode>> motors;
        std::cout << "Creating motor nodes for IDs: ";
        for (huint8 id : motor_node_ids) {
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