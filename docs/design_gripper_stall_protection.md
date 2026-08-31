# 夹爪电机 CSP 模式恒力夹持与堵转防护 — 设计文档

## 1. 目标、范围与约束

夹爪在 CSP（周期同步位置）模式下闭合接触物体时，持续向完全闭合位置发送目标会造成堵转和过热。本设计在**始终保持 CSP 模式**的前提下，实现：

1. 通过 CiA 402 对象 `0x6072` 建立驱动器侧的最大力矩硬上限；
2. 通过 TPDO 反馈的 `0x6077 Actual Torque` 检测接触；
3. 接触后在 CSP 内部持续微调位置目标，使实际力矩维持在期望夹持力附近；
4. 发生反馈丢失、持续超载或故障时进入安全停止。

### 范围边界

- **不切换 CST，也不调用运行时模式切换。** 从初始化、接近、接触、恒力夹持到释放，驱动器均保持 CSP。
- **本次不维护且不修改 `ros2_ws/` 下的 ros2_control、URDF 或插件代码。**
- 仅交付 C++ SDK、pybind11 绑定、纯 SDK 示例和测试；Python 是唯一支持的上层控制入口。
- SDK/pybind11 的所有位置参数及反馈均为**度**；Python 调用方如使用弧度，必须自行转换。
- 这是利用 CSP 位置误差构建的**软件外环恒力**。其稳态误差和响应受 TPDO 周期、摩擦、传动回差、物体刚度及位置环参数影响；必须按实际夹爪标定，不能宣称等同于 CST 或驱动器原生力控。

## 2. 方案选型

| 方案 | 结论 | 原因 |
|---|---|---|
| CSP + `0x6072` 力矩限制 | 采用 | 硬件级安全上限；单独使用仍会以受限力矩堵转。 |
| CSP + 冻结实际位置 | 不采用 | 可停止推进，但冻结后力矩下降，不能恒力夹持。 |
| **CSP + 外环力控微调位置目标** | **采用** | 保持 CSP，不切 CST；接触后按实际力矩增减位置目标，以近似维持夹持力。 |
| CSP→CST | 不采用 | 用户明确不希望切换 CST；且完整模式切换不适合作为本次实时控制路径。 |
| TorquePositionFixed | 不采用 | 厂商行为和限制未验证，不能用于安全功能。 |

## 3. 状态机与数据流

```
Python 控制循环（周期调用 send_csp_target_position）
  │  用户目标：闭合或张开（度）
  ▼
EuMotorNode（始终 CSP）
  ├─ Layer 1：配置 CSP 前写入并回读 0x6072
  └─ Layer 2：新鲜 TPDO 力矩反馈驱动状态机

  APPROACH ──连续 N 个闭合方向接触样本──> HOLD
      ▲                                           │
      └──── 用户给出张开方向目标 ──────────────────┘
                         │
          反馈超时 / 严重超载 / 驱动故障
                         ▼
                      SAFE_STOP
```

| 状态 | 实际发送的 CSP 位置目标 | 行为 |
|---|---|---|
| `APPROACH` | 用户目标 | 正常闭合或张开；仅闭合方向参与接触检测。 |
| `HOLD` | 内部 `hold_target_position_deg_` | 每个新鲜反馈样本按力矩误差小步调整目标，维持期望夹持力。 |
| `SAFE_STOP` | 当前实际位置（不再推进） | 退出 HOLD，停止施加额外位置误差；需用户发明确张开指令或显式清除后退出。 |

### 关键运行要求

外环仅在 `sendCspTargetPosition()` 被调用时执行。因此 Python 在 `APPROACH` 和 `HOLD` 均必须持续以固定周期调用 `send_csp_target_position(close_position_deg)`；推荐周期不慢于 TPDO 周期。若 Python 停止调用，驱动器会维持最后一个 CSP 目标，但外环无法补偿物体滑移、扰动或力矩变化。

示例 TPDO 周期为 20 ms。若 `contact_detect_consecutive_samples=5`，接触确认最坏约为 100 ms；不得以 CSP 插补周期或 Python 循环次数替代独立反馈样本数。

## 4. 方向、接触与恒力控制

定义：

```
closing_sign = sign(close_position_deg - open_position_deg)
```

这支持正反安装。仅当：

```
(user_target_deg - actual_position_deg) * closing_sign > position_tolerance_deg
```

时视为闭合。张开方向高力矩不报告为夹持；如需保护张开受阻，应另定义独立的 `open_collision` 功能。

### 接触判定

仅在以下条件同时成立时，对一个 TPDO 样本累计一次：

- 反馈时间戳是新样本；
- 反馈不超过 `feedback_timeout_ms`；
- 当前命令处于闭合方向；
- `abs(actual_torque_milli) >= contact_detect_threshold_milli`。

达到 `contact_detect_consecutive_samples` 后进入 `HOLD`，并以当前实际位置初始化 `hold_target_position_deg_`。同一时间戳不得重复计数；反馈未到达、过期或回调未注册时重置计数，不得复用旧高力矩触发接触。

### HOLD 外环

每收到一个新鲜 TPDO 样本，计算：

```
torque_error = hold_torque_milli - abs(actual_torque_milli)
delta_deg = clamp(force_kp_deg_per_milli * torque_error,
                  -max_hold_step_deg, +max_hold_step_deg)
hold_target += closing_sign * delta_deg
hold_target = clamp(hold_target, open_position_deg, close_position_deg)
```

- `torque_error > 0`：力不足，向闭合方向小步推进；`torque_error < 0`：力过大，反向小步释放。
- 当 `abs(torque_error) <= hold_torque_tolerance_milli` 时令 `delta_deg = 0`，避免高频抖动。
- `max_hold_target_offset_deg` 限制 `hold_target` 相对接触位置的最大闭合偏移，防止长期积分式推进；达到该限位仍力不足时报告 `hold_unreachable`，不得继续推进。
- 首版只使用带死区和单样本限幅的 P 控制，不引入积分项，避免摩擦和反馈延迟导致积分饱和。若硬件测试表明稳态误差不可接受，再单独评审带 anti-windup 的 I 项。

## 5. API 与配置

文件：`include/eu_motor.h`

```cpp
enum class GripperState {
    Disabled,
    Approach,
    Hold,
    SafeStop,
};

struct GripperConfig {
    hreal32 open_position_deg = 0.0f;
    hreal32 close_position_deg = 90.0f;

    hint16 torque_limit_milli = 500;                // 0x6072 硬上限
    hint16 hold_torque_milli = 300;                 // 目标夹持力
    hint16 hold_torque_tolerance_milli = 30;        // 力矩死区
    hint16 contact_detect_threshold_milli = 180;    // 接触判定阈值
    hint16 overload_threshold_milli = 450;          // 软件严重超载阈值
    int contact_detect_consecutive_samples = 5;

    hreal32 force_kp_deg_per_milli = 0.002f;
    hreal32 max_hold_step_deg = 0.2f;               // 每个新样本的最大修正
    hreal32 max_hold_target_offset_deg = 5.0f;      // 相对接触点的最大闭合偏移
    hreal32 position_tolerance_deg = 1.0f;
    huint16 feedback_timeout_ms = 100;
};

bool setGripperConfig(const GripperConfig& config);
void disableGripperMode();
bool isGripperMode() const;
GripperState getGripperState() const;
bool isGripDetected() const;       // state == Hold
hreal32 getGripPosition() const;   // 接触位置（度）
hint16 getHoldTorqueError() const; // 最新目标与实际力矩差
void clearGripperSafeStop();

bool setTorqueLimit(hint16 torque_milli);
hint16 getTorqueLimit();
```

配置校验必须拒绝以下情况，并保持夹爪模式禁用：开闭位置相等；非正阈值；`contact_detect_threshold_milli > hold_torque_milli`；`hold_torque_milli + hold_torque_tolerance_milli >= overload_threshold_milli`；`overload_threshold_milli >= torque_limit_milli`；样本数、反馈超时或步长非正；目标偏移或容差为负。默认值不是所有夹爪的安全标定值。

不在本次暴露 `setMotorRatedTorque`、I²t、Torque Window 等高风险驱动参数写接口；它们属于驱动器标定/保护范围，误写会改变硬件安全边界。

### 5.1 配置项说明与建议初始值

位置单位均为度；所有力矩数值均为额定力矩的千分比。下表中的数值是保守起点，必须根据实际电机、减速比、夹爪结构、被夹物和温升测试重新标定。

| 配置项 | 含义 | 建议配置 |
|---|---|---|
| `open_position_deg` | 完全张开位置 | 使用标定后的真实安全张开角，例如 `0`。 |
| `close_position_deg` | 空载完全闭合位置 | 使用标定后的机械安全闭合角，例如 `90`；不得超过机械硬限位。 |
| `torque_limit_milli` | 驱动器 `0x6072` 硬件力矩上限 | 从低值开始；建议初始 `300`，确认夹持能力和温升后再逐步提高。不能假定默认 `500` 对所有夹爪安全。 |
| `hold_torque_milli` | 恒力夹持目标 | 通常取硬上限的 50%–70%；上限为 `300` 时从 `180` 起步。 |
| `hold_torque_tolerance_milli` | 目标力矩死区 | 建议 `15–30`。过小容易抖动，过大会增加夹持力波动。 |
| `contact_detect_threshold_milli` | 从接近阶段进入保持阶段的接触阈值 | 建议为目标力矩的 50%–80%；目标为 `180` 时可设 `120`。 |
| `overload_threshold_milli` | 软件严重超载阈值，触发 `SafeStop` | 必须低于硬上限；上限为 `300` 时可设 `270`。 |
| `contact_detect_consecutive_samples` | 连续高力矩的独立 TPDO 样本数 | 推荐 `3–5`；TPDO 周期为 20 ms 时，`5` 的接触确认约为 100 ms。 |
| `force_kp_deg_per_milli` | 力矩误差转为位置修正的比例增益 | 从 `0.001` 起。响应慢可逐步增加到 `0.002–0.005`；出现振荡、啸叫或反复开合时降低。 |
| `max_hold_step_deg` | 每个新反馈样本的最大位置修正 | 推荐 `0.05–0.2`；刚性夹爪优先从 `0.05` 起。 |
| `max_hold_target_offset_deg` | 保持期间相对接触点的最大额外闭合量 | 推荐 `1–3`；这是防止力不足时无限推进的关键保护。 |
| `position_tolerance_deg` | 判定闭合/张开方向的最小位移 | 推荐 `0.5–1.0`，且应大于编码器噪声与机械回差。 |
| `feedback_timeout_ms` | 力矩反馈超时阈值，超时进入安全停止 | 建议为 TPDO 周期的 3–5 倍；TPDO 为 20 ms 时设 `100`。 |

推荐保守初始配置：

```python
cfg.open_position_deg = 0.0
cfg.close_position_deg = 90.0

cfg.torque_limit_milli = 300
cfg.hold_torque_milli = 180
cfg.hold_torque_tolerance_milli = 20
cfg.contact_detect_threshold_milli = 120
cfg.overload_threshold_milli = 270
cfg.contact_detect_consecutive_samples = 5

cfg.force_kp_deg_per_milli = 0.001
cfg.max_hold_step_deg = 0.05
cfg.max_hold_target_offset_deg = 2.0
cfg.position_tolerance_deg = 0.5
cfg.feedback_timeout_ms = 100
```

配置必须满足：

```text
0 < contact_detect_threshold_milli <= hold_torque_milli
hold_torque_milli + hold_torque_tolerance_milli < overload_threshold_milli
overload_threshold_milli < torque_limit_milli
```

调参顺序：先设低 `torque_limit_milli` 并确认 `0x6072` 硬件上限生效；再确定可安全夹住物体的 `hold_torque_milli`；随后用 `force_kp_deg_per_milli` 和 `max_hold_step_deg` 消除振荡；最后根据误触发与接触响应调整阈值和连续样本数。

## 6. 实现细节

### 6.1 Layer 1：硬件上限

在 `configureCspMode()` 开始处，若夹爪模式已启用：

1. 调用 `setTorqueLimit(config.torque_limit_milli)`；
2. 调用 `getTorqueLimit()` 回读并比较；
3. 写入、回读失败或不一致时返回 `false`，不得继续 CSP 配置和使能。

必须在目标固件上确认 `0x6072` 对 CSP 输出生效，不能只依据 API 存在推断。

### 6.2 Layer 2：保护状态

私有状态至少包括：

```cpp
GripperState gripper_state_ = GripperState::Disabled;
GripperConfig gripper_config_;
hreal32 grip_position_deg_ = 0.0f;
hreal32 hold_target_position_deg_ = 0.0f;
hint16 hold_torque_error_milli_ = 0;
int contact_sample_count_ = 0;
std::chrono::steady_clock::time_point last_processed_feedback_time_{};
```

`sendCspTargetPosition()` 的逻辑顺序必须为：

1. 读取缓存反馈并检查时间戳；没有新鲜反馈时，重置接触去抖并保持当前安全目标，不使用旧反馈控制；
2. 用户发出明确张开目标时：退出 `HOLD`/ `SAFE_STOP`，发送用户目标；
3. `APPROACH`：对新鲜闭合样本累计接触；确认后初始化 `hold_target_position_deg_` 并进入 `HOLD`；
4. `HOLD`：只在新鲜样本上运行一次 P 外环；发送内部 `hold_target_position_deg_`，忽略重复的用户闭合目标；
5. 实际力矩达到 `overload_threshold_milli`、反馈中出现故障或反馈连续超时：进入 `SAFE_STOP`，发送当前位置或最近安全位置；记录原因；
6. 仅对最终的 `effective_target` 执行 `angleToPulses()` 与 CSP PDO 发送。

实现不得在实时发送路径打印每周期日志；仅记录状态迁移、超时与异常。

### 6.3 pybind11

文件：`src/bindings.cpp`。绑定 `GripperState`、`GripperConfig` 的全部字段以及：

```cpp
.def("set_gripper_config", &EuMotorNode::setGripperConfig)
.def("disable_gripper_mode", &EuMotorNode::disableGripperMode)
.def("is_gripper_mode", &EuMotorNode::isGripperMode)
.def("get_gripper_state", &EuMotorNode::getGripperState)
.def("is_grip_detected", &EuMotorNode::isGripDetected)
.def("get_grip_position", &EuMotorNode::getGripPosition)
.def("get_hold_torque_error", &EuMotorNode::getHoldTorqueError)
.def("clear_gripper_safe_stop", &EuMotorNode::clearGripperSafeStop)
.def("set_torque_limit", &EuMotorNode::setTorqueLimit)
.def("get_torque_limit", &EuMotorNode::getTorqueLimit)
```

Python 示例必须持续发送闭合目标，直到状态进入 `Hold`；保持期间也必须继续以固定周期调用该接口，使外环能补偿力矩变化。

### 6.4 示例与构建

新增 `example/test_gripper_csp_mode.cpp`，并在 `example/CMakeLists.txt` 添加：

```cmake
add_custom_test(test_gripper_csp_mode)
```

示例顺序：创建节点 → 设置并校验 `GripperConfig` → `configureCspMode()` → `startAutoFeedback(0, 255, 20)` → 注册回调 → 以 20 ms 周期发送闭合位置 → 观察 `Hold` 状态与力矩误差 → 发送张开位置释放。

## 7. 验证方案

### 7.1 自动化测试

将状态机/外环提取为可注入反馈的私有 helper 或无公共 API 的内部类，并覆盖：

| 测试 | 期望 |
|---|---|
| 配置合法性 | 所有不等式、范围和步长约束均拒绝启用 |
| 接触去抖 | 同一时间戳多次调用不重复计数；第 N 个新样本才进入 Hold |
| 恒力方向 | 力不足时只向闭合方向修正，力过大时只向张开方向修正 |
| 死区与限幅 | 死区内不移动；每样本移动不超过最大步长；总偏移不超过最大偏移 |
| 正反安装 | 两种 `closing_sign` 均正确接近、保持和释放 |
| 反馈超时 | 不以旧高力矩进入 Hold；Hold 中超时进入 SafeStop |
| 严重超载/故障 | 进入 SafeStop，不再向闭合方向推进 |
| 回归 | 未启用夹爪模式时，CSP 目标完全透传 |

### 7.2 Python 冒烟测试

```python
import eu_motor_py as eu

m = eu.EuMotorNode(0, 31)
cfg = eu.GripperConfig()
cfg.open_position_deg = 0.0
cfg.close_position_deg = 90.0
cfg.torque_limit_milli = 500
cfg.hold_torque_milli = 300
cfg.contact_detect_threshold_milli = 180
cfg.overload_threshold_milli = 450
assert m.set_gripper_config(cfg)
assert m.get_gripper_state() == eu.GripperState.Approach
```

### 7.3 硬件测试

| 测试项 | 期望 |
|---|---|
| `0x6072` 生效 | 回读值正确；TPDO `0x6077` 或 `getTorque()` 显示实际力矩不超过允许测量误差范围。 |
| 阶跃接触 | 接触后进入 Hold，峰值、稳定时间和稳态误差满足标定指标，且不进入机械硬堵转。 |
| 柔软/刚硬物体 | 在不同刚度和摩擦条件下无明显振荡、啸叫或持续推进。 |
| 扰动恢复 | 轻微移动被夹物后，外环能在目标误差范围内恢复夹持力。 |
| 张开释放 | 明确张开目标立即退出 Hold，夹爪正常张开。 |
| 反馈丢失/故障 | 不以旧反馈控制；进入 SafeStop；硬件上限持续有效。 |
| 30 分钟保持 | 温度、故障码、力矩误差和目标位置均记录；以实际热指标验收。 |

## 8. 改动文件清单

| 文件 | 操作 | 说明 |
|---|---|---|
| `include/eu_motor.h` | 修改 | 配置、状态枚举、最小必要 API、私有状态 |
| `src/eu_motor.cpp` | 修改 | CSP 内外环、硬件上限回读、安全状态机 |
| `src/bindings.cpp` | 修改 | Python 状态、配置与查询绑定 |
| `example/test_gripper_csp_mode.cpp` | 新增 | 纯 SDK/C++ 示例 |
| `example/CMakeLists.txt` | 修改 | 添加示例构建目标 |
| `ros2_ws/**` | **不修改** | 明确不在维护范围内 |
