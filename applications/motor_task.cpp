#include <atomic>
#include <cmath>
#include <cstring>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

// 使用 C 板 CAN1
sp::CAN motor_can(&hcan1);

// A 为 ID 1，B 为 ID 2；两台都使用电流控制
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);

// 遥控器在 uart_task.cpp 中创建
extern sp::DBus remote;

// IMU 任务发布的 C 板 yaw、yaw 角速度及更新时间
extern std::atomic<float> imu_yaw_rad;
extern std::atomic<float> imu_yaw_rate_rad_s;
extern std::atomic<uint32_t> imu_yaw_stamp_ms;

// 给 SerialPlot 显示的物理目标角度，单位 rad
std::atomic<float> a_target_rad{0.0f};
std::atomic<float> b_target_rad{0.0f};

namespace
{
constexpr float TWO_PI = 6.283185307f;

// 统一三者的逻辑正方向。若实测方向相反，只修改对应符号。
constexpr float YAW_SIGN = 1.0f;
constexpr float A_SIGN = 1.0f;
constexpr float B_SIGN = 1.0f;

// 直接角度 PD。先维持较小力矩，确认方向后再逐步调参。
constexpr float KP_A = 1.0f;
constexpr float KD_A = 0.03f;
constexpr float KP_B = 1.0f;
constexpr float KD_B = 0.03f;
// 基础联动调试阶段先使用较保守但足以克服静摩擦的力矩上限。
constexpr float MAX_LINK_TORQUE = 0.15f;
constexpr float MAX_RESET_TORQUE = 0.10f;

// 单次 IMU yaw 异常跳变阈值：45°。
constexpr float MAX_YAW_STEP = 0.785398f;

// 手动输入判定初值，需要结合 SerialPlot 实测调整。
constexpr float BOARD_RATE_THRESHOLD = 0.05f;     // rad/s
constexpr float MANUAL_ANGLE_THRESHOLD = 0.035f; // 约 2°
constexpr float MANUAL_SPEED_THRESHOLD = 0.15f;  // rad/s
constexpr float MANUAL_RELEASE_SPEED = 0.08f;    // rad/s
constexpr uint32_t MANUAL_DETECT_MS = 30;
constexpr uint32_t MANUAL_RELEASE_MS = 120;

// 开启“手动转任一电机，另一台按比例跟随”。
constexpr bool ENABLE_MANUAL_INPUT = true;

// 实物标定结果：两台电机的 R 标与 C 板 R 标对齐时均为 -55°。
constexpr float A_R_ZERO_RAD = -0.9599311f;
constexpr float B_R_ZERO_RAD = -0.9599311f;

enum class ControlMode
{
    DISABLED,
    LINKAGE,
    RESET
};

enum class LinkSource
{
    BOARD,
    MOTOR_A,
    MOTOR_B
};

struct InputSnapshot
{
    bool remote_online;
    bool a_online;
    bool b_online;
    sp::DBusSwitchMode sw_r;
    sp::DBusSwitchMode sw_l;
    float a_angle;
    float a_speed;
    float b_angle;
    float b_speed;
};

float clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

float ratioFromSwitch(sp::DBusSwitchMode sw)
{
    switch (sw) {
        case sp::DBusSwitchMode::DOWN:
            return 0.5f;
        case sp::DBusSwitchMode::MID:
            return -1.0f;
        case sp::DBusSwitchMode::UP:
            return 3.0f;
    }
    return 1.0f;
}

float nearestEquivalent(float current, float zero)
{
    return zero + std::round((current - zero) / TWO_PI) * TWO_PI;
}

InputSnapshot readInputs(uint32_t now_ms)
{
    InputSnapshot input{};

    // 遥控器由 UART 中断更新，电机由 CAN 中断更新；在短临界区内
    // 复制出同一时刻的快照，后续控制只使用本地快照。
    taskENTER_CRITICAL();
    input.remote_online = remote.is_alive(now_ms);
    input.a_online = motor_a.is_alive(now_ms);
    input.b_online = motor_b.is_alive(now_ms);
    input.sw_r = remote.sw_r;
    input.sw_l = remote.sw_l;
    input.a_angle = A_SIGN * motor_a.angle;
    input.a_speed = A_SIGN * motor_a.speed;
    input.b_angle = B_SIGN * motor_b.angle;
    input.b_speed = B_SIGN * motor_b.speed;
    taskEXIT_CRITICAL();

    return input;
}
} // namespace

extern "C" void motor_task(void const * argument)
{
    (void)argument;

    motor_can.config();
    motor_can.start();
    osDelay(100);

    bool armed = false;
    bool yaw_inited = false;
    float last_yaw_raw = 0.0f;
    float yaw_continuous = 0.0f;

    ControlMode mode = ControlMode::DISABLED;
    LinkSource source = LinkSource::BOARD;
    LinkSource manual_candidate = LinkSource::BOARD;
    uint32_t manual_candidate_since_ms = 0;
    uint32_t manual_still_since_ms = 0;

    float ratio_b = 1.0f;
    sp::DBusSwitchMode last_sw_l = sp::DBusSwitchMode::MID;

    // LINKAGE 模式使用的锚点和虚拟 A 角度。
    float yaw_anchor = 0.0f;
    float a_anchor = 0.0f;
    float b_anchor = 0.0f;
    float virtual_a = 0.0f;

    float a_target = 0.0f;
    float b_target = 0.0f;
    float a_reset_target = 0.0f;
    float b_reset_target = 0.0f;

    while (true) {
        const uint32_t now_ms = osKernelSysTick();
        const InputSnapshot input = readInputs(now_ms);

        const uint32_t yaw_stamp_ms = imu_yaw_stamp_ms.load();
        const float yaw_raw = YAW_SIGN * imu_yaw_rad.load();
        const float yaw_rate = YAW_SIGN * imu_yaw_rate_rad_s.load();
        const bool imu_online =
            yaw_stamp_ms != 0 && (now_ms - yaw_stamp_ms) < 50;

        bool yaw_valid = imu_online && std::isfinite(yaw_raw) &&
                         std::isfinite(yaw_rate);

        if (yaw_valid) {
            if (!yaw_inited) {
                last_yaw_raw = yaw_raw;
                yaw_continuous = yaw_raw;
                yaw_inited = true;
            }
            else {
                const float yaw_step =
                    std::remainder(yaw_raw - last_yaw_raw, TWO_PI);
                last_yaw_raw = yaw_raw;

                if (!std::isfinite(yaw_step) ||
                    std::fabs(yaw_step) > MAX_YAW_STEP) {
                    yaw_valid = false;
                    yaw_inited = false;
                }
                else {
                    yaw_continuous += yaw_step;
                }
            }
        }
        else {
            yaw_inited = false;
        }

        const bool finite_motor_data =
            std::isfinite(input.a_angle) &&
            std::isfinite(input.a_speed) &&
            std::isfinite(input.b_angle) &&
            std::isfinite(input.b_speed);

        const bool all_online =
            input.remote_online && input.a_online && input.b_online &&
            yaw_valid && finite_motor_data;

        ControlMode requested_mode = ControlMode::DISABLED;

        if (!all_online) {
            armed = false;
        }
        else if (input.sw_r == sp::DBusSwitchMode::DOWN) {
            // 必须先经过右下档，才允许进入任何有力模式。
            armed = true;
        }
        else if (armed && input.sw_r == sp::DBusSwitchMode::MID) {
            requested_mode = ControlMode::LINKAGE;
        }
        else if (armed && input.sw_r == sp::DBusSwitchMode::UP) {
            requested_mode = ControlMode::RESET;
        }

        if (requested_mode != mode) {
            mode = requested_mode;
            source = LinkSource::BOARD;
            manual_candidate = LinkSource::BOARD;
            manual_candidate_since_ms = 0;
            manual_still_since_ms = 0;

            if (mode == ControlMode::LINKAGE) {
                yaw_anchor = yaw_continuous;
                a_anchor = input.a_angle;
                b_anchor = input.b_angle;
                virtual_a = input.a_angle;
                a_target = input.a_angle;
                b_target = input.b_angle;
                ratio_b = ratioFromSwitch(input.sw_l);
                last_sw_l = input.sw_l;
            }
            else if (mode == ControlMode::RESET) {
                a_reset_target =
                    nearestEquivalent(input.a_angle, A_R_ZERO_RAD);
                b_reset_target =
                    nearestEquivalent(input.b_angle, B_R_ZERO_RAD);
                a_target = a_reset_target;
                b_target = b_reset_target;
            }
            else {
                a_target = input.a_angle;
                b_target = input.b_angle;
            }
        }

        // 每轮先默认零力矩，只有有效模式会覆盖。
        float torque_a = 0.0f;
        float torque_b = 0.0f;

        if (mode == ControlMode::LINKAGE && all_online) {
            // 左拨杆比例变化时从当前位置重新锚定，避免 B 目标跳变。
            if (input.sw_l != last_sw_l) {
                last_sw_l = input.sw_l;
                ratio_b = ratioFromSwitch(input.sw_l);
                yaw_anchor = yaw_continuous;
                a_anchor = input.a_angle;
                b_anchor = input.b_angle;
                virtual_a = input.a_angle;
                source = LinkSource::BOARD;
                manual_candidate = LinkSource::BOARD;
                manual_candidate_since_ms = 0;
                manual_still_since_ms = 0;
            }

            if (source == LinkSource::BOARD) {
                virtual_a =
                    a_anchor + (yaw_continuous - yaw_anchor);

                const float candidate_a_target = virtual_a;
                const float candidate_b_target =
                    b_anchor + ratio_b * (virtual_a - a_anchor);
                const float error_a = candidate_a_target - input.a_angle;
                const float error_b = candidate_b_target - input.b_angle;

                // 正常追赶目标时 error*speed 为正；人将电机推离目标时为负。
                const bool board_is_still =
                    std::fabs(yaw_rate) < BOARD_RATE_THRESHOLD;
                const bool a_moved_away =
                    board_is_still &&
                    std::fabs(error_a) > MANUAL_ANGLE_THRESHOLD &&
                    std::fabs(input.a_speed) > MANUAL_SPEED_THRESHOLD &&
                    error_a * input.a_speed < 0.0f;
                const bool b_moved_away =
                    board_is_still &&
                    std::fabs(error_b) > MANUAL_ANGLE_THRESHOLD &&
                    std::fabs(input.b_speed) > MANUAL_SPEED_THRESHOLD &&
                    error_b * input.b_speed < 0.0f;

                LinkSource detected_source = LinkSource::BOARD;

                if (ENABLE_MANUAL_INPUT &&
                    (a_moved_away || b_moved_away)) {
                    if (a_moved_away && b_moved_away) {
                        detected_source =
                            std::fabs(error_a * input.a_speed) >=
                                    std::fabs(error_b * input.b_speed)
                                ? LinkSource::MOTOR_A
                                : LinkSource::MOTOR_B;
                    }
                    else {
                        detected_source = a_moved_away
                            ? LinkSource::MOTOR_A
                            : LinkSource::MOTOR_B;
                    }
                }

                // 必须连续判定为同一个手动输入端一段时间后才切换，
                // 防止正常跟随时的瞬时误差或超调被误判为手动拖动。
                if (detected_source == LinkSource::BOARD) {
                    manual_candidate = LinkSource::BOARD;
                    manual_candidate_since_ms = 0;
                }
                else if (detected_source != manual_candidate) {
                    manual_candidate = detected_source;
                    manual_candidate_since_ms = now_ms;
                }
                else if (manual_candidate_since_ms != 0 &&
                         now_ms - manual_candidate_since_ms >=
                             MANUAL_DETECT_MS) {
                    source = manual_candidate;
                    manual_candidate = LinkSource::BOARD;
                    manual_candidate_since_ms = 0;
                    manual_still_since_ms = 0;
                }
            }

            if (source == LinkSource::MOTOR_A) {
                // A 作为手动输入端；A 自身不施加力矩，B 按比例跟随。
                virtual_a = input.a_angle;
                yaw_anchor =
                    yaw_continuous - (virtual_a - a_anchor);

                if (std::fabs(input.a_speed) < MANUAL_RELEASE_SPEED) {
                    if (manual_still_since_ms == 0)
                        manual_still_since_ms = now_ms;
                    else if (now_ms - manual_still_since_ms >=
                             MANUAL_RELEASE_MS) {
                        source = LinkSource::BOARD;
                        manual_still_since_ms = 0;
                    }
                }
                else {
                    manual_still_since_ms = 0;
                }
            }
            else if (source == LinkSource::MOTOR_B) {
                // B 的变化量除以比例，换算为等效 A 位置。
                virtual_a =
                    a_anchor + (input.b_angle - b_anchor) / ratio_b;
                yaw_anchor =
                    yaw_continuous - (virtual_a - a_anchor);

                if (std::fabs(input.b_speed) < MANUAL_RELEASE_SPEED) {
                    if (manual_still_since_ms == 0)
                        manual_still_since_ms = now_ms;
                    else if (now_ms - manual_still_since_ms >=
                             MANUAL_RELEASE_MS) {
                        source = LinkSource::BOARD;
                        manual_still_since_ms = 0;
                    }
                }
                else {
                    manual_still_since_ms = 0;
                }
            }

            a_target = virtual_a;
            b_target =
                b_anchor + ratio_b * (virtual_a - a_anchor);

            torque_a = clamp(
                KP_A * (a_target - input.a_angle) - KD_A * input.a_speed,
                MAX_LINK_TORQUE);
            torque_b = clamp(
                KP_B * (b_target - input.b_angle) - KD_B * input.b_speed,
                MAX_LINK_TORQUE);

            // 被手动转动的输入电机保持无力，另一台负责跟随。
            if (source == LinkSource::MOTOR_A) torque_a = 0.0f;
            if (source == LinkSource::MOTOR_B) torque_b = 0.0f;
        }
        else if (mode == ControlMode::RESET && all_online) {
            a_target = a_reset_target;
            b_target = b_reset_target;

            torque_a = clamp(
                KP_A * (a_target - input.a_angle) - KD_A * input.a_speed,
                MAX_RESET_TORQUE);
            torque_b = clamp(
                KP_B * (b_target - input.b_angle) - KD_B * input.b_speed,
                MAX_RESET_TORQUE);
        }
        else {
            a_target = input.a_angle;
            b_target = input.b_angle;
        }

        // SerialPlot 显示值转换回电机物理角度。
        a_target_rad.store(A_SIGN * a_target);
        b_target_rad.store(B_SIGN * b_target);

        // 将逻辑力矩方向转换回实际电机方向。
        motor_a.cmd(A_SIGN * torque_a);
        motor_b.cmd(B_SIGN * torque_b);

        std::memset(motor_can.tx_data, 0, sizeof(motor_can.tx_data));
        motor_a.write(motor_can.tx_data);
        motor_b.write(motor_can.tx_data);

        // ID 1～4 的 GM6020 电流控制帧为 0x1FE。
        motor_can.send(motor_a.tx_id);

        osDelay(1);
    }
}

// CAN 收到反馈时更新对应电机的数据。
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef * hcan)
{
    if (hcan != &hcan1) return;

    const uint32_t now_ms = osKernelSysTick();

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        motor_can.recv(CAN_RX_FIFO0);

        if (motor_can.rx_id == motor_a.rx_id) {
            motor_a.read(motor_can.rx_data, now_ms);
        }
        else if (motor_can.rx_id == motor_b.rx_id) {
            motor_b.read(motor_can.rx_data, now_ms);
        }
    }
}
