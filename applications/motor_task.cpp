#include <atomic>
#include <cmath>
#include <cstring>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

// 使用 C 板 CAN1
sp::CAN motor_can(&hcan1);

// 已确认：A 电机 ID 为 1，B 电机 ID 为 2
// 两台都使用电流控制模式
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);

// remote 在 uart_task.cpp 中创建
extern sp::DBus remote;

// imu_task.cpp 发布的 C 板 yaw，单位 rad
extern std::atomic<float> imu_yaw_rad;

// 给 imu_task.cpp/SerialPlot 显示的 A 目标角度，单位 rad
std::atomic<float> a_target_rad{0.0f};

extern "C" void motor_task(void const * argument)
{
    (void)argument;

    // 配置过滤器并启动 CAN 接收
    motor_can.config();
    motor_can.start();

    // 等待 CAN 和电机上电稳定
    osDelay(100);

    bool mid_active = false;
    float yaw_ref = 0.0f;
    float a_ref = 0.0f;

    while (true) {
        const uint32_t now_ms = osKernelSysTick();
        const bool remote_online = remote.is_alive(now_ms);
        const bool motors_online =
            motor_a.is_alive(now_ms) && motor_b.is_alive(now_ms);

        // 本阶段只预演目标角度：两台电机始终零指令
        motor_a.cmd(0.0f);
        motor_b.cmd(0.0f);

        if (remote_online &&
            motors_online &&
            remote.sw_r == sp::DBusSwitchMode::MID) {

            const float yaw_now = imu_yaw_rad.load();

            if (!mid_active) {
                // 刚进入中档：记住此刻 C 板和 A 电机的位置
                yaw_ref = yaw_now;
                a_ref = motor_a.angle;
                mid_active = true;
            }

            // 处理 yaw 穿过 ±180° 时的跳变
            // 当前只用于小角度预演，不支持连续转动超过半圈
            const float yaw_delta =
                std::remainder(yaw_now - yaw_ref, 6.2831853f);

            // 1:1 联动的目标角度；这里只计算，不驱动电机
            a_target_rad.store(a_ref + yaw_delta);
        } else {
            // 下档、上档或失联时，下次进入中档重新取参考零点
            mid_active = false;
            a_target_rad.store(motor_a.angle);
        }

        // 先把整帧 8 字节清零
        std::memset(motor_can.tx_data, 0, sizeof(motor_can.tx_data));

        // 把 A、B 电机的零指令放入同一帧
        motor_a.write(motor_can.tx_data);
        motor_b.write(motor_can.tx_data);

        // ID 1～4 的 GM6020 电流控制共用 0x1FE 帧
        motor_can.send(motor_a.tx_id);

        osDelay(1);
    }
}

// CAN 收到电机反馈时，HAL 会进入这个函数
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef * hcan)
{
    if (hcan != &hcan1) {
        return;
    }

    const uint32_t now_ms = osKernelSysTick();

    // 把 FIFO 中当前已经收到的消息全部取出
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