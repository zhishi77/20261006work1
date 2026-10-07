#include <cstring>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "io/dbus/dbus.hpp"

// 使用C板CAN1
sp::CAN motor_can(&hcan1);

// 假设A电机ID为1，B电机ID为2
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);

// remote 在 uart_task.cpp 中创建，这里读取它
extern sp::DBus remote;

extern "C" void motor_task(void const * argument)
{
    (void)argument;

    // 配置过滤器并启动CAN接收
    motor_can.config();
    motor_can.start();

    // 等待CAN和电机上电稳定
    osDelay(100);

    while (true) {
        // 当前安全测试阶段：两台电机都不主动出力
       //motor_a.cmd(0.0f);
        //motor_b.cmd(0.0f);

        const bool remote_online = remote.is_alive(osKernelSysTick());

if (!remote_online || remote.sw_r == sp::DBusSwitchMode::DOWN) {
    // 遥控器失联，或右拨杆下档：必须零输出
    motor_a.cmd(0.0f);
    motor_b.cmd(0.0f);
}
else if (remote.sw_r == sp::DBusSwitchMode::MID) {
    // 中档：将来写姿态联动；现在仍保持零输出
    motor_a.cmd(0.0f);
    motor_b.cmd(0.0f);
}
else {
    // 上档：将来写 R 标复位；现在仍保持零输出
    motor_a.cmd(0.0f);
    motor_b.cmd(0.0f);
}

        // 先把整帧8字节清零
        std::memset(motor_can.tx_data, 0, sizeof(motor_can.tx_data));

        // 把A、B电机的命令放入同一帧
        motor_a.write(motor_can.tx_data);
        motor_b.write(motor_can.tx_data);

        // ID 1～4的GM6020共用0x1FE控制帧
        motor_can.send(motor_a.tx_id);

        osDelay(1);
    }
}

// CAN收到电机反馈时，HAL会进入这个函数
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef * hcan)
{
    if (hcan != &hcan1) {
        return;
    }

    uint32_t now_ms = osKernelSysTick();

    // 把FIFO中当前已经收到的消息全部取出
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