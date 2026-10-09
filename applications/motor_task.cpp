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

//增加短脉冲计数实验
volatile uint32_t pulse_trigger_count = 0;

extern "C" void motor_task(void const * argument)
{
    (void)argument;

    // 配置过滤器并启动CAN接收
    motor_can.config();
    motor_can.start();

    // 等待CAN和电机上电稳定
    osDelay(100);

    bool armed = false;         // 是否已在下档准备好一次测试
    bool pulse_active = false;  // A 电机的短脉冲是否正在进行
    uint32_t pulse_start_ms = 0;

    while (true) {
        // 当前安全测试阶段：两台电机都不主动出力
       //motor_a.cmd(0.0f);
        //motor_b.cmd(0.0f);

        const uint32_t now_ms = osKernelSysTick();
const bool remote_online = remote.is_alive(now_ms);
const bool motors_online =
    motor_a.is_alive(now_ms) && motor_b.is_alive(now_ms);

// 每一轮先默认两台电机都为零输出
motor_a.cmd(0.0f);
motor_b.cmd(0.0f);

if (!remote_online || !motors_online) {
    // 遥控器或任一电机掉线：取消测试，必须重新从下档开始
    armed = false;
    pulse_active = false;
}
else if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
    // 看到下档，才允许下一次“下档→中档”触发测试
    armed = true;
    pulse_active = false;
}
else if (remote.sw_r == sp::DBusSwitchMode::MID) {
    if (armed) {
        armed = false;  // 只触发一次，不因任务循环而反复触发
        pulse_active = true;
        pulse_start_ms = now_ms;
        ++pulse_trigger_count;
    }

    if (pulse_active && (now_ms - pulse_start_ms < 100)) {
        motor_a.cmd(0.05f);  // 仅 A：很小的转矩指令，持续最多 100 ms
    } else {
        pulse_active = false;
    }
}
else {
    // 上档暂不做复位
    armed = false;
    pulse_active = false;
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