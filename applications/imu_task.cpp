#include <atomic>

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

// BMI088 坐标系转换到 C 板使用的坐标系
const float r_ab[3][3] = {
    {0.0f, -1.0f, 0.0f},
    {1.0f,  0.0f, 0.0f},
    {0.0f,  0.0f, 1.0f}
};

// SPI1 读取 BMI088
// PA4：加速度计片选
// PB0：陀螺仪片选
sp::BMI088 bmi088(
    &hspi1,
    GPIOA, GPIO_PIN_4,
    GPIOB, GPIO_PIN_0,
    r_ab
);

// 每 1 ms 更新一次姿态，因此 dt = 0.001 秒
sp::Mahony imu(0.001f);

// 发布给 motor_task.cpp 的 yaw 快照，单位 rad
std::atomic<float> imu_yaw_rad{0.0f};

// 使用 USART1 发送数据
// false 表示暂时不使用 DMA，方便调试
sp::Plotter plotter(&huart1, false);

// remote 实际创建在 uart_task.cpp 中
extern sp::DBus remote;

// 两台电机实际创建在 motor_task.cpp
extern sp::RM_Motor motor_a;
extern sp::RM_Motor motor_b;

// A 的目标角度实际创建在 motor_task.cpp
extern std::atomic<float> a_target_rad;

extern "C" void imu_task(void const * argument)
{
    (void)argument;

    // 初始化加速度计和陀螺仪
    bmi088.init();

    uint32_t print_count = 0;

    while (true) {
        // 读取加速度、角速度和温度
        bmi088.update();

        // 根据加速度和角速度计算姿态
        imu.update(bmi088.acc, bmi088.gyro);

        // 将最新 yaw 提供给电机任务
        imu_yaw_rad.store(imu.yaw);

        // 读取周期约为 1 ms，但每 20 ms 才发送一次
        // 避免串口发送过于频繁
        print_count++;

        if (print_count >= 20) {
            print_count = 0;

            constexpr float RAD_TO_DEG = 57.29578f;
            const uint32_t now_ms = osKernelSysTick();
            const bool remote_online = remote.is_alive(now_ms);

            plotter.plot(
                // 通道 1～3：加速度，单位 m/s²
                bmi088.acc[0],
                bmi088.acc[1],
                bmi088.acc[2],

                // 通道 4～6：角速度，单位 rad/s
                bmi088.gyro[0],
                bmi088.gyro[1],
                bmi088.gyro[2],

                // 通道 7～9：姿态角，单位°
                imu.roll  * RAD_TO_DEG,
                imu.pitch * RAD_TO_DEG,
                imu.yaw   * RAD_TO_DEG,

                // 通道 10～13：遥控器状态和 A 目标角度
                remote_online ? 1.0f : 0.0f,  // 10：遥控器在线
                remote_online
                    ? static_cast<float>(remote.sw_r)
                    : -1.0f,                  // 11：右拨杆
                remote_online
                    ? static_cast<float>(remote.sw_l)
                    : -1.0f,                  // 12：左拨杆
                a_target_rad.load() * RAD_TO_DEG, // 13：A 目标角度

                // 通道 14～17：两台电机在线状态和实际角度
                motor_a.is_alive(now_ms) ? 1.0f : 0.0f, // 14：A 在线
                motor_a.angle * RAD_TO_DEG,              // 15：A 实际角度
                motor_b.is_alive(now_ms) ? 1.0f : 0.0f, // 16：B 在线
                motor_b.angle * RAD_TO_DEG               // 17：B 实际角度
            );
        }

        osDelay(1);
    }
}