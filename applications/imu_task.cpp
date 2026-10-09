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

// 发布给 motor_task.cpp 的 yaw 快照和更新时间
std::atomic<float> imu_yaw_rad{0.0f};
std::atomic<float> imu_yaw_rate_rad_s{0.0f};
std::atomic<uint32_t> imu_yaw_stamp_ms{0};

// 使用 USART1 发送数据；暂时不使用 DMA
sp::Plotter plotter(&huart1, false);

// remote 实际创建在 uart_task.cpp 中
extern sp::DBus remote;

// 两台电机实际创建在 motor_task.cpp
extern sp::RM_Motor motor_a;
extern sp::RM_Motor motor_b;

// 目标角度实际创建在 motor_task.cpp
extern std::atomic<float> a_target_rad;
extern std::atomic<float> b_target_rad;

extern "C" void imu_task(void const * argument)
{
    (void)argument;

    bmi088.init();

    uint32_t print_count = 0;

    while (true) {
        bmi088.update();
        imu.update(bmi088.acc, bmi088.gyro);

        imu_yaw_rad.store(imu.yaw);
        imu_yaw_rate_rad_s.store(imu.vyaw);
        imu_yaw_stamp_ms.store(osKernelSysTick());

        // IMU 约每 1 ms 更新；串口约每 20 ms 发送一次
        print_count++;

        if (print_count >= 20) {
            print_count = 0;

            constexpr float RAD_TO_DEG = 57.29578f;
            const uint32_t now_ms = osKernelSysTick();
            const bool remote_online = remote.is_alive(now_ms);

            plotter.plot(
                // 加速度：m/s²
                bmi088.acc[0],
                bmi088.acc[1],
                bmi088.acc[2],

                // 角速度：rad/s
                bmi088.gyro[0],
                bmi088.gyro[1],
                bmi088.gyro[2],

                // C 板姿态角：°
                imu.roll  * RAD_TO_DEG,
                imu.pitch * RAD_TO_DEG,
                imu.yaw   * RAD_TO_DEG,

                // 遥控器及目标角度
                remote_online ? 1.0f : 0.0f,
                remote_online
                    ? static_cast<float>(remote.sw_r)
                    : -1.0f,
                remote_online
                    ? static_cast<float>(remote.sw_l)
                    : -1.0f,
                a_target_rad.load() * RAD_TO_DEG,
                b_target_rad.load() * RAD_TO_DEG,

                // 两台电机的在线状态及实际角度
                motor_a.is_alive(now_ms) ? 1.0f : 0.0f,
                motor_a.angle * RAD_TO_DEG,
                motor_b.is_alive(now_ms) ? 1.0f : 0.0f,
                motor_b.angle * RAD_TO_DEG
            );
        }

        osDelay(1);
    }
}
