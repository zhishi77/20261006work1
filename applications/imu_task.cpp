#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"

// BMI088坐标系转换到C板使用的坐标系
const float r_ab[3][3] = {
    {0.0f, -1.0f, 0.0f},
    {1.0f,  0.0f, 0.0f},
    {0.0f,  0.0f, 1.0f}
};

// SPI1读取BMI088
// PA4：加速度计片选
// PB0：陀螺仪片选
sp::BMI088 bmi088(
    &hspi1,
    GPIOA, GPIO_PIN_4,
    GPIOB, GPIO_PIN_0,
    r_ab
);

// 每1 ms更新一次姿态，因此dt = 0.001秒
sp::Mahony imu(0.001f);

// 使用USART1发送数据
// false表示暂时不使用DMA，方便第一次调试
sp::Plotter plotter(&huart1, false);

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

        // 读取周期约为1 ms，但每20 ms才发送一次
        // 避免串口发送过于频繁
        print_count++;

        if (print_count >= 20) {
            print_count = 0;

            constexpr float RAD_TO_DEG = 57.29578f;

            plotter.plot(
                // 通道1～3：加速度，单位m/s²
                bmi088.acc[0],
                bmi088.acc[1],
                bmi088.acc[2],

                // 通道4～6：角速度，单位rad/s
                bmi088.gyro[0],
                bmi088.gyro[1],
                bmi088.gyro[2],

                // 通道7～9：姿态角，转换成度
                imu.roll  * RAD_TO_DEG,
                imu.pitch * RAD_TO_DEG,
                imu.yaw   * RAD_TO_DEG
            );
        }

        osDelay(1);
    }
}