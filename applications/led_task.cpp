#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task(void const * argument)
{
    (void)argument;

    // 启动TIM5的三个PWM通道
    led.start();

    while (true) {
        // 红色逐渐变成绿色
        for (int i = 0; i <= 50; i++) {
            float value = i / 50.0f;
            led.set(1.0f - value, value, 0.0f);
            osDelay(20);
        }

        // 绿色逐渐变成蓝色
        for (int i = 0; i <= 50; i++) {
            float value = i / 50.0f;
            led.set(0.0f, 1.0f - value, value);
            osDelay(20);
        }

        // 蓝色逐渐变成红色
        for (int i = 0; i <= 50; i++) {
            float value = i / 50.0f;
            led.set(value, 0.0f, 1.0f - value);
            osDelay(20);
        }
    }
}