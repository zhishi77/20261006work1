#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

// 使用TIM4第三通道。
// 84e6表示TIM4的输入时钟是84 MHz。
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6f);

extern "C" void buzzer_task(void const * argument)
{
    // 当前任务不使用传入参数
    (void)argument;

    // 等待系统上电和初始化稳定
    osDelay(300);

    // Do、Mi、Sol三个音符
    const float notes[] = {
        1046.5f,
        1318.5f,
        1568.0f
    };

    // 2%占空比，先避免声音过大
    const float duty = 0.02f;

    for (int i = 0; i < 3; i++) {
        // 设置音调和占空比
        buzzer.set(notes[i], duty);

        // 开始输出PWM，蜂鸣器发声
        buzzer.start();

        // 当前音符持续180毫秒
        osDelay(180);

        // 停止PWM
        buzzer.stop();

        // 音符之间停顿60毫秒
        osDelay(60);
    }

    // 提示音只播放一次
    while (true) {
        osDelay(1000);
    }
}