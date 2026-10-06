#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// C板的DR16接收机使用USART3和DMA
sp::DBus remote(&huart3);

extern "C" void uart_task(void const * argument)
{
    (void)argument;

    // 开始第一次接收
    remote.request();

    while (true) {
        // 数据接收和解析在下面的回调函数中完成
        osDelay(10);
    }
}

// USART接收到一帧数据后，HAL会调用这个函数
extern "C" void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef * huart,
    uint16_t size)
{
    if (huart == &huart3) {
        // 把18字节数据解析成摇杆、拨杆数值
        remote.update(size, osKernelSysTick());

        // 开始等待下一帧数据
        remote.request();
    }
}

// 如果串口接收出错，重新启动接收
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if (huart == &huart3) {
        remote.request();
    }
}