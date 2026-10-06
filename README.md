# RoboMaster C板第一阶段考核

## 项目内容

本项目用于完成C板基础功能考核：

- 上电后蜂鸣器播放Do-Mi-Sol提示音
- RGB LED循环渐变，用作程序运行状态指示
- 读取BMI088加速度、角速度及姿态角
- 通过USART1向SerialPlot发送IMU数据
- 通过USART3和DMA接收DT7/DR16遥控器数据

## 硬件

- RoboMaster开发板C型
- DT7遥控器和DR16接收机
- CMSIS-DAP无线烧录器

## 软件与配置

- STM32CubeMX
- VS Code
- CMake和ARM GCC
- OpenOCD
- SerialPlot

## 重要配置

- 系统时钟：168 MHz
- 蜂鸣器：PD14 / TIM4_CH3
- RGB LED：PH10、PH11、PH12 / TIM5_CH1～CH3
- BMI088：SPI1，Mode 3，Prescaler 8
- 上位机串口：USART1，921600
- 遥控器：USART3，100000，8E1，DMA接收

## 编译

1. 使用STM32CubeMX打开 `.ioc` 文件并生成代码。
2. 使用VS Code打开工程根目录。
3. 选择CMake Debug预设。
4. 执行Configure和Build。

## 注意事项

- 修改CubeMX配置后需要重新生成代码。
- 修改CMakeLists.txt后需要重新执行CMake Configure。
- SerialPlot使用921600波特率和Custom Frame格式。
- 烧录及硬件功能仍需在实验室验证。