# Skiing-Robot-STM32-Firmware

#### 开发环境  
Windows + Keil MDK v5.29 + STM32F4标准外设固件库

#### 文件结构
> CMSIS/: STM32固件库的内容  
> Project/: 存放Keil的工程文件，输出文件等  
> StdPeriph/: STM32F4固件库  
> User/: 用户编写的代码  
>> app/: 机器人控制相关的函数，通过定时器中断和线中断定时运行  
>>> INS_Task.c/h: 陀螺仪加速度计任务，通过线中断以1khz运行，SPI读取MPU6500数据并进行滤波、解算，移植自大疆  
>>> Control_Task.c/h: 滑雪机器人姿态控制任务，通过定时器以200Hz运行，包含平衡闭环控制和方向角速度控制  

>> driver/: 开发板所连接各个外设的驱动  
>>> AHRS/: 陀螺仪和磁力计的驱动和姿态解算链接库，移植自大疆  
>>> buzzer.c/h: 蜂鸣器  
>>> can.c/h: CAN总线收发  
>>> dbus_rc.c/h: 遥控器串口接收  
>>> delay.c/h: 毫秒及微秒延时函数，通过Systick实现  
>>> exti_init.c/h: 初始化线中断，用于当MPU6500数据准备好时触发中断，改变标志变量  
>>> led.c/h: LED指示灯  
>>> oled.c/h/oledfont.h: OLED 128×32显示屏用于调试  
>>> pwm.c/h: 定时器，包括PWM舵机输出和控制任务使用的定时器中断初始化  
>>> spi.c/h: SPI总线，用于读取MPU6500陀螺仪   

>> function/: 一些功能  
>>> pid.c/h: PID计算函数实现  
