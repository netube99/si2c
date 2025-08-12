#ifndef _SI2C_PORT_H_
#define _SI2C_PORT_H_

#include "si2c.h"

/* 硬件相关头文件与宏定义，根据实际硬件修改 */
//引用芯片固件库基础头文件和GPIO控制等接口
#include "msp_gpio.h"
#include "stm32f1xx_hal.h"
// 默认参数基于STM32F103C8T6 主时钟频率64Mhz，可根据实际情况修改
// I2C CLK频率大约为 95khz ~ 105khz，可通过宏 CYCLES_PER_LOOP 微调
#define SI2C_CORE_FREQ          64000000    // 内核时钟频率
#define CYCLES_PER_LOOP         8           // 每个循环大约消耗N个时钟周期，值越小I2C时钟频率越低
// 中断控制宏
#define SI2C_DISABLE_IRQ()      __disable_irq()
#define SI2C_ENABLE_IRQ()       __enable_irq()
// GPIO操作宏
#define SI2C_SET_SDA_HIGH()     _Msp_Gpio_Set_B7()          //LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7)
#define SI2C_SET_SDA_LOW()      _Msp_Gpio_Reset_B7()        //LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7)
#define SI2C_GET_SDA()          _Msp_Gpio_Get_B7()          //LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_7)
#define SI2C_SET_SDA_INPUT()    _Msp_Gpio_InputMode_B7()    //LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT)
#define SI2C_SET_SDA_OUTPUT()   _Msp_Gpio_OutMode_B7()      //LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT)
#define SI2C_SET_SCL_HIGH()     _Msp_Gpio_Set_B6()          //LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6)
#define SI2C_SET_SCL_LOW()      _Msp_Gpio_Reset_B6()        //LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6)
#define SI2C_SET_SCL_OUTPUT()   _Msp_Gpio_OutMode_B6()      //LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT)

extern si2c_t si2c;
void Si2c_Port_Delay_Us(uint32_t us);
int Si2c_Port_Ioctl(uint8_t opt);
void Si2c_Critical(bool on);

#endif