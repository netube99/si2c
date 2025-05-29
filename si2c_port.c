#include "si2c_port.h"

//接口操作句柄
si2c_t si2c;

void Si2c_Port_Delay_Us(uint32_t us)
{
    if(us == 0) return;
    volatile uint32_t cycles = (SI2C_CORE_FREQ * us)/(1000000 * CYCLES_PER_LOOP);
    if(cycles == 0) cycles = 1;
    do {__asm__ volatile("nop");} while(--cycles);// nop循环
}

//中断使能控制函数，用于临界区保护操作
void Si2c_Critical(uint8_t on)
{
    if(on) SI2C_DISABLE_IRQ();
    else SI2C_ENABLE_IRQ();
}

//I2C引脚控制函数
int Si2c_Port_Ioctl(uint8_t opt)
{
    int ret = -1;
    switch (opt)
    {
    case HAL_IO_OPT_SET_SDA_HIGH:
        SI2C_SET_SDA_HIGH();
        break;
    case HAL_IO_OPT_SET_SDA_LOW:
        SI2C_SET_SDA_LOW();
        break;
    case HAL_IO_OPT_GET_SDA_LEVEL:
        ret = SI2C_GET_SDA();
        break;
    case HAL_IO_OPT_SET_SDA_INPUT:
        SI2C_SET_SDA_INPUT();
        break;
    case HAL_IO_OPT_SET_SDA_OUTPUT:
        SI2C_SET_SDA_OUTPUT();
        break;
    case HAL_IO_OPT_SET_SCL_HIGH:
        SI2C_SET_SCL_HIGH();
        break;
    case HAL_IO_OPT_SET_SCL_LOW:
        SI2C_SET_SCL_LOW();
        break;
    case HAL_IO_OPT_SET_SCL_OUTPUT:
        SI2C_SET_SCL_OUTPUT();
        break;
    default:
        break;
    }
    return ret;
}