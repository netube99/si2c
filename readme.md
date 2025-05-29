# si2c

一个灵活的软件I2C总线库函数，支持8/16位寄存器，支持一些非标准读写时序的I2C设备操作

原作者：https://schkorea.tistory.com/437
Forked from：https://github.com/liyanboy74/soft-i2c

参数 `pdata/regaddr/len` 可为`NULL`或`0`，以实现一些特殊的操作

使用例程：

```C
//引用头文件
//根据项目硬件具体情况，修改si2c_port.h内定义的宏
#include "si2c.h"
#include "si2c_port.h"

//创建软件I2C操作句柄
//默认已在si2c_port.c中创建，可在其他位置定义
si2c_t si2c;

//初始化软件I2C接口
Si2c_Init(&si2c, Si2c_Port_Ioctl, Si2c_Port_Delay_Us, Si2c_Critical);

// 初始化传感器
uint8_t Bsp_Opt4001_Init(void)
{
    uint8_t chip_id[2] = {0};
    //复位全局I2C芯片
    Si2c_General_Call_Reset(&si2c);
    HAL_Delay(20);
    //读取并检查芯片ID
    if(Si2c_Read(&si2c, OPT4001_I2C_ADDR, OPT4001_DEVICE_ID_REG, chip_id, 2, USE_REG_ADDR, REG8) != SWI2C_TRUE)
        return OPT4001_ERROR_CHIPID;
    if((chip_id[0] != 0x01) || (chip_id[1] != 0x21))
        return OPT4001_ERROR_CHIPID;
    uint8_t opt4001_config[2] = {0x32, 0xF0};
    //写入传感器配置
    if(Si2c_Write(&si2c, OPT4001_I2C_ADDR, OPT4001_CONFIG_REG, opt4001_config, 2, USE_REG_ADDR, REG8) != SWI2C_TRUE)
        return OPT4001_ERROR_WRITE;
    HAL_Delay(20);
    return OPT4001_SUCCESS;
}
```