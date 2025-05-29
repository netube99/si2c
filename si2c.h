#ifndef _SI2C_H_
#define _SI2C_H_

#include <stdint.h>
#include <stdbool.h>

#define I2C_READ            0x01    // 读指令bit定义
#define SI2C_WAIT_TIME      1       // 软件简易延时时间
#define READ_CMD            1       // 读
#define WRITE_CMD           0       // 写
#define SWI2C_TRUE          1       // I2C操作成功
#define SWI2C_FALSE         0       // I2C操作失败

#define USE_REG_ADDR        0       // 使用寄存器地址
#define WHITOUT_REG_ADDR    1       // 不使用寄存器地址
#define REG8                0       // 使用八位寄存器地址
#define REG16               1       // 使用十六位寄存器地址

//引脚操作枚举
typedef enum
{
    HAL_IO_OPT_SET_SDA_LOW = 0,
    HAL_IO_OPT_SET_SDA_HIGH,
    HAL_IO_OPT_SET_SCL_LOW,
    HAL_IO_OPT_SET_SCL_HIGH,
    HAL_IO_OPT_SET_SDA_INPUT,
    HAL_IO_OPT_SET_SDA_OUTPUT,
    HAL_IO_OPT_SET_SCL_INPUT,
    HAL_IO_OPT_SET_SCL_OUTPUT,
    HAL_IO_OPT_GET_SDA_LEVEL,
    HAL_IO_OPT_GET_SCL_LEVEL,
} hal_io_opt_e;

//I2C接口句柄
typedef struct si2c_s
{
    int (*hal_io_ctl)(hal_io_opt_e opt);
    void (*hal_delay_us)(uint32_t us);
    void (*critical)(uint8_t on);
} si2c_t;

int Si2c_Init(si2c_t *si2c_handle, \
            int (*io_ctl)(hal_io_opt_e opt),\
            void (*delay_us)(uint32_t us), \
            void (*irq_ctl)(uint8_t on));
uint8_t Si2c_Read(si2c_t *si2c_handle, uint8_t i2c_id, uint16_t regaddr, \
                uint8_t *pdata, uint8_t len, bool use_reg, bool reg_size);
uint8_t Si2c_Write(si2c_t *si2c_handle, uint8_t i2c_id, uint16_t regaddr, \
                uint8_t *pdata, uint8_t len, bool use_reg, bool reg_size);
uint8_t Si2c_General_Call_Reset(si2c_t *si2c_handle);
uint8_t Si2c_Check_Slave_Addr(si2c_t *si2c_handle, uint8_t i2c_id);

#endif