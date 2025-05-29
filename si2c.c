#include <stdlib.h>
#include "si2c.h"

/**
 * @brief 初始化 I2C 框架状态
 *
 * @param[in,out] si2c_handle  I2C 控制句柄，用于存储初始化后的状态和函数指针
 * @param[in] io_ctl           硬件 IO 控制函数指针，用于设置 SDA/SCL 方向
 * @param[in] delay_us         微秒级延时函数指针，需配合时钟频率参数
 * @param[in] irq_ctl          临界区保护函数指针（中断开关控制）
 * @return int                 初始化结果：
 *                             - SWI2C_TRUE:  初始化成功
 *                             - SWI2C_FALSE: 初始化失败（句柄无效）
 */
int Si2c_Init(si2c_t *si2c_handle,\
            int (*io_ctl)(hal_io_opt_e opt),\
            void (*delay_us)(uint32_t us),\
            void (*irq_ctl)(uint8_t on))
{
    if(si2c_handle)
    {
        si2c_handle->hal_io_ctl = io_ctl;
        si2c_handle->hal_delay_us = delay_us;
        si2c_handle->critical = irq_ctl;
        si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_OUTPUT);
        si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_OUTPUT);
        si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
        return SWI2C_TRUE;
    }
    return SWI2C_FALSE;
}

static void Si2c_Sda_Out(si2c_t *si2c_handle, uint8_t out)
{
    if(out) si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_HIGH);
    else si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_LOW);
}

static void Si2c_Clk_Data_Out(si2c_t *si2c_handle)
{
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_HIGH);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_LOW);
}

static void Si2c_Port_Initial(si2c_t *si2c_handle)
{
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_OUTPUT);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_OUTPUT);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_HIGH);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_HIGH);
}

static uint8_t Si2c_Read_Val_SDA(si2c_t *si2c_handle)
{
    return si2c_handle->hal_io_ctl(HAL_IO_OPT_GET_SDA_LEVEL);
}

static void Si2c_Start_Condition(si2c_t *si2c_handle)
{
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_HIGH);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_HIGH);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_LOW);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_LOW);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME << 1);
}

static void Si2c_Stop_Condition(si2c_t *si2c_handle)
{
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_LOW);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_HIGH);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_HIGH);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
}

static uint8_t Si2c_Check_Ack(si2c_t *si2c_handle)
{
    uint8_t ack;
    int i;
    unsigned int temp;
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_INPUT);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_HIGH);
    ack = 0;
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    for (i = 10; i > 0; i--)
    {
        temp = !(Si2c_Read_Val_SDA(si2c_handle));
        if (temp)
        {
            ack = 1;
            break;
        }
    }
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_LOW);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_OUTPUT);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    return ack;
}

static void Si2c_Check_Not_Ack(si2c_t *si2c_handle)
{
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_INPUT);
    Si2c_Clk_Data_Out(si2c_handle);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_OUTPUT);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
}

static void Si2c_Slave_Address(si2c_t *si2c_handle, uint8_t i2c_id, uint8_t readwrite)
{
    int x;
    if (readwrite) i2c_id |= I2C_READ;
    else i2c_id &= ~I2C_READ;
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_LOW);
    for (x = 7; x >= 0; x--)
    {
        Si2c_Sda_Out(si2c_handle, i2c_id & (1 << x));
        si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
        Si2c_Clk_Data_Out(si2c_handle);
    }
}

static void Si2c_Register_Address(si2c_t *si2c_handle, uint8_t addr)
{
    int x;

    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_LOW);

    for (x = 7; x >= 0; x--)
    {
        Si2c_Sda_Out(si2c_handle, addr & (1 << x));
        si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
        Si2c_Clk_Data_Out(si2c_handle);
    }
}

static void Si2c_Send_Ack(si2c_t *si2c_handle)
{
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_OUTPUT);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_LOW);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_HIGH);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME << 1);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_LOW);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME << 1);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_LOW);
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_OUTPUT);
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
}

static void Si2c_Write_Data(si2c_t *si2c_handle, uint8_t data)
{
    int x;
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_LOW);
    for (x = 7; x >= 0; x--)
    {
        Si2c_Sda_Out(si2c_handle, data & (1 << x));
        si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
        Si2c_Clk_Data_Out(si2c_handle);
    }
}

static uint8_t Si2c_Read_Data(si2c_t *si2c_handle)
{
    int x;
    uint8_t readdata = 0;
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_INPUT);
    for (x = 8; x--;)
    {
        si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_HIGH);
        readdata <<= 1;
        if (Si2c_Read_Val_SDA(si2c_handle))
            readdata |= 0x01;
        si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
        si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SCL_LOW);
        si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    }
    si2c_handle->hal_io_ctl(HAL_IO_OPT_SET_SDA_OUTPUT);
    return readdata;
}

/**
 * @brief 通过I2C接口从设备读取数据
 *
 * @param[in] si2c_handle   I2C接口句柄，指向已初始化的I2C控制结构体
 * @param[in] i2c_id        I2C设备地址（包含读写位，bit0值为0）
 * @param[in] regaddr       要读取的寄存器地址（支持8/16位），可为NULL
 * @param[out] pdata        数据读缓冲区指针，读取的数据将存储于此，可为NULL
 * @param[in] len           需要读取的字节数，可为0
 * @param[in] use_reg       寄存器使用模式：
 *                          - USE_REG_ADDR: 需要先写入寄存器地址再读取数据
 *                          - WHITOUT_REG_ADDR: 直接读取数据（无寄存器地址阶段）
 * @param[in] reg_size      寄存器地址宽度：
 *                          - REG8:  8位寄存器地址
 *                          - REG16: 16位寄存器地址
 * @return uint8_t          执行结果：
 *                          - SWI2C_TRUE:  操作成功
 *                          - SWI2C_FALSE: 操作失败
 * @details 工作流程：
 * 1. USE_REG_ADDR模式：
 *    启动总线 → 写芯片ID → 写寄存器地址 → 重启总线 → 读芯片ID → 循环读取数据 → 结束总线
 * 2. WHITOUT_REG_ADDR模式：
 *    启动总线 → 读芯片ID → 循环读取数据 → 结束总线
 */
uint8_t Si2c_Read(si2c_t *si2c_handle, uint8_t i2c_id, uint16_t regaddr, \
                uint8_t *pdata, uint8_t len, bool use_reg, bool reg_size)
{
    uint8_t index;
    if(si2c_handle == NULL) return SWI2C_FALSE;
    si2c_handle->critical(1);//进入临界区保护
    Si2c_Port_Initial(si2c_handle);
    // 是否需要设定寄存器地址
    if(!use_reg)
    {
        Si2c_Start_Condition(si2c_handle);
        // 写 ID
        Si2c_Slave_Address(si2c_handle, i2c_id, WRITE_CMD);
        if(!Si2c_Check_Ack(si2c_handle)) goto error_exit;
        si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
        // 16位寄存器地址
        if(reg_size)
        {
            // 写高八位地址
            Si2c_Register_Address(si2c_handle, (uint8_t)(regaddr >> 8));
            if (!Si2c_Check_Ack(si2c_handle)) goto error_exit;
            si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
        }
        // 写低八位地址
        Si2c_Register_Address(si2c_handle, (uint8_t)regaddr);
        if(!Si2c_Check_Ack(si2c_handle)) goto error_exit;
        si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    }
    // 重启 I2C 总线
    Si2c_Start_Condition(si2c_handle);
    // 读 ID
    Si2c_Slave_Address(si2c_handle, i2c_id, READ_CMD);
    if(!Si2c_Check_Ack(si2c_handle)) goto error_exit;
    // 处理 len=0 的情况
    if(len == 0)
    {
        Si2c_Stop_Condition(si2c_handle);
        si2c_handle->critical(0);//退出临界区保护
        return SWI2C_TRUE;  // 无数据读取，但 ACK 成功
    }
    // 循环读数据（len > 0）
    if((pdata != NULL) && (len > 0))
    {
        for(index = 0; index < len; index++)
        {
            si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
            pdata[index] = Si2c_Read_Data(si2c_handle);
            // 最后一个字节发 NACK，其余发 ACK
            if (index < len - 1) Si2c_Send_Ack(si2c_handle);
            else Si2c_Check_Not_Ack(si2c_handle);
        }
    }
    Si2c_Stop_Condition(si2c_handle);
    si2c_handle->critical(0);//退出临界区保护
    return SWI2C_TRUE;
error_exit:
    Si2c_Stop_Condition(si2c_handle);  // 确保在错误时发送 STOP
    si2c_handle->critical(0);//退出临界区保护
    return SWI2C_FALSE;
}

/**
 * @brief 通过I2C接口向设备写入数据
 *
 * @param[in] si2c_handle   I2C接口句柄，指向已初始化的I2C控制结构体
 * @param[in] i2c_id        I2C设备地址（包含读写位，bit0值为0）
 * @param[in] regaddr       目标寄存器地址（支持8/16位），可为NULL
 * @param[in] pdata         数据写缓冲区指针，待写入的数据存储于此，可为NULL
 * @param[in] len           需要写入的字节数，可为0
 * @param[in] use_reg       寄存器使用模式：
 *                          - USE_REG_ADDR: 需要先写入寄存器地址再写入数据
 *                          - WITHOUT_REG_ADDR: 直接写入数据（无寄存器地址阶段）
 * @param[in] reg_size      寄存器地址宽度：
 *                          - REG8:  8位寄存器地址
 *                          - REG16: 16位寄存器地址
 * @return uint8_t          执行结果：
 *                          - SWI2C_TRUE:  操作成功
 *                          - SWI2C_FALSE: 操作失败
 *
 * @details 工作流程：
 * 1. USE_REG_ADDR模式：
 *    启动总线 → 写芯片ID → 写寄存器地址 → 循环写入数据 → 结束总线
 * 2. WITHOUT_REG_ADDR模式：
 *    启动总线 → 写芯片ID → 循环写入数据 → 结束总线
 */
uint8_t Si2c_Write(si2c_t *si2c_handle, uint8_t i2c_id, uint16_t regaddr,
                   uint8_t *pdata, uint8_t len, bool use_reg, bool reg_size)
{
    uint8_t index;
    si2c_handle->critical(1);//进入临界区保护
    Si2c_Port_Initial(si2c_handle);
    Si2c_Start_Condition(si2c_handle);
    // 写 ID
    Si2c_Slave_Address(si2c_handle, i2c_id, WRITE_CMD);
    if(!Si2c_Check_Ack(si2c_handle)) goto error_exit;
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    // 是否需要设定寄存器地址
    if(!use_reg)
    {
        // 16位寄存器地址
        if(reg_size)
        {
            // 写高八位地址
            Si2c_Register_Address(si2c_handle, (uint8_t)(regaddr >> 8));
            if (!Si2c_Check_Ack(si2c_handle)) goto error_exit;
            si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
        }
        // 写低八位地址
        Si2c_Register_Address(si2c_handle, (uint8_t)regaddr);
        if (!Si2c_Check_Ack(si2c_handle)) goto error_exit;
        si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    }
    // 写数据
    if(pdata != NULL && len > 0)
    {
        for(index = 0; index < len; index++)
        {
            Si2c_Write_Data(si2c_handle, pdata[index]);
            if (!Si2c_Check_Ack(si2c_handle)) goto error_exit;
            si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
        }
    }
    Si2c_Stop_Condition(si2c_handle);
    si2c_handle->critical(0);//退出临界区保护
    return SWI2C_TRUE;
error_exit:
    Si2c_Stop_Condition(si2c_handle);
    si2c_handle->critical(0);//退出临界区保护
    return SWI2C_FALSE;
}

/**
 * @brief 发送I2C通用复位命令（General Call Reset）
 *
 * @param[in] si2c_handle I2C控制句柄
 * @return uint8_t 执行结果：
 *   - SWI2C_TRUE:  复位命令发送成功
 *   - SWI2C_FALSE: 设备未应答或操作失败
 * @details
 * - 向总线所有设备广播复位命令（地址0x00 + 数据0x06）
 */
uint8_t Si2c_General_Call_Reset(si2c_t *si2c_handle)
{
    uint8_t index;
    si2c_handle->critical(1);//进入临界区保护
    Si2c_Port_Initial(si2c_handle);
    Si2c_Start_Condition(si2c_handle);
    Si2c_Slave_Address(si2c_handle, 0x00, WRITE_CMD);
    if(!Si2c_Check_Ack(si2c_handle))
    {
        Si2c_Stop_Condition(si2c_handle);
        si2c_handle->critical(0);//退出临界区保护
        return SWI2C_FALSE;
    }
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    Si2c_Register_Address(si2c_handle, 0x06);
    if(!Si2c_Check_Ack(si2c_handle))
    {
        Si2c_Stop_Condition(si2c_handle);
        si2c_handle->critical(0);//退出临界区保护
        return SWI2C_FALSE;
    }
    si2c_handle->hal_delay_us(SI2C_WAIT_TIME);
    Si2c_Stop_Condition(si2c_handle);
    si2c_handle->critical(0);//退出临界区保护
    return SWI2C_TRUE;
}

/**
 * @brief 检测指定I2C从机设备是否在线
 *
 * @param[in] si2c_handle I2C控制句柄
 * @param[in] i2c_id      目标设备地址（包含读写位，bit0值为0）
 * @return uint8_t 检测结果：
 *   - SWI2C_TRUE:  设备在线且应答ACK
 *   - SWI2C_FALSE: 设备无应答或通信失败
 * @details
 * - 通过发送设备地址（写模式）并检测ACK判断设备是否存在
 */
uint8_t Si2c_Check_Slave_Addr(si2c_t *si2c_handle, uint8_t i2c_id)
{
    si2c_handle->critical(1);//进入临界区保护
    Si2c_Start_Condition(si2c_handle);
    Si2c_Slave_Address(si2c_handle, i2c_id, WRITE_CMD);
    if(!Si2c_Check_Ack(si2c_handle))
    {
        Si2c_Stop_Condition(si2c_handle);
        si2c_handle->critical(0);//退出临界区保护
        return SWI2C_FALSE;
    }
    Si2c_Stop_Condition(si2c_handle);
    si2c_handle->critical(0);//退出临界区保护
    return SWI2C_TRUE;
}
