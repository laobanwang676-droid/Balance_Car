#include "stm32f10x.h"
#include "oled.h"
#include "delay.h"
#include "oledfont.h"
uint8_t a[] = {0x00,0x8d,0x14,0xaf,0xa5};

uint8_t data[] = 
{    
    0xAE, // 1. 关闭显示 (0xAE，无参数) 
    0xD5, 0x80,
    // 2. 设置多路复用率 (0xA8 + 参数0x3F)
    0xA8, 0x3F,
    // 3. 设置显示偏移 (0xD3 + 参数0x00)
    0xD3, 0x00,
    // 4. 设置显示起始行 (0x40，无参数)
    0x40,
    0xA1,
    // 8. COM扫描方向（上下翻转）(0xC8，无参数)
    0xC8,
    // 9. COM引脚硬件配置 (0xDA + 参数0x12)
    0xDA, 0x12,
    // 10. 设置对比度 (0x81 + 参数0xCF)
    0x81, 0xCF,
    // 11. 设置预充电周期 (0xD9 + 参数0xF1)
    0xD9, 0xF1,
    // 12. 设置VCOMH电压倍率 (0xDB + 参数0x30)
    0xDB, 0x30,
    // 14. 正常显示模式（按显存显示）(0xA4，无参数)
    0xA4,
    // 15. 正显模式（非反显）(0xA6，无参数)
    0xA6,
    0X8D, 0X14, // 13. 电荷泵设置 (0x8D + 参数0x14)
    // 16. 开启OLED显示 (0xAF，无参数)
    0xAF
};

// 用于合并清屏的一页 128 字节缓冲区
static uint8_t zero_buf[128] = {0};

static void I2C1_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_ClockSpeed = 400000;//提高到400kHz以加快传输（视模块支持情况）
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;//地址 主模式不需要
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}

static void I2C_wait_event(uint32_t event, uint16_t timeout)
{
    while(!I2C_CheckEvent(I2C1, event) && timeout > 0)
    {
        tim_delay_us(2);
        timeout -= 10;
    }
    if(timeout <= 0)
        return;
}

static void oled_write_command(uint8_t cmd[], uint8_t lenth, uint16_t timeout)
{
    I2C_AcknowledgeConfig(I2C1, ENABLE);//使能应答
    I2C_GenerateSTART(I2C1, ENABLE);//发送起始信号
    I2C_wait_event(I2C_EVENT_MASTER_MODE_SELECT, timeout);//等待起始信号发送完成
    I2C_Send7bitAddress(I2C1, 0x78, I2C_Direction_Transmitter);//发送器件地址+写信号
    I2C_wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, timeout);//等待地址发送完成
    I2C_SendData(I2C1, 0x00);//发送数据控制字节（0x40 表示后续为数据）
    I2C_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout);//等待数据发送完成
    for(uint8_t i = 0; i < lenth; i++)
    {
        I2C_SendData(I2C1, cmd[i]);
        I2C_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout);//等待数据发送完成
    }
    I2C_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout);//等待数据发送完成
    I2C_GenerateSTOP(I2C1, ENABLE);
}

static void oled_write_data(uint8_t data[], uint8_t lenth, uint16_t timeout)
{
    I2C_AcknowledgeConfig(I2C1, ENABLE);//使能应答
    I2C_GenerateSTART(I2C1, ENABLE);//发送起始信号
    I2C_wait_event(I2C_EVENT_MASTER_MODE_SELECT, timeout);//等待起始信号发送完成
    I2C_Send7bitAddress(I2C1, 0x78, I2C_Direction_Transmitter);//发送器件地址+写信号
    I2C_wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, timeout);//等待地址发送完成
    I2C_SendData(I2C1, 0x40);//发送数据控制字节（0x40 表示后续为数据）
    I2C_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout);//等待数据发送完成
    for(uint8_t i = 0; i < lenth; i++)
    {
        I2C_SendData(I2C1, data[i]);
        I2C_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout);//等待数据发送完成
    }
    I2C_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout);//等待数据发送完成
    I2C_GenerateSTOP(I2C1, ENABLE);
}

static void oled_clear(void)
{   
    for(uint8_t page = 0; page < 8; page++ )
    {
        oled_set_pos(0, page);
        oled_write_data(zero_buf, 128, 100);
    }
}

void oled_init(void)
{
    I2C1_Init();
    tim_delay_ms(100); //等待OLED电源稳定
    oled_write_command(data, sizeof(data) / sizeof(data[0]), 100);
    tim_delay_ms(100);
    oled_clear();
 }

static void oled_set_pos(uint8_t x, uint8_t page)//设置坐标
{
    uint8_t cmd[3];
    cmd[0] = 0x00 | (x & 0x0F);           
    cmd[1] = 0x10 | ((x & 0xF0) >> 4);          
    cmd[2] = 0xB0 | page;   
    oled_write_command(cmd, 3, 100);
}

void oled_write_char(uint8_t x, uint8_t page, char ch, uint8_t size)//显示一个字符
{
    if(size == 8)
    {
    oled_set_pos(x, page);
    uint8_t c = ch - ' ';//得到偏移后的值
    oled_write_data((uint8_t *)&F6x8[c][0], 6, 100);
    }
    if(size == 16)
    {
        oled_set_pos(x, page);
        uint8_t c = ch - ' ';//得到偏移后的值
        oled_write_data((uint8_t *)&F8X16[c][0], 8, 500);
        oled_set_pos(x, page + 1);
        oled_write_data((uint8_t *)&F8X16[c][8], 8, 500);
    }
}

void oled_write_string(uint8_t x, uint8_t page, const char *s, uint8_t size)
{   
    if (size == 8)
    {
        uint8_t buf[128]; // 一页最多 128 列
        uint16_t idx = 0; // 缓冲区索引
        uint8_t cur_x = x;

        // 先设置起始位置（当前页）
        oled_set_pos(cur_x, page);

        while (*s != '\0')
        {
            uint8_t c = *s - ' ';

            // 如果当前缓冲不足以容纳下一个字符（6 字节），先发送已缓存的数据
            if (idx + 6 > sizeof(buf))
            {
                oled_write_data(buf, idx, 100);
                idx = 0;
                // 发送后继续在当前列 cur_x 写入下一段数据，需更新位置
                oled_set_pos(cur_x, page);
            }

            // 将字符点阵（6 字节）追加到缓冲
            for (uint8_t k = 0; k < 6; k++)
                buf[idx++] = F6x8[c][k];

            cur_x += 6; // 前进列坐标
            s++;
        }

        // 发送剩余数据
        if (idx > 0)
            oled_write_data(buf, idx, 500);

        return;
    }
    else if (size == 16)
    {
        while (*s != '\0')
        {
            oled_write_char(x, page, *s, size);
            x += 8;
            s++;
        }
    }
}
