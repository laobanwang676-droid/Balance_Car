/*    oled_write_command(0xAE); //关闭显示//
    oled_write_command(0xD5); //设置时钟分频因子,震荡频率//
    oled_write_command(0x80); //建议0x80//
    oled_write_command(0xA8); //设置驱动路数//
    oled_write_command(0x3F); //默认0x3F(1/64)//
    oled_write_command(0xD3); //设置显示偏移//
    oled_write_command(0x00); //默认为0//
    oled_write_command(0x40); //设置起始行[5:0]//
    oled_write_command(0x8D); //电荷泵设置
    oled_write_command(0x14); //bit2开启/关闭//
    oled_write_command(0x20); //内存地址模式
    oled_write_command(0x00); //水平地址模式
    oled_write_command(0xA1); //段重定义 设置列地址0映射到SEG127//
    oled_write_command(0xC8); //COM输出扫描方向 remapped mode//
    oled_write_command(0xDA); //COM引脚硬件配置//
    oled_write_command(0x12);//
    oled_write_command(0x81); //对比度设置//
    oled_write_command(0xCF); //建议0xCF//
    oled_write_command(0xD9); //预充电周期设置//
    oled_write_command(0xF1);//
    oled_write_command(0xDB); //VCOMH 电压倍率设置//
    oled_write_command(0x40);//
    oled_write_command(0xA4); //全局显示开启/关闭 全局显示开启//
    oled_write_command(0xA6); //正常/反相显示 正常显示//
    oled_write_command(0xAF); //开启显示//
*/
#ifndef __OLED_H__
#define __OLED_H__
#include <stdint.h>
void oled_init(void);
void oled_set_pos(uint8_t x, uint8_t page);
void oled_write_char(uint8_t x, uint8_t page, char ch, uint8_t size);//显示一个字符
void oled_write_string(uint8_t x, uint8_t page, const char *s, uint8_t size);

#endif /* __OLED_H__ */
