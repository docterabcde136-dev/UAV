/**
 * @file    bsp_oled.c
 * @brief   OLED 显示屏板级驱动实现（SPI 模拟时序）
 * @note    OLED 型号: SSD1306 兼容, 分辨率 128x64
 *          接口: 4 线软件 SPI（SCL, SDA, RST, DC）
 *          引脚: PC13=SCL, PC14=SDA, PC15=RST, PC0=DC
 * @author  lf
 * @date    2022-06-11
 */

#include "bsp_oled.h"
#include "oledfont.h"
#include "gpio.h"

/* OLED 引脚宏定义（软件 SPI 模拟）---------------------------------------------*/
/* 4 线 SPI 接口: SCL=时钟, SDA=数据, RST=复位, DC=命令/数据选择 */
#define OLED_RST_Clr()  HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET)  /* RST 拉低 */
#define OLED_RST_Set()  HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET)    /* RST 拉高 */

#define OLED_RS_Clr()   HAL_GPIO_WritePin(OLED_DC_GPIO_Port,  OLED_DC_Pin,  GPIO_PIN_RESET)  /* DC=0: 命令 */
#define OLED_RS_Set()   HAL_GPIO_WritePin(OLED_DC_GPIO_Port,  OLED_DC_Pin,  GPIO_PIN_SET)    /* DC=1: 数据 */

#define OLED_SCLK_Clr() HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET)  /* SCL 低电平 */
#define OLED_SCLK_Set() HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET)    /* SCL 高电平 */

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_RESET)  /* SDA 低电平 */
#define OLED_SDIN_Set() HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET)    /* SDA 高电平 */

#define OLED_CMD  0  /* DC 拉低：写入的是命令 */
#define OLED_DATA 1  /* DC 拉高：写入的是数据 */

/* 函数前置声明 ---------------------------------------------------------------*/
static void OLED_WR_Byte(uint8_t dat, uint8_t cmd);

/* OLED 显存（GRAM）-----------------------------------------------------------*/
/* 128 列 x 8 页，每页 8 像素行 = 64 行 */
uint8_t OLED_GRAM[128][8];

/**
 * @brief  刷新 OLED 屏幕（将 GRAM 全部写入 OLED 控制器）
 * @note   按页地址模式写入，共 8 页，每页 128 列
 */
static void OLED_Refresh_Gram(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD);   /* 设置页地址（0~7） */
        OLED_WR_Byte(0x00, OLED_CMD);       /* 设置列低地址 */
        OLED_WR_Byte(0x10, OLED_CMD);       /* 设置列高地址 */
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
    }
}

/**
 * @brief  向 OLED 写入一个字节（软件 SPI 时序，MSB 优先）
 * @param  dat: 要写入的数据/命令
 * @param  cmd: 数据类型标志（OLED_CMD=0: 命令, OLED_DATA=1: 数据）
 */
static void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    uint8_t i;
    if (cmd)
        OLED_RS_Set();                      /* DC=1: 写数据 */
    else
        OLED_RS_Clr();                      /* DC=0: 写命令 */

    for (i = 0; i < 8; i++)
    {
        OLED_SCLK_Clr();                    /* SCL 拉低，准备数据 */
        if (dat & 0x80)
            OLED_SDIN_Set();                /* 输出 bit=1 */
        else
            OLED_SDIN_Clr();                /* 输出 bit=0 */
        OLED_SCLK_Set();                    /* SCL 上升沿写入数据 */
        dat <<= 1;                          /* 左移准备下一位 */
    }
    OLED_RS_Set();                          /* 恢复 DC 为高 */
}

/* 以下注释掉的函数为 SSD1306 电源控制函数，保留备用 ---------------------------*/
/*
static void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);   // SET DCDC 命令
    OLED_WR_Byte(0X14, OLED_CMD);   // DCDC ON
    OLED_WR_Byte(0XAF, OLED_CMD);   // DISPLAY ON
}

static void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);   // SET DCDC 命令
    OLED_WR_Byte(0X10, OLED_CMD);   // DCDC OFF
    OLED_WR_Byte(0XAE, OLED_CMD);   // DISPLAY OFF
}
*/

/**
 * @brief  清屏函数（将 GRAM 全部清零后刷新）
 * @note   整个屏幕变为黑色（像素不点亮）
 */
static void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
        for (n = 0; n < 128; n++)
            OLED_GRAM[n][i] = 0X00;         /* GRAM 全写零 */
    OLED_Refresh_Gram();                    /* 更新显示 */
}

/**
 * @brief  画点函数
 * @param  x: 横坐标（0~127）
 * @param  y: 纵坐标（0~63）
 * @param  t: 1=点亮该像素, 0=熄灭该像素
 * @note   超出范围自动忽略
 */
static void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t pos, bx, temp = 0;
    if (x > 127 || y > 63) return;          /* 超出屏幕范围，直接返回 */
    pos = 7 - y / 8;                         /* 计算所在页 */
    bx  = y % 8;                             /* 计算页内位偏移 */
    temp = 1 << (7 - bx);                    /* 生成位掩码 */
    if (t)
        OLED_GRAM[x][pos] |= temp;           /* 点亮该像素 */
    else
        OLED_GRAM[x][pos] &= ~temp;          /* 熄灭该像素 */
}

/**
 * @brief  在指定位置显示一个 ASCII 字符
 * @param  x:    起始横坐标
 * @param  y:    起始纵坐标
 * @param  chr:  要显示的字符（ASCII码）
 * @param  size: 字体大小（12=6x12, 16=8x16）
 * @param  mode: 显示模式（0=反白显示, 1=正常显示）
 */
static void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';                          /* 计算字库偏移（字库从空格开始） */
    for (t = 0; t < size; t++)
    {
        if (size == 12)
            temp = oled_asc2_1206[chr][t];    /* 调用 1206 字体 */
        else
            temp = oled_asc2_1608[chr][t];    /* 调用 1608 字体 */
        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp <<= 1;
            y++;
            if ((y - y0) == size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

/**
 * @brief  求 m 的 n 次方
 * @param  m: 底数
 * @param  n: 指数
 * @retval m 的 n 次方结果
 */
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/**
 * @brief  显示整数数字
 * @param  x:    起始横坐标
 * @param  y:    起始纵坐标
 * @param  num:  要显示的数值（0~4294967295）
 * @param  len:  数字的位数
 * @param  size: 字体大小（12 或 16）
 * @note   高位 0 不显示（前导零抑制），但最后一位始终显示
 */
static void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;                       /* 前导零抑制标志 */
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size / 2) * t, y, ' ', size, 1);
                continue;
            }
            else
                enshow = 1;                   /* 遇到第一个非零位，开始正常显示 */
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, 1);
    }
}

/**
 * @brief  显示字符串
 * @param  x: 起始横坐标
 * @param  y: 起始纵坐标
 * @param  p: 要显示的字符串（以 '\0' 结尾）
 * @note   超出屏幕范围自动换行/清屏
 */
static void OLED_ShowString(uint8_t x, uint8_t y, const char *p)
{
#define MAX_CHAR_POSX 122                       /* 最大横坐标边界 */
#define MAX_CHAR_POSY 58                        /* 最大纵坐标边界 */
    while (*p != '\0')
    {
        if (x > MAX_CHAR_POSX) { x = 0; y += 16; }  /* 横向越界换行 */
        if (y > MAX_CHAR_POSY) { y = x = 0; OLED_Clear(); }  /* 纵向越界清屏 */
        OLED_ShowChar(x, y, *p, 12, 1);
        x += 8;
        p++;
    }
}

/**
 * @brief  OLED 初始化函数
 * @note   按照 SSD1306 数据手册的初始化序列配置
 *         包括：时钟分频、驱动路数、显示偏移、电荷泵、内存地址模式、
 *         段重定义、COM 扫描方向、对比度、预充电周期、VCOMH 电压等
 */
static void OLED_Init(void)
{
    /* 硬件复位时序 */
    OLED_RST_Clr();
    HAL_Delay(100);
    OLED_RST_Set();

    OLED_WR_Byte(0xAE, OLED_CMD);   /* 关闭显示 */
    OLED_WR_Byte(0xD5, OLED_CMD);   /* 设置时钟分频因子/振荡频率 */
    OLED_WR_Byte(80, OLED_CMD);     /* [3:0]=分频因子, [7:4]=振荡频率 */
    OLED_WR_Byte(0xA8, OLED_CMD);   /* 设置驱动路数 */
    OLED_WR_Byte(0X3F, OLED_CMD);   /* 默认 0x3F（1/64 占空比） */
    OLED_WR_Byte(0xD3, OLED_CMD);   /* 设置显示偏移 */
    OLED_WR_Byte(0X00, OLED_CMD);   /* 默认偏移为 0 */

    OLED_WR_Byte(0x40, OLED_CMD);   /* 设置显示起始行 [5:0] */

    OLED_WR_Byte(0x8D, OLED_CMD);   /* 电荷泵设置 */
    OLED_WR_Byte(0x14, OLED_CMD);   /* bit2: 电荷泵开启 */
    OLED_WR_Byte(0x20, OLED_CMD);   /* 设置内存地址模式 */
    OLED_WR_Byte(0x02, OLED_CMD);   /* [1:0]=10: 页地址模式 */
    OLED_WR_Byte(0xA1, OLED_CMD);   /* 段重定义: bit0=1, 列地址 0->127 */
    OLED_WR_Byte(0xC0, OLED_CMD);   /* COM 扫描方向: 正常模式 COM0->COM[N-1] */
    OLED_WR_Byte(0xDA, OLED_CMD);   /* COM 硬件引脚配置 */
    OLED_WR_Byte(0x12, OLED_CMD);   /* [5:4] 配置 */

    OLED_WR_Byte(0x81, OLED_CMD);   /* 对比度设置 */
    OLED_WR_Byte(0xEF, OLED_CMD);   /* 对比度值 1~255（越大越亮），默认 0x7F */
    OLED_WR_Byte(0xD9, OLED_CMD);   /* 预充电周期 */
    OLED_WR_Byte(0xf1, OLED_CMD);   /* [3:0]=PHASE1, [7:4]=PHASE2 */
    OLED_WR_Byte(0xDB, OLED_CMD);   /* VCOMH 电压倍率 */
    OLED_WR_Byte(0x30, OLED_CMD);   /* [6:4]=000: 0.65*Vcc */

    OLED_WR_Byte(0xA4, OLED_CMD);   /* 全局显示: bit0=1 开启全屏点亮 */
    OLED_WR_Byte(0xA6, OLED_CMD);   /* 显示模式: bit0=0 正常显示, bit0=1 反相显示 */
    OLED_WR_Byte(0xAF, OLED_CMD);   /* 开启显示 */
    OLED_Clear();                   /* 清屏 */
}

/* 以下为注释掉的位置设置函数，保留备用 ----------------------------------------*/
/*
static void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0f), OLED_CMD);
}
*/

/**
 * @brief  浮点数分解为整数部分和小数部分
 * @param  fudian: 要分解的浮点数
 * @retval 静态数组指针: tmp[0]=整数部分, tmp[1]=小数部分(3位), tmp[2]=符号(1正/-1负)
 * @note   返回静态缓冲区，每次调用会覆盖上次结果
 */
static int* FenJie_float(const float fudian)
{
    static int tmp[3];
    float temp;
    temp = fudian;
    if (temp < 0)
        temp = -temp, tmp[2] = -1;           /* 负数：取绝对值，符号位=-1 */
    else
        tmp[2] = 1;                          /* 正数：符号位=1 */

    tmp[0] = (int)temp;                      /* 提取整数部分 */
    tmp[1] = (temp - tmp[0]) * 1000;         /* 提取小数部分（保留3位） */
    return tmp;
}

/**
 * @brief  OLED 显示浮点数
 * @param  show_x:     显示起始横坐标
 * @param  show_y:     显示起始纵坐标
 * @param  needtoshow: 要显示的浮点数
 * @param  zs_num:     整数部分显示位数
 * @param  xs_num:     小数部分显示位数（2 或 3）
 * @note   格式: [+/-]整数.小数（如 "+12.345"）
 */
static void oled_showfloat(uint8_t show_x, uint8_t show_y,
                           const float needtoshow,
                           uint8_t zs_num, uint8_t xs_num)
{
    static int* p;
    p = FenJie_float(needtoshow);

    /* 显示符号位 */
    if (p[2] > 0)
        OLED_ShowChar(show_x, show_y, '+', 12, 1);
    else
        OLED_ShowChar(show_x, show_y, '-', 12, 1);

    /* 显示整数部分 */
    OLED_ShowNumber(show_x + 8, show_y, p[0], zs_num, 12);

    /* 显示小数点 */
    OLED_ShowChar(show_x + 6 + 8 * zs_num, show_y, '.', 12, 1);

    /* 显示小数部分（含前导零补全） */
    if (p[1] < 100)                           /* 小数部分不足 3 位需补零 */
    {
        if (xs_num == 3)                      /* 显示 3 位小数 */
        {
            OLED_ShowNumber(show_x + 12 + 8 * zs_num, show_y, 0, 1, 12);
            if (p[1] >= 10)
                OLED_ShowNumber(show_x + 18 + 8 * zs_num, show_y, p[1], 2, 12);
            else
            {
                OLED_ShowNumber(show_x + 18 + 8 * zs_num, show_y, 0, 1, 12);
                OLED_ShowNumber(show_x + 24 + 8 * zs_num, show_y, p[1], 1, 12);
            }
        }
        else                                  /* 显示 2 位小数 */
        {
            if (p[1] >= 0 && p[1] < 100)
                OLED_ShowNumber(show_x + 12 + 8 * zs_num, show_y, 0, 1, 12);
            OLED_ShowNumber(show_x + 18 + 8 * zs_num, show_y, p[1] / 10, 1, 12);
        }
    }
    else                                      /* 小数部分满 3 位，直接显示 */
    {
        if (xs_num == 3)
            OLED_ShowNumber(show_x + 12 + 8 * zs_num, show_y, p[1], 3, 12);
        else
            OLED_ShowNumber(show_x + 12 + 8 * zs_num, show_y, p[1] / 10, 2, 12);
    }
}

/* OLED 接口结构体实例 ---------------------------------------------------------*/
/* 通过函数指针向上层 APP 暴露统一的 OLED 操作接口 */

OLEDInterface_t UserOLED = {
    .init        = OLED_Init,         /* OLED 硬件初始化 */
    .ShowChar    = OLED_ShowChar,     /* 显示字符 */
    .ShowNumber  = OLED_ShowNumber,   /* 显示整数 */
    .ShowString  = OLED_ShowString,   /* 显示字符串 */
    .ShowFloat   = oled_showfloat,    /* 显示浮点数 */
    .RefreshGram = OLED_Refresh_Gram, /* 刷新 GRAM 到屏幕 */
    .Clear       = OLED_Clear,        /* 清屏 */
};
