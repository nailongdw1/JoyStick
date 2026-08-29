/*
 * joystick.h - STM32F103 双摇杆 + 10键 标准游戏手柄驱动 (Custom HID)
 *
 * 引脚定义（由用户重新定义）：
 *   摇杆 ADC (ADC1):
 *     LF_X = PA0 (IN0)   左摇杆 X
 *     LF_Y = PA1 (IN1)   左摇杆 Y
 *     RT_X = PA2 (IN2)   右摇杆 X
 *     RT_Y = PA3 (IN3)   右摇杆 Y
 *
 *   按键 (独立 GPIO, 上拉输入, 按下=低电平):
 *     B_X   = PA4   X 键        (Button 1)
 *     B_Y   = PA5   Y 键        (Button 4)
 *     B_DU  = PA6   D-Pad 上    (Hat Switch)
 *     B_DD  = PA7   D-Pad 下    (Hat Switch)
 *     B_LF  = PB0   左前 L1     (Button 5)
 *     B_RT  = PB1   右前 R1     (Button 6)
 *     B_B   = PB10  B 键        (Button 3)
 *     B_DL  = PB11  D-Pad 左    (Hat Switch)
 *     B_DR  = PB12  D-Pad 右    (Hat Switch)
 *     B_A   = PB13  A 键        (Button 2)
 *
 * HID 报告格式 (标准 Gamepad, 共 7 字节):
 *   [0]   Button 1~8   (X/Y/DU/DD/LF/RT/B/DL? -> 见 joystick.c 打包)
 *   [1]   Button 9~16  (DR/A/... 空余补 0)
 *   [2]   Hat Switch   (方向键, Null=8 表示空闲)
 *   [3]   左摇杆 X (0~255)
 *   [4]   左摇杆 Y (0~255)
 *   [5]   右摇杆 X (0~255)  Usage Z
 *   [6]   右摇杆 Y (0~255)  Usage Rz
 */

#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include <stdint.h>
#include "math.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "main.h"
#include "usbd_customhid.h"

/* ================= 摇杆(ADC)配置 ================= */
#define JOYSTICK_ADC_CHANNELS      4       /* 4路摇杆: LF_X/LF_Y/RT_X/RT_Y */
#define JOY_ADC_FS                 1000    /* ADC采样率 1kHz (TIM3触发) */
#define JOY_CUTOFF_FREQ            50      /* IIR低通截止频率 50Hz */
#define JOY_CENTER_CAL_TIMES       16      /* 上电中心校准采样次数 */
#define JOY_DEADZONE_Q15           256     /* 摇杆死区 (Q15, 对应ADC约±32) */

/* ================= 按键配置 ================= */
#define JOY_KEY_COUNT              10      /* 总按键数 */
#define JOY_KEY_DEBOUNCE_MS        5       /* 按键消抖 5ms */

/* 按键 ID 枚举 (和 HID Button 编号对应) */
typedef enum {
    KEY_ID_X    = 0,    /* X 键    -> Button 1 */
    KEY_ID_Y    = 1,    /* Y 键    -> Button 4 */
    KEY_ID_DU   = 2,    /* D-Pad 上 -> Hat Switch */
    KEY_ID_DD   = 3,    /* D-Pad 下 -> Hat Switch */
    KEY_ID_LF   = 4,    /* 左前 L1 -> Button 5 */
    KEY_ID_RF   = 5,    /* 右前 R1 -> Button 6 */
    KEY_ID_B    = 6,    /* B 键    -> Button 3 */
    KEY_ID_DL   = 7,    /* D-Pad 左 -> Hat Switch */
    KEY_ID_DR   = 8,    /* D-Pad 右 -> Hat Switch */
    KEY_ID_A    = 9     /* A 键    -> Button 2 */
} key_id_t;

/* HID Button 位映射 (Byte0/Byte1) */
#define KEY_X_BIT    0    /* Button 1  -> Byte0 Bit0 */
#define KEY_Y_BIT    3    /* Button 4  -> Byte0 Bit3 */
#define KEY_L1_BIT   4    /* Button 5  -> Byte0 Bit4 (L1=左前) */
#define KEY_R1_BIT   5    /* Button 6  -> Byte0 Bit5 (R1=右前) */
#define KEY_B_BIT    2    /* Button 3  -> Byte0 Bit2 */
#define KEY_A_BIT    1    /* Button 2  -> Byte0 Bit1 */
#define KEY_DR_BIT   8    /* Button 9  -> Byte1 Bit0 (D-Pad右 兼 Button9) */

/* ================= HID 报告配置 ================= */
#define HID_REPORT_SIZE            7       /* 2字节键 + 1字节Hat + 4字节轴 */
#define HID_REPORT_BTN0_OFFSET     0       /* Byte0: Button1~8 */
#define HID_REPORT_BTN1_OFFSET     1       /* Byte1: Button9~16 */
#define HID_REPORT_HAT_OFFSET      2       /* Byte2: Hat Switch */
#define HID_REPORT_AXIS_OFFSET     3       /* Byte3~6: 4轴 */
#define HID_REPORT_AXIS_LX_OFFSET  3       /* 左摇杆 X */
#define HID_REPORT_AXIS_LY_OFFSET  4       /* 左摇杆 Y */
#define HID_REPORT_AXIS_RX_OFFSET  5       /* 右摇杆 X (Usage Z) */
#define HID_REPORT_AXIS_RY_OFFSET  6       /* 右摇杆 Y (Usage Rz) */

/* 轴值域 */
#define HID_AXIS_MIN               0
#define HID_AXIS_MAX               255
#define HID_AXIS_CENTER            128     /* 255/2 */

/* Hat Switch 方向值 (Null State = 8 表示空闲) */
#define HAT_CENTER                 8
#define HAT_UP                     0
#define HAT_UPRIGHT                1
#define HAT_RIGHT                  2
#define HAT_DOWNRIGHT              3
#define HAT_DOWN                   4
#define HAT_DOWNLEFT               5
#define HAT_LEFT                   6
#define HAT_UPLEFT                 7

/* ================= 对外 API ================= */
void     Joystick_Init(void);
void     Joystick_Update(void);
void     Joystick_FillHIDReport(uint8_t *report);
uint8_t  Joystick_GetAxis(uint8_t ch);
uint8_t  Joystick_GetKeyState(uint8_t key_id);
void     Joystick_SetFilterCoeffs(q15_t b0, q15_t b1, q15_t a1);

#endif /* __JOYSTICK_H */
