/*
 * joystick.c - STM32F103 双摇杆 + 10键 标准游戏手柄驱动
 *
 * 功能:
 *   - ADC1 四通道扫描 (PA0~PA3), TIM3 触发, DMA Circular 搬运
 *   - 每路独立 CMSIS-DSP Q15 Biquad(1阶IIR低通, fc=50Hz@fs=1kHz)
 *   - 10 个独立按键轮询 + 消抖
 *   - 标准 Gamepad HID 报告打包 (2键 + Hat + 4轴 = 7字节)
 *
 * 引脚依赖 (见 joystick.h 顶部说明):
 *   LF_X/Y = PA0/PA1, RT_X/Y = PA2/PA3
 *   按键: B_X(PA4), B_Y(PA5), B_DU(PA6), B_DD(PA7),
 *         B_LF(PB0), B_RT(PB1), B_B(PB10), B_DL(PB11), B_DR(PB12), B_A(PB13)
 */

#include "joystick.h"

/* ================= 外部句柄 (CubeMX 生成) ================= */
extern ADC_HandleTypeDef  hadc1;
extern DMA_HandleTypeDef  hdma_adc1;
extern TIM_HandleTypeDef  htim3;
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ================= ADC + 滤波相关 ================= */
volatile uint16_t adc_raw[JOYSTICK_ADC_CHANNELS];   /* DMA 循环写入 */
static uint16_t   adc_center[JOYSTICK_ADC_CHANNELS];/* 上电中心校准值 */
static q15_t      adc_filtered[JOYSTICK_ADC_CHANNELS];/* 滤波后 Q15 */

/*
 * 1阶 IIR 低通 (定点 Q15, 验证通过)
 *   y[n] = alpha*x[n] + (1-alpha)*y[n-1]
 *   fs=1000Hz, fc=50Hz -> alpha = 1 - exp(-2*pi*fc/fs) = 0.269597
 *   Q15: b0=b1=8834, a1=-23934 (已取反, 供 CMSIS-DSP 直接累加)
 *   所有系数绝对值<1, 无需 postShift
 *
 * 用 Biquad DF1 实现 1阶: [b0,0,b1,0,a1,0] (b2=a2=0 退化为1阶节)
 */
static q15_t iir_coeffs[6] = {
    8834,    /* b0 = alpha        (x[n]) */
    0,       /* padding (SIMD) */
    8834,    /* b1 = alpha        (x[n-1]) */
    0,       /* b2 = 0 */
    -23934,  /* a1 = -(1-alpha)   (y[n-1], 已取反) */
    0        /* a2 = 0 */
};
#define IIR_POST_SHIFT   0
#define IIR_NUM_STAGES   1

/* 每路独立滤波器实例 + 状态 (避免串扰) */
static arm_biquad_casd_df1_inst_q15 biquad_inst[JOYSTICK_ADC_CHANNELS];
static q15_t biquad_state[JOYSTICK_ADC_CHANNELS][4]; /* 4*numStages */

/* ================= 按键相关 ================= */
static uint8_t key_state[JOY_KEY_COUNT];      /* 消抖后确认状态 (1=按下) */
static uint8_t key_debounce_cnt[JOY_KEY_COUNT];/* 消抖计数器 */
static uint8_t key_raw[JOY_KEY_COUNT];        /* 原始引脚状态 (1=按下) */

/* ================= 内部函数声明 ================= */
static void Joystick_ReadAllKeys(void);
static void Joystick_DebounceKeys(void);
static void Joystick_FilterADC(void);
static void Joystick_CalibrateCenter(void);
static void Joystick_PackKeyBits(uint8_t *report);
static uint8_t Joystick_CalcHat(void);

/* =====================================================================
 * 公共 API 实现
 * ===================================================================== */

/**
 * @brief  摇杆模块初始化
 *         - 初始化 4 路 IIR 滤波器
 *         - 启动 ADC+DMA (TIM3 触发) + TIM3
 *         - 上电中心校准 (摇杆须回中!)
 *         - 初始化按键状态
 */
void Joystick_Init(void) {
    /* 1. 初始化 4 路独立滤波器实例 */
    for (uint8_t i = 0; i < JOYSTICK_ADC_CHANNELS; i++) {
        arm_biquad_cascade_df1_init_q15(
            &biquad_inst[i],
            IIR_NUM_STAGES,
            iir_coeffs,
            biquad_state[i],
            IIR_POST_SHIFT
        );
    }

    /* 2. 启动 ADC DMA (Circular, TIM3 TRGO 触发) */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw, JOYSTICK_ADC_CHANNELS) != HAL_OK) {
        Error_Handler();
    }

    /* 3. 启动 TIM3 (开始周期性触发 ADC) */
    HAL_TIM_Base_Start(&htim3);

    /* 4. 上电中心校准 (摇杆必须回中!) */
    Joystick_CalibrateCenter();

    /* 5. 初始化按键状态 */
    for (uint8_t i = 0; i < JOY_KEY_COUNT; i++) {
        key_state[i] = 0;
        key_debounce_cnt[i] = 0;
        key_raw[i] = 0;
    }
}

/**
 * @brief  1ms 周期更新 (主循环或 TIM 中断中每 1ms 调用一次)
 */
void Joystick_Update(void) {
    Joystick_ReadAllKeys();   /* 读取全部按键原始状态 */
    Joystick_DebounceKeys();  /* 按键消抖 */
    Joystick_FilterADC();     /* 4路 ADC 低通滤波 */
}

/**
 * @brief  填充 7 字节标准 Gamepad HID 报告
 * @param  report: 指向至少 7 字节的缓冲区
 *
 *  格式: [Btn0][Btn1][Hat][LX][LY][RX][RY]
 */
void Joystick_FillHIDReport(uint8_t *report) {
    for (uint8_t i = 0; i < HID_REPORT_SIZE; i++) {
        report[i] = 0;
    }

    /* ---- 按键位打包 (Button1~16) ---- */
    Joystick_PackKeyBits(report);

    /* ---- Hat Switch (方向键) ---- */
    report[HID_REPORT_HAT_OFFSET] = Joystick_CalcHat();

    /* ---- 4 路摇杆轴: Q15 -> 0~255 ---- */
    for (uint8_t i = 0; i < JOYSTICK_ADC_CHANNELS; i++) {
        /* Q15[-32768,32767] +32768 -> [0,65535] >>8 -> [0,255] */
        int32_t axis_val = (int32_t)adc_filtered[i] + 32768;
        axis_val >>= 8;

        /* 死区 (回中归零) */
        int16_t center = HID_AXIS_CENTER;
        if (abs(axis_val - center) < (JOY_DEADZONE_Q15 >> 8)) {
            axis_val = center;
        }

        /* 限幅 */
        if (axis_val < HID_AXIS_MIN) axis_val = HID_AXIS_MIN;
        if (axis_val > HID_AXIS_MAX) axis_val = HID_AXIS_MAX;

        report[HID_REPORT_AXIS_OFFSET + i] = (uint8_t)axis_val;
    }
}

/**
 * @brief  获取指定轴 HID 值 (0~255)
 */
uint8_t Joystick_GetAxis(uint8_t ch) {
    if (ch >= JOYSTICK_ADC_CHANNELS) return HID_AXIS_CENTER;
    int32_t val = (int32_t)adc_filtered[ch] + 32768;
    val >>= 8;
    if (val < HID_AXIS_MIN) val = HID_AXIS_MIN;
    if (val > HID_AXIS_MAX) val = HID_AXIS_MAX;
    return (uint8_t)val;
}

/**
 * @brief  获取指定按键状态 (1=按下, 0=松开)
 */
uint8_t Joystick_GetKeyState(uint8_t key_id) {
    if (key_id >= JOY_KEY_COUNT) return 0;
    return key_state[key_id];
}

/* =====================================================================
 * 内部函数实现
 * ===================================================================== */

/**
 * @brief  读取全部 10 个按键原始引脚状态
 *         独立 GPIO, 上拉输入, 按下=低电平
 */
static void Joystick_ReadAllKeys(void) {
    /* GPIOA 组: X / Y / DU / DD */
    key_raw[KEY_ID_X]  = (HAL_GPIO_ReadPin(B_X_GPIO_Port,  B_X_Pin)  == GPIO_PIN_SET) ? 1 : 0;
    key_raw[KEY_ID_Y]  = (HAL_GPIO_ReadPin(B_Y_GPIO_Port,  B_Y_Pin)  == GPIO_PIN_SET) ? 1 : 0;
    key_raw[KEY_ID_DU] = (HAL_GPIO_ReadPin(B_DU_GPIO_Port, B_DU_Pin) == GPIO_PIN_SET) ? 1 : 0;
    key_raw[KEY_ID_DD] = (HAL_GPIO_ReadPin(B_DD_GPIO_Port, B_DD_Pin) == GPIO_PIN_SET) ? 1 : 0;

    /* GPIOB 组: LF / RF / B / DL / DR / A */
    key_raw[KEY_ID_LF] = (HAL_GPIO_ReadPin(B_LF_GPIO_Port, B_LF_Pin) == GPIO_PIN_SET) ? 1 : 0;
    key_raw[KEY_ID_RF] = (HAL_GPIO_ReadPin(B_RT_GPIO_Port, B_RT_Pin) == GPIO_PIN_SET) ? 1 : 0;
    key_raw[KEY_ID_B]  = (HAL_GPIO_ReadPin(B_B_GPIO_Port,  B_B_Pin)  == GPIO_PIN_SET) ? 1 : 0;
    key_raw[KEY_ID_DL] = (HAL_GPIO_ReadPin(B_DL_GPIO_Port, B_DL_Pin) == GPIO_PIN_SET) ? 1 : 0;
    key_raw[KEY_ID_DR] = (HAL_GPIO_ReadPin(B_DR_GPIO_Port, B_DR_Pin) == GPIO_PIN_SET) ? 1 : 0;
    key_raw[KEY_ID_A]  = (HAL_GPIO_ReadPin(B_A_GPIO_Port,  B_A_Pin)  == GPIO_PIN_SET) ? 1 : 0;
}

/**
 * @brief  按键消抖: 连续 JOY_KEY_DEBOUNCE_MS 次扫描到相同状态才确认
 */
static void Joystick_DebounceKeys(void) {
    for (uint8_t i = 0; i < JOY_KEY_COUNT; i++) {
        if (key_raw[i] != key_state[i]) {
            key_debounce_cnt[i]++;
            if (key_debounce_cnt[i] >= JOY_KEY_DEBOUNCE_MS) {
                key_state[i] = key_raw[i];
                key_debounce_cnt[i] = 0;
            }
        } else {
            key_debounce_cnt[i] = 0;
        }
    }
}

/**
 * @brief  4 路 ADC 低通滤波
 *         adc_raw[] -> 减中心 -> 转Q15 -> arm_biquad_cascade_df1_q15
 */
static void Joystick_FilterADC(void) {
    for (uint8_t i = 0; i < JOYSTICK_ADC_CHANNELS; i++) {
        /* 1. 减中心校准值, 得有符号偏移 (约 +/-2048) */
        int16_t offset = (int16_t)adc_raw[i] - (int16_t)adc_center[i];

        /* 2. 转 Q15: 12位ADC偏移左移4位 -> 16位Q15 (最大 +/-32768, 不溢出) */
        q15_t adc_q15 = offset << 4;

        /* 3. 限幅 (防极端值溢出 Q15) */
        if (adc_q15 > 32767)  adc_q15 = 32767;
        if (adc_q15 < -32768) adc_q15 = -32768;

        /* 4. 1阶低通滤波 (blockSize=1) */
        arm_biquad_cascade_df1_q15(
            &biquad_inst[i],
            &adc_q15,
            &adc_filtered[i],
            1
        );
    }
}

/**
 * @brief  上电中心校准: 摇杆回中状态下采样取平均
 */
static void Joystick_CalibrateCenter(void) {
    HAL_Delay(500); /* 等电源稳定, 用户不要碰摇杆! */

    uint32_t sum[JOYSTICK_ADC_CHANNELS] = {0};
    for (uint8_t t = 0; t < JOY_CENTER_CAL_TIMES; t++) {
        HAL_Delay(1); /* 等 1ms 让 DMA 刷新 adc_raw */
        for (uint8_t ch = 0; ch < JOYSTICK_ADC_CHANNELS; ch++) {
            sum[ch] += adc_raw[ch];
        }
    }
    for (uint8_t ch = 0; ch < JOYSTICK_ADC_CHANNELS; ch++) {
        adc_center[ch] = (uint16_t)(sum[ch] / JOY_CENTER_CAL_TIMES);
    }
}

/**
 * @brief  将 10 个按键状态打包到 HID 报告的按键字节 (Button1~16)
 */
static void Joystick_PackKeyBits(uint8_t *report) {
    /* Byte0: Button1~8 */
    if (key_state[KEY_ID_X])  report[HID_REPORT_BTN0_OFFSET] |= (1 << KEY_X_BIT);  /* B1  X */
    if (key_state[KEY_ID_A])  report[HID_REPORT_BTN0_OFFSET] |= (1 << KEY_A_BIT);  /* B2  A */
    if (key_state[KEY_ID_B])  report[HID_REPORT_BTN0_OFFSET] |= (1 << KEY_B_BIT);  /* B3  B */
    if (key_state[KEY_ID_Y])  report[HID_REPORT_BTN0_OFFSET] |= (1 << KEY_Y_BIT);  /* B4  Y */
    if (key_state[KEY_ID_LF]) report[HID_REPORT_BTN0_OFFSET] |= (1 << KEY_L1_BIT); /* B5  L1 */
    if (key_state[KEY_ID_RF]) report[HID_REPORT_BTN0_OFFSET] |= (1 << KEY_R1_BIT); /* B6  R1 */
    /* B7/B8 空余, 保持 0 */

    /* Byte1: Button9~16 */
    if (key_state[KEY_ID_DR]) report[HID_REPORT_BTN1_OFFSET] |= (1 << (KEY_DR_BIT - 8)); /* B9 D-Pad右兼Button9 */
    /* B10~B16 空余, 保持 0 */
}

/**
 * @brief  根据 DU/DD/DL/DR 四个方向键计算 Hat Switch 值
 *         多键同按或全松均返回 HAT_CENTER(8, Null State)
 */
static uint8_t Joystick_CalcHat(void) {
    uint8_t up    = key_state[KEY_ID_DU];
    uint8_t down  = key_state[KEY_ID_DD];
    uint8_t left  = key_state[KEY_ID_DL];
    uint8_t right = key_state[KEY_ID_DR];

    if (up && !down && !left && !right)        return HAT_UP;
    if (up && right && !down && !left)         return HAT_UPRIGHT;
    if (!up && !down && right && !left)        return HAT_RIGHT;
    if (down && right && !up && !left)         return HAT_DOWNRIGHT;
    if (down && !up && !left && !right)        return HAT_DOWN;
    if (down && left && !up && !right)         return HAT_DOWNLEFT;
    if (!down && !up && left && !right)        return HAT_LEFT;
    if (up && left && !down && !right)         return HAT_UPLEFT;

    return HAT_CENTER; /* 空闲或多键同按 -> Null State */
}

/* =====================================================================
 * 动态更换滤波器系数 (可选扩展接口, 热切换截止频率)
 *   用法: 重新生成系数后调用即可
 * ===================================================================== */
void Joystick_SetFilterCoeffs(q15_t b0, q15_t b1, q15_t a1) {
    iir_coeffs[0] = b0; /* b0 */
    iir_coeffs[2] = b1; /* b1 */
    iir_coeffs[4] = a1; /* a1 (已取反) */
    for (uint8_t i = 0; i < JOYSTICK_ADC_CHANNELS; i++) {
        arm_biquad_cascade_df1_init_q15(
            &biquad_inst[i],
            IIR_NUM_STAGES,
            iir_coeffs,
            biquad_state[i],
            IIR_POST_SHIFT
        );
    }
}
