/**
 * @file    spwm.c
 * @brief   三相 SPWM 开环驱动实现
 *
 * 工作原理：
 *   SPWM 不经过 Clarke/Park/SVPWM，直接在三相上输出互差 120° 电角度的
 *   正弦调制波：
 *       duty_u = 0.5 + k·sin(θ)
 *       duty_v = 0.5 + k·sin(θ - 2π/3)
 *       duty_w = 0.5 + k·sin(θ + 2π/3)
 *   其中 θ 为电角度指令 θ_cmd，由定时中断匀速递增（开环虚构角度），
 *   k 为调制深度 = 相电压幅值 / 母线电压。
 *
 *   三相占空比均值恒为 50%（共模抵消），线电压为对称三相正弦，
 *   合成一个幅值 k·VM、角度 θ 的旋转电压矢量。转子被该矢量以同步
 *   电机方式拖动旋转（功角 < 90° 时同步稳定，转速过高或负载突变
 *   会失步——这是开环的本质局限）。
 *
 *   与闭环 FOC 的关系：本模块仅用于验证功率链路（PWM 相序、驱动使能、
 *   同步拖动）。闭环时弃用本模块，改用编码器实测电角度 + SVPWM。
 *
 * 硬件连接（见 hardware/SCH.pdf）：
 *   TIM2_CH1(PA0) → DRV8313 IN1   ┐
 *   TIM2_CH3(PA2) → DRV8313 IN2   ├ 三相桥输入，中心对齐 10kHz
 *   TIM2_CH4(PA3) → DRV8313 IN3   ┘
 *   PA7(MOTOR_EN) → DRV8313 EN1/2/3（三相使能并联，高电平有效）
 *   SLEEP#/RESET# 板上接地（常使能），FAULT# 未接 MCU
 */
#include "spwm.h"
#include "main.h"      /* htim2、MOTOR_EN_Pin / MOTOR_EN_GPIO_Port 标签 */
#include <math.h>

extern TIM_HandleTypeDef htim2;   /* CubeMX 生成，三相 PWM 载波 */

/* ============================ 参数宏 ============================ */
#define PWM_PERIOD     1800.0f   /* TIM2 ARR = 1800，中心对齐 → PWM = 72MHz/(2×1800) = 20kHz，
                                  * 与 CubeMX 里 TIM2.Period 一致 */
#define ADC_TRIG_PULSE 1700U     /* TIM2 CH2 比较值：ARR - 100，用作 ADC 注入触发
                                  * （小 < ARR，否则 OC2REF 恒高触发不了） */
#define VM             12.4f     /* 母线电压：USB 5V 经 MT3608 升压 ≈12.4V，实测后修正 */
#define TICK_FREQ      1000.0f   /* TIM3 中断频率：72MHz/72/1000 = 1kHz，须与 CubeMX 一致 */
#define POLE_PAIRS     7.0f      /* 电机极对数：spwm_start/spwm_set_speed 参数为机械
                                  * 角速度，内部 ×POLE_PAIRS 换算为电角速度 */
#define K_MOD_MAX      0.3f      /* 调制深度上限：0.3×12.4V ≈ 3.7V 相幅值，开环安全限幅 */
#define TWO_PI         6.2831853f

/* ========================== 模块内状态 ==========================
 * 均加 volatile：theta_cmd/omega/k_mod/running 在中断里写，
 * 主循环可通过接口读（get_angle），避免被编译器优化。 */
static volatile float theta_cmd;  /* 电角度指令（rad，0~2π），开环虚构角度 */
static volatile float omega;      /* 电角速度指令（rad/s），= 设定机械角速度 × POLE_PAIRS */
static volatile float k_mod;      /* 调制深度（0~K_MOD_MAX），= 相电压幅值 / VM */
static volatile bool  running;    /* 运行标志：false 时 tick 直接返回，保持 50% 零矢量 */

/**
 * @brief 初始化：复位内部状态，三相输出 50% 零矢量并启动 PWM 载波
 * @note  此时 PA7 仍为低（CubeMX GPIO 初始电平），DRV8313 未使能，
 *        三相输出等效 0V，电机不通电——安全。
 * @note  调用顺序：MX_TIM2_Init() → spwm_init() → ... → spwm_start()
 */
void spwm_init(void)
{
    theta_cmd = 0.0f;
    omega     = 0.0f;
    k_mod     = 0.0f;
    running   = false;

    /* 三相 50% 占空比 = 零矢量（相电压均为 VM/2，线电压为 0），
     * 先写比较寄存器再启动 PWM，保证第一拍就是安全电平 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(PWM_PERIOD * 0.5f));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(PWM_PERIOD * 0.5f));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(PWM_PERIOD * 0.5f));

    /* ---- TIM2 CH2：ADC 注入专用触发通道（TRGO = OC2REF）----
     * 最大值必须 < ARR(=1800)：中心对齐下 CNT 最大就是 ARR，
     * CCR2 >= ARR+1 时 PWM1 的 CNT<CCR2 恒成立，OC2REF 恒为高，一次都不触发。
     * 当前值 1700 = ARR - 100，即采样点在 CNT=ARR 顶点之后 100 个 tick。
     * 必须与 tim.c / firmware.ioc 里 TIM2 Pulse-PWM Generation2 No Output 一致。 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ADC_TRIG_PULSE);

    /* 手动产生一次更新事件，把上面四个 CCR 从预装载寄存器刷进影子寄存器，
     * 不用等计数器自然跑到第一个 UEV（CMRx 的 OCxPE 是 HAL 默认打开的）。 */
    // SET_BIT(htim2.Instance->EGR, TIM_EGR_UG);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* PA0 → IN1 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   /* 仅开 CC2E，PA1 未配 AF 故不输出引脚，
                                                 * OC2REF 内部送给 TRGO 触发 ADC */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);   /* PA2 → IN2 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   /* PA3 → IN3 */
}

/**
 * @brief 启动开环旋转（只能调用一次！）
 * @param omega_rads 机械角速度 rad/s（1 圈/s = 6.28），内部 ×7 → 电角速度
 * @param volt       相电压幅值 V，换算为调制深度 k = volt/VM
 * @note  执行顺序不可颠倒：先清角度、算好 k → 再拉高 EN → 最后置 running。
 *        若 EN 拉高时占空比还不是 50% 零矢量，MOS 会有电压冲击。
 * @note  重复调用会把 theta_cmd 清零 → 角度停摆、电机锁死，务必只调一次。
 */
void spwm_start(float omega_rads, float volt)
{
    theta_cmd = 0.0f;                 /* 从电角度 0 起转 */
    omega     = omega_rads * POLE_PAIRS;   /* 机械 → 电角速度换算 */
    k_mod     = volt / VM;            /* 电压幅值 → 调制深度 */
    if (k_mod > K_MOD_MAX) k_mod = K_MOD_MAX;   /* 安全限幅：≤3.7V 相幅值 */

    /* 三相使能（PA7 → EN1/2/3），此时占空比已是上次 tick/init 的 50% 零矢量 */
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
    running = true;                   /* 置标志后，下一个 TIM3 tick 开始推进角度 */
}

/**
 * @brief 停止旋转并关断驱动
 * @note  顺序：先停 tick（running=false，不再推进角度）→ 三相回 50% 零矢量
 *        → 拉低 PA7 关断。保证关断瞬间 MOS 上无电压差。
 */
void spwm_stop(void)
{
    running = false;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(PWM_PERIOD * 0.5f));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(PWM_PERIOD * 0.5f));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(PWM_PERIOD * 0.5f));
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 运行中修改目标机械角速度（单位与 spwm_start 一致：rad/s 机械，内部 ×7）
 * @note  只改推进速率、不清角度，电机平滑加减速，可用于正反转验证
 *        （传负值反转；若实际不反转，说明相序需调整——对调 V/W 两根电机线）。
 */
void spwm_set_speed(float omega_rads)
{
    omega = omega_rads * POLE_PAIRS;   /* 与 spwm_start 保持同一参数语义 */
}

/**
 * @brief 读取当前电角度指令 θ_cmd（0~2π）
 * @return 电角度 rad
 * @note  调试用法：主循环每 1s 打印一次，增量应 ≈ 设定机械角速度 × 7
 *        （例：6.28 rad/s 机械 → 每秒增 44）。偏差倍数即 TIM3 实际
 *        频率与 TICK_FREQ 的偏差倍数，用于快速核对中断配置。
 */
float spwm_get_angle(void)
{
    return theta_cmd;
}

/**
 * @brief SPWM 节拍：由 TIM3 1kHz 更新中断调用，推进角度并刷新三相占空比
 * @note  单次执行内容：角度积分 → 3 次 sinf → 3 次写比较寄存器。
 *        sinf 在 72MHz M3 上约几 µs，1kHz 节拍下开销可忽略；
 *        若将来把调制频率提到 10kHz 以上，需换查表法（example1 sine_array）
 *        或 CMSIS-DSP arm_sin_f32。
 */
void spwm_tick(void)
{
    if (!running) return;             /* 未启动：保持 50% 零矢量不动 */

    /* ---- 1. 角度推进：θ += ω·dt，dt = 1/TICK_FREQ ----
     * 正负方向都做 2π 回卷，防止 float 长期累计造成精度损失 */
    theta_cmd += omega / TICK_FREQ;
    if (theta_cmd >= TWO_PI)      theta_cmd -= TWO_PI;
    else if (theta_cmd < 0.0f)    theta_cmd += TWO_PI;

    /* ---- 2. 三相正弦调制：互差 120° 电角度 ----
     * 提前算好三个 sin 值再统一写寄存器，缩短临界区时间 */
    const float TWO_PI_3 = 2.0943951f;          /* 2π/3 */
    float s_u = sinf(theta_cmd);                /* U 相：θ          */
    float s_v = sinf(theta_cmd - TWO_PI_3);     /* V 相：θ - 120°   */
    float s_w = sinf(theta_cmd + TWO_PI_3);     /* W 相：θ + 120°   */

    /* ---- 3. 写占空比：duty = (0.5 + k·sinθ)·ARR ----
     * 不需要关中断做临界区保护：HAL_TIM_PWM_ConfigChannel() 已经给 CH1~CH4
     * 置了 CCMRx 的 OCxPE 位，写 CCRx 只是写预装载寄存器，要等下一个 UEV
     * 才一起传送到影子寄存器 —— 三相比值天然在同一个计数边界生效，
     * 不存在"写到一半被打断"的问题。
     * 反而关中断会阻塞 ADC 注入完成中断（20kHz），给采样时刻引入抖动，
     * 所以这里直接写即可。（CH2 是 ADC 触发通道，任何情况下都别写它的 CCR。） */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
        (uint32_t)((0.5f + k_mod * s_u) * PWM_PERIOD));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
        (uint32_t)((0.5f + k_mod * s_v) * PWM_PERIOD));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,
        (uint32_t)((0.5f + k_mod * s_w) * PWM_PERIOD));
}
