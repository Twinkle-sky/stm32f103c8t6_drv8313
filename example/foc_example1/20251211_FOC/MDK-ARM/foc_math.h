#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#include <stdint.h>

/* 常数定义 (Q10格式) */
#define _PI_2_1000         1570      /* π/2 * 1000 */
#define _PI_1000            3141      /* π * 1000 */
#define _3PI_2_1000        4712      /* 3π/2 * 1000 */
#define _2PI_1000           6283      /* 2π * 1000 */
#define _PI_3_1000 					1047 			/* π/3 * 1000 */
#define _PI_6_Q10 				523				/* π/6 * 1000 */
/* 预计算常数 (Q10格式) */
#define ONE_OVER_SQRT3_Q10 591       /* 1/√3 ≈ 0.577 * 1024 */
#define SQRT3_OVER_2_Q10   887       /* √3/2 ≈ 0.866 * 1024 */
#define SQRT3_Q10          1773      /* √3 ≈ 1.732 * 1024 = 1773 */

/* 电压限制 (Q10格式) */
#define U_MAX_LIMIT_Q10    307       /* 0.300/1024 * 1024 = 300，但实际限制为307对应0.3 */
#define U_MAX_ABS_Q10      307       /* 最大电压绝对值 */

/* 正弦表 (0-π/2范围，Q10格式) */
//const int16_t sine_array[1571] = {
//    /* 这里应该填充实际的sine表值，从sin(0)到sin(π/2) */
//    0, 2, 4, 6, /* ... 省略，实际使用时需要填充完整 */
//    /* 注意：数组大小是1571，对应0-1570(包含) */
//};

extern const int16_t sine_array[1572];


/* 函数声明 */
/* 三角函数 */
int32_t _sin_q10(uint16_t a);
int32_t _cos_q10(uint16_t a);
uint16_t _normalizeAngle(int32_t angle);

/* 数学运算 */
int32_t _sqrt_fast(int32_t x);
int32_t fast_atan2_int(int32_t y, int32_t x);
int32_t FOC_MulQ10(int32_t a, int32_t b);


#endif /* __FOC_MATH_H */