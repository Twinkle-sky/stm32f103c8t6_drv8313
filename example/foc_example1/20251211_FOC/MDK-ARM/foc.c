#include "foc.h"
#include "foc_math.h"

volatile PWM_Duty duty = {512, 512, 512};  /* 初始化为50%占空比 (512/1024) */
volatile PWM_Duty dutyPeriod = {1800, 1800, 1800};  /* 初始化为50%占空比 (512/1024) */


/**
  * @brief  Clark变换
  * @param  currents: 电流结构体指针，输入a,b相电流，计算c,alpha,beta
  * @retval None
  */
void FOC_ClarkTransform(Currents* currents)
{
    /* Clark变换公式：
     * alpha = a
     * beta = (a + 2b) / √3
     * 假设 ia + ib + ic = 0
     */
    
    /* 计算C相电流 */
    currents->c = -currents->a - currents->b;
    
    /* α轴电流 = A相电流 */
    currents->alpha = currents->a;
    
    /* β = (a + 2b) * (1/√3) */
    volatile int32_t numerator = currents->a + (currents->b << 1);  /* a + 2b */
    currents->beta = FOC_MulQ10(numerator, ONE_OVER_SQRT3_Q10);
}

/**
  * @brief  Park变换
  * @param  currents: 电流结构体指针，输入alpha,beta，计算d,q
  * @param  angle: 角度 (0-6283)
  * @retval None
  */
void FOC_ParkTransform(Currents* currents, uint16_t angle)
{
    /* Park变换公式：
     * d = α*cosθ + β*sinθ
     * q = -α*sinθ + β*cosθ
     */
    
    /* 获取sin和cos值 (Q10格式) */
    volatile int32_t sin_val = _sin_q10(angle);
    volatile int32_t cos_val = _cos_q10(angle);
    
    /* 计算d轴电流 */
    volatile int32_t d1 = FOC_MulQ10(currents->alpha, cos_val);
    volatile int32_t d2 = FOC_MulQ10(currents->beta, sin_val);
	
    currents->d = 0.9*currents->d+0.1*(d1 + d2);
    
    /* 计算q轴电流 */
    volatile int32_t q1 = FOC_MulQ10(-currents->alpha, sin_val);
    volatile int32_t q2 = FOC_MulQ10(currents->beta, cos_val);
    currents->q = 0.9*currents->q+0.1*(q1 + q2);
}

/**
  * @brief  逆Park变换
  * @param  vd: d轴电压 (Q10格式)
  * @param  vq: q轴电压 (Q10格式)
  * @param  angle: 角度 (0-6283)
  * @param  valpha: 输出的α轴电压指针
  * @param  vbeta: 输出的β轴电压指针
  * @retval None
  */
void FOC_InverseParkTransform(int32_t vd, int32_t vq, uint16_t angle, 
                              int32_t* valpha, int32_t* vbeta)
{
    /* 逆Park变换公式：
     * α = d*cosθ - q*sinθ
     * β = d*sinθ + q*cosθ
     */
    
    /* 获取sin和cos值 (Q10格式) */
    volatile int32_t sin_val = _sin_q10(angle);
    volatile int32_t cos_val = _cos_q10(angle);
    
    /* 计算α轴电压 */
    volatile int32_t alpha1 = FOC_MulQ10(vd, cos_val);
    volatile int32_t alpha2 = FOC_MulQ10(-vq, sin_val);
    *valpha = alpha1 + alpha2;
    
    /* 计算β轴电压 */
    volatile int32_t beta1 = FOC_MulQ10(vd, sin_val);
    volatile int32_t beta2 = FOC_MulQ10(vq, cos_val);
    *vbeta = beta1 + beta2;
}

/**
  * @brief  电流处理函数（完整FOC电流变换）
  * @param  currents: 电流结构体指针，输入a,b相电流，输出所有电流
  * @param  angle: 角度 (0-6283)
  * @retval None
  */
void FOC_CurrentProcessing(Currents* currents, uint16_t angle)
{
    /* Clark变换 */
    FOC_ClarkTransform(currents);
    
    /* Park变换 */
    FOC_ParkTransform(currents, angle);
}
/**
  * @brief  电压矢量输出函数（使用经典的SVPWM算法，Q10格式）
  * @param  vd: d轴电压 (Q10格式，1024=1.0)
  * @param  vq: q轴电压 (Q10格式，1024=1.0)
  * @param  angle_el: 电角度 (0-6283，对应0-2π)
  * @retval PWM占空比 (0-1024)
  */
PWM_Duty FOC_VoltageOutput(int32_t vd, int32_t vq, uint16_t angle_el)
{

    volatile int32_t Uout = 0;
    volatile uint8_t sector = 0;
    
    /* 电压矢量计算 */
    if(vd != 0) 
    {
        /* 如果有d轴电压，计算合成电压矢量 */
        Uout = _sqrt_fast(vd * vd + vq * vq);
        /* 计算电压矢量角度偏移 (使用快速atan2) */
        volatile int32_t angle_offset = fast_atan2_int(vq, vd);
        angle_el = _normalizeAngle(angle_el + angle_offset);
    }
    else
    {
        /* 只有q轴电压 */
        Uout = vq;
        /* 角度偏移90度 (π/2) */
        angle_el = _normalizeAngle(angle_el + _PI_2_1000);
    }
    
    /* 限制输出电压幅值 (Q10格式) 正常最大完全调制圆调制为0.577,最大六边形调制为0.667，因为需要采样电流，所以要先保证至少10us*/
    if(Uout >  U_MAX_LIMIT_Q10) Uout =  U_MAX_LIMIT_Q10;
    if(Uout < -U_MAX_LIMIT_Q10) Uout = -U_MAX_LIMIT_Q10;
    
    /* 计算扇区 (0-5，对应1-6扇区) */
    /* sector = floor(angle_el / (π/3)) */
    sector = (angle_el * 6) / _2PI_1000;  /* angle_el * 6 / 6283 */
    if(sector >= 6) sector = 5;  /* 确保在0-5范围内 */
    
    /* 计算扇区内相对角度 (0-π/3) */
    volatile uint16_t theta = angle_el - sector * _PI_3_1000;
    
    /* 计算基本矢量作用时间 (Q10格式) */
    /* T1 = √3 * sin(π/3 - θ) * Uout / 1024 */
    volatile int32_t sin_pi3_minus_theta = _sin_q10(_PI_3_1000 - theta);
    volatile int32_t T1_temp = (SQRT3_Q10 * sin_pi3_minus_theta) >> 10;  /* 除以1024 */
    volatile int32_t T1 = (T1_temp * Uout) >> 10;  /* 结果Q10格式 */
    
    /* T2 = √3 * sin(θ) * Uout / 1024 */
    volatile int32_t sin_theta = _sin_q10(theta);
    volatile int32_t T2_temp = (SQRT3_Q10 * sin_theta) >> 10;  /* 除以1024 */
    volatile int32_t T2 = (T2_temp * Uout) >> 10;  /* 结果Q10格式 */
    
    /* 计算零矢量作用时间 */
    volatile int32_t T0 = 1024 - T1 - T2;  /* 总时间归一化为1024 (Q10格式) */
    
    /* 预计算中间值 */
    //int32_t T0 >> 1 = T0 >> 1;     /* T0/2 */
    //int32_t T1 + T2 = T1 + T2;       /* T1 + T2 */
    //int32_t T1_T0_half = T1 + T0_half;  /* T1 + T0/2 */
    //int32_t T2_T0_half = T2 + T0_half;  /* T2 + T0/2 */
    
    /* 根据扇区计算占空比 (直接输出0-1024范围) */
    switch(sector)
    {
        case 0:  /* 扇区1: 0-60度 */
            duty.a = T1 + T2 + (T0 >> 1);    /* Ta */
            duty.b = T2 + (T0 >> 1);         /* Tb */
            duty.c = (T0 >> 1);            /* Tc */
            break;
        case 1:  /* 扇区2: 60-120度 */
            duty.a = T1 + (T0 >> 1);         /* Ta */
            duty.b = T1 + T2 + (T0 >> 1);    /* Tb */
            duty.c = (T0 >> 1);            /* Tc */
            break;
        case 2:  /* 扇区3: 120-180度 */
            duty.a = T0 >> 1;            /* Ta */
            duty.b = T1 + T2 + (T0 >> 1);    /* Tb */
            duty.c = T2 + (T0 >> 1);         /* Tc */
            break;
        case 3:  /* 扇区4: 180-240度 */
            duty.a = (T0 >> 1);            /* Ta */
            duty.b = T1 + (T0 >> 1);         /* Tb */
            duty.c = T1 + T2 + (T0 >> 1);    /* Tc */
            break;
        case 4:  /* 扇区5: 240-300度 */
            duty.a = T2 + (T0 >> 1);         /* Ta */
            duty.b = (T0 >> 1);            /* Tb */
            duty.c = T1 + T2 + (T0 >> 1);    /* Tc */
            break;
        case 5:  /* 扇区6: 300-360度 */
            duty.a = T1 + T2 + (T0 >> 1);    /* Ta */
            duty.b = (T0 >> 1);            /* Tb */
            duty.c = T1 + (T0 >> 1);         /* Tc */
            break;
				default:  // possible error state
            duty.a = 0;    /* Ta */
            duty.b = 0;            /* Tb */
            duty.c = 0;         /* Tc */
    }
		
		dutyPeriod.a=0.1*dutyPeriod.a+0.9*3.51f*duty.a;//1024转PWM_Period
		dutyPeriod.b=0.1*dutyPeriod.b+0.9*3.51f*duty.b;
    dutyPeriod.c=0.1*dutyPeriod.c+0.9*3.51f*duty.c;
    /* 确保占空比在0-1024范围内 */
    dutyPeriod.a = (dutyPeriod.a < 0) ? 0 : (dutyPeriod.a > PWM_Period) ? PWM_Period : dutyPeriod.a;
    dutyPeriod.b = (dutyPeriod.b < 0) ? 0 : (dutyPeriod.b > PWM_Period) ? PWM_Period : dutyPeriod.b;
    dutyPeriod.c = (dutyPeriod.c < 0) ? 0 : (dutyPeriod.c > PWM_Period) ? PWM_Period : dutyPeriod.c;
		
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dutyPeriod.a);//8us
	
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, dutyPeriod.b);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, dutyPeriod.c);
    
    return duty;
}

