#include "foc_config.h"
#include "main.h"
#include "tim.h"
#include "foc_current.h"
#include "foc_as5600.h"
#include "foc_math.h"
#include "foc.h"
#include <stdio.h>   
#include <math.h>  

#define ENCODER_MAX     6283    // AS5600输出0~6283对应0~2π弧度
#define ENCODER_MID     3141    // π
#define VQ_RAMP_STEPS   20      // Vq斜坡步数
#define SETTLE_TIME     50      // 稳定时间（ms）
#define ANGLE_THRESHOLD 50      // 角度接近阈值（0.05弧度）
#define M_PI  3.1415
#define M_2PI 6.2832
#define VQ_MAX 120

// 电机参数结构体
typedef struct {
    uint16_t pole_pairs;        // 极对数
    float zero_offset;          // 编码器零位偏置（弧度）
    uint16_t encoder_resolution;// 编码器分辨率
    uint8_t identification_done;// 识别标志
} Motor_Params_t;

Motor_Params_t motor_params = {0};
/**
 * @brief  ADC零位校准函数
 * @param  adc_buffer: ADC缓冲区指针
 * @param  buffer_size: 缓冲区大小
 * @param  zero_offset: 存储零位偏移值的数组
 * @param  channel_count: 要校准的通道数量
 * @retval 无
 */
void ADC_ZeroCalibration() {
    uint32_t sum[2] = {0};  // 累加和
    uint16_t read_count = 0; // 读取次数
		uint16_t zero_offset[2]={0};
    
    printf("》开始ADC零位校准...\r\n");
    printf("等待100ms稳定...\r\n");
    
    // 1. 延时100ms等待系统稳定
    HAL_Delay(100);
    
    printf("开始采集1000个样本...\r\n");
    
    // 2. 每1ms读取一次，共100次
    for (read_count = 0; read_count < 1000; read_count++) {
        
				// 累加adc_buffer[0]和adc_buffer[1]的值
				sum[0] += adc_buffer[0];
				sum[1] += adc_buffer[1];
				
				// 打印进度
				if ((read_count + 1) % 100 == 0) {
						printf("已采集 %d/1000 个样本\r\n", read_count + 1);
				}
        
        // 等待1ms
        HAL_Delay(1);
    }
    
    // 3. 计算平均值
    zero_offset[0] = (uint16_t)(sum[0] / 1000);
    zero_offset[1] = (uint16_t)(sum[1] / 1000);
    
    printf("ADC零位校准完成！\r\n");
    printf("通道0零位值: %d\r\n", zero_offset[0]);
    printf("通道1零位值: %d\r\n", zero_offset[1]);
    printf("通道0零位电压: %.3fV\r\n", (zero_offset[0] * 3.3f) / 4095.0f);
    printf("通道1零位电压: %.3fV\r\n", (zero_offset[1] * 3.3f) / 4095.0f);
}





// 角度差值计算（考虑周期）
float AngleDifference(float a, float b)
{
    float diff = fabs(a - b);
    if (diff > M_PI)
    {
        diff = 2 * M_PI - diff;
    }
    return diff;
}

// 弧度转角度
float RadToDeg(float rad)
{
    return rad * 180.0f / M_PI;
}

// 识别电机极对数和编码器零位
// 识别电机极对数和编码器零位
void MotorParameterIdentification(void)
{
    // 变量声明部分
    float encoder_angles[4] = {0};      // 存储四次电角度对应的编码器读数（弧度）
    float first_angle = 0;              // 第一次0电角度时的编码器值（弧度）
    float current_angle = 0;            // 当前编码器值（弧度）
    uint16_t current_angle_1000 = 0;    // 当前编码器原始值
    
    // 新增：正转记录数组
    float forward_zero_angles[20] = {0};  // 正转过程中每次指向0电角度的编码器角度
    uint8_t forward_zero_count = 0;       // 正转记录的数量
    
    // 新增：反转记录数组
    float reverse_zero_angles[20] = {0};  // 反转过程中每次指向0电角度的编码器角度
    uint8_t reverse_zero_count = 0;       // 反转记录的数量
    
    uint16_t pole_pairs = 0;            // 极对数
    uint16_t reverse_pole_pairs = 0;    // 反转极对数
    float zero_offset = 0;             // 编码器零位偏置（弧度）
    uint16_t cycle_count = 0;          // 循环次数
    uint16_t step_count = 0;            // 总步数
    float angle_diff = 0;              // 角度差（弧度）
    int16_t vq = 0;                    // 当前Vq值
    
    printf("\r\n===== 开始电机参数识别 =====\r\n");
    printf("注意: 请确保电机可以自由旋转！\r\n");
    printf("编码器读数范围: 0~6283 (0~2π弧度)\r\n");
    printf("3秒后开始 〓 〓 〓\r\n");
    HAL_Delay(1000);
		printf("2秒后开始 〓 〓\r\n");
    HAL_Delay(1000);
		printf("1秒后开始 〓\r\n");
    HAL_Delay(1000);
    
    // 第1步：正转识别
    printf("\r\n[正向旋转识别]\r\n");
    printf("------------------------\r\n");
    
    for (cycle_count = 0; cycle_count < 20; cycle_count++)
    {
        printf("\r\n正转循环 %d:\r\n", cycle_count + 1);
        
        // 步骤1: 指向0电角度
        printf("步骤1: 指向0电角度 (电气角=0)\r\n");
        
        // 缓慢增加Vq到VQ_MAX
        for (vq = 0; vq <= VQ_MAX; vq += 5)
        {
            FOC_VoltageOutput(0, vq, _3PI_2_1000);
            HAL_Delay(5);
        }
        
        HAL_Delay(SETTLE_TIME);
        
        // 读取编码器角度
        current_angle_1000 = AS5600GetAngle_DMA(as5600_angle);
        current_angle = (float)current_angle_1000 / 1000.0f;
        printf("  编码器角度: %.3f rad (%.1f°)\r\n", 
               current_angle, RadToDeg(current_angle));
        printf("  编码器原始值: %d\r\n", as5600_angle);
        
        // 新增：记录正转0电角度位置
        if (forward_zero_count < 20)
        {
            forward_zero_angles[forward_zero_count] = current_angle;
            forward_zero_count++;
        }
        
        if (cycle_count == 0)
        {
            first_angle = current_angle;
            encoder_angles[0] = current_angle;
            printf("  首次0电角度基准: %.3f rad\r\n", first_angle);
        }
        else
        {
            angle_diff = AngleDifference(current_angle, first_angle);
            printf("  与首次角度差: %.3f rad (%.1f°), 阈值: %.3f rad\r\n", 
                   angle_diff, RadToDeg(angle_diff), ANGLE_THRESHOLD/1000.0f);
            
            if (angle_diff < (ANGLE_THRESHOLD/1000.0f))
            {
                pole_pairs = cycle_count;
                printf("\r\n? 检测到极对数: %d\r\n", pole_pairs);
                printf("  正向旋转完成!\r\n");
                break;
            }
        }
        
        // 继续完成其他三个步骤
        for (uint8_t i = 1; i < 4; i++)
        {
            printf("步骤%d: ", i + 1);
            
            int16_t next_angle = 0;
            switch(i)
            {
                case 1: 
                    next_angle = 0;
                    printf("指向π/2电角度\r\n");
                    break;
                case 2: 
                    next_angle = _PI_2_1000;
                    printf("指向π电角度\r\n");
                    break;
                case 3: 
                    next_angle = _PI_1000;
                    printf("指向3π/2电角度\r\n");
                    break;
            }
            
            for (vq = 0; vq <= VQ_MAX; vq += 5)
            {
                FOC_VoltageOutput(0, vq, next_angle);
                HAL_Delay(5);
            }
            
            HAL_Delay(SETTLE_TIME);
            current_angle_1000 = AS5600GetAngle_DMA(as5600_angle);
            printf("  编码器角度: %.3f rad\r\n", (float)current_angle_1000/1000.0f);
            
            for (vq = VQ_MAX; vq > 0; vq -= 20)
            {
                FOC_VoltageOutput(0, vq, next_angle);
                HAL_Delay(5);
            }
        }
        
        step_count++;
        if (step_count > 100)
        {
            printf("\r\n?? 超时! 强制停止识别\r\n");
            break;
        }
    }
    
    // 新增：打印正转0电角度记录
    printf("\r\n[正转0电角度记录]\r\n");
    printf("------------------------\r\n");
    printf("共记录了 %d 个0电角度位置:\r\n", forward_zero_count);
    for (uint8_t i = 0; i < forward_zero_count; i++)
    {
        printf("  循环%d: %.3f rad (%.1f°) 原始值: %d\r\n", 
               i+1, 
               forward_zero_angles[i], 
               RadToDeg(forward_zero_angles[i]),
               (uint16_t)(forward_zero_angles[i] * 1000));
    }
    
    if (pole_pairs == 0 && cycle_count >= 20)
    {
        printf("\r\n? 未检测到完整电周期\r\n");
    }
    
    // 第2步：反转识别
    printf("\r\n[反向旋转验证]\r\n");
    printf("------------------------\r\n");
    
    printf("停止电机...\r\n");
    for (vq = VQ_MAX; vq >= 0; vq -= 20)
    {
        FOC_VoltageOutput(0, vq, _3PI_2_1000);
        HAL_Delay(10);
    }
    FOC_VoltageOutput(0, 0, 0);
    HAL_Delay(1000);
    
    // 重置循环计数
    cycle_count = 0;
    reverse_zero_count = 0;
    
    for (cycle_count = 0; cycle_count < 20; cycle_count++)
    {
        printf("\r\n反转验证循环 %d:\r\n", cycle_count + 1);
        
        // 步骤1: 指向0电角度
        printf("步骤1: 指向0电角度\r\n");
        
        for (vq = 0; vq <= VQ_MAX; vq += 5)
        {
            FOC_VoltageOutput(0, vq, _3PI_2_1000);
            HAL_Delay(5);
        }
        
        HAL_Delay(SETTLE_TIME);
        current_angle_1000 = AS5600GetAngle_DMA(as5600_angle);
        current_angle = (float)current_angle_1000 / 1000.0f;
        printf("  编码器角度: %.3f rad\r\n", current_angle);
        
        // 新增：记录反转0电角度位置
        if (reverse_zero_count < 20)
        {
            reverse_zero_angles[reverse_zero_count] = current_angle;
            reverse_zero_count++;
        }
        
        for (vq = VQ_MAX; vq > 0; vq -= 20)
        {
            FOC_VoltageOutput(0, vq, _3PI_2_1000);
            HAL_Delay(5);
        }
        
        if (cycle_count == 0)
        {
            first_angle = current_angle;
        }
        else
        {
            angle_diff = AngleDifference(current_angle, first_angle);
            printf("  与首次角度差: %.3f rad, 阈值: %.3f rad\r\n", 
                   angle_diff, ANGLE_THRESHOLD/1000.0f);
            
            if (angle_diff < (ANGLE_THRESHOLD/1000.0f))
            {
                reverse_pole_pairs = cycle_count;
                printf("\r\n? 反向检测到极对数: %d\r\n", reverse_pole_pairs);
                break;
            }
        }
        
        for (uint8_t i = 1; i < 4; i++)
        {
            printf("步骤%d: ", i + 1);
            
            int16_t next_angle = 0;
            switch(i)
            {
                case 1: 
                    next_angle = _PI_1000;
                    printf("指向3π/2电角度\r\n");
                    break;
                case 2: 
                    next_angle = _PI_2_1000;
                    printf("指向π电角度\r\n");
                    break;
                case 3: 
                    next_angle = 0;
                    printf("指向π/2电角度\r\n");
                    break;
            }
            
            for (vq = 0; vq <= VQ_MAX; vq += 5)
            {
                FOC_VoltageOutput(0, vq, next_angle);
                HAL_Delay(5);
            }
            
            HAL_Delay(SETTLE_TIME);
            current_angle_1000 = AS5600GetAngle_DMA(as5600_angle);
            printf("  编码器角度: %.3f rad\r\n", (float)current_angle_1000/1000.0f);
            
            for (vq = VQ_MAX; vq > 0; vq -= 20)
            {
                FOC_VoltageOutput(0, vq, next_angle);
                HAL_Delay(5);
            }
        }
    }
    
    // 新增：打印反转0电角度记录
    printf("\r\n[反转0电角度记录]\r\n");
    printf("------------------------\r\n");
    printf("共记录了 %d 个0电角度位置:\r\n", reverse_zero_count);
    for (uint8_t i = 0; i < reverse_zero_count; i++)
    {
        printf("  循环%d: %.3f rad (%.1f°) 原始值: %d\r\n", 
               i+1, 
               reverse_zero_angles[i], 
               RadToDeg(reverse_zero_angles[i]),
               (uint16_t)(reverse_zero_angles[i] * 1000));
    }
    
    // 第3步：验证结果
    printf("\r\n===== 识别结果 =====\r\n");
    
    if (pole_pairs > 0 && pole_pairs == reverse_pole_pairs)
    {
        motor_params.pole_pairs = pole_pairs;
        motor_params.encoder_resolution = ENCODER_MAX;
        motor_params.identification_done = 1;
        
        zero_offset = encoder_angles[0];
        motor_params.zero_offset = zero_offset;
        
        printf("? 识别成功!\r\n");
        printf("------------------------\r\n");
        printf("电机极对数: %d\r\n", pole_pairs);
        printf("编码器零位偏置: %.3f rad (%.1f°)\r\n", 
               zero_offset, RadToDeg(zero_offset));
        printf("编码器零位原始值: %d\r\n", (uint16_t)(zero_offset * 1000));
        
        // 比较正反转对应位置的差异
        if (forward_zero_count == reverse_zero_count && forward_zero_count > 1)
        {
            printf("\r\n[正反转对应点角度]\r\n");
            printf("------------------------\r\n");
            for (uint8_t i = 0; i < forward_zero_count; i++)  // 从第2个点开始比较
            {
                float diff = AngleDifference(forward_zero_angles[i], reverse_zero_angles[pole_pairs-i]);
							printf("  第%d个点:正转0角度：%.3f rad，反转0角度：%.3f rad，差异: %.3f rad (%.1f°)\r\n", 
                       i+1,forward_zero_angles[i], reverse_zero_angles[pole_pairs-i],diff, RadToDeg(diff));
            }
        }
				
				// 新增：余数计算（嵌入在函数中）

        printf("\r\n[余数计算分析]\r\n");
        printf("========================\r\n");
        
        const float DIVISOR = M_2PI / 7.0f;
        float forward_remainder_sum = 0;
        float reverse_remainder_sum = 0;
        float forward_remainder_avg = 0;
        float reverse_remainder_avg = 0;
        float final_remainder_avg = 0;
        
        // 计算正转余数
        printf("[正转余数]\r\n");
        for (uint8_t i = 0; i < forward_zero_count; i++)
        {
            float remainder = fmod(forward_zero_angles[i], DIVISOR);
            if (remainder < 0) remainder += DIVISOR;
            forward_remainder_sum += remainder;
            printf("  点%d: %.6f rad 余数: %.6f rad\r\n", 
                   i+1, forward_zero_angles[i], remainder);
        }
        forward_remainder_avg = forward_remainder_sum / forward_zero_count;
        printf("  平均值: %.6f rad\r\n", forward_remainder_avg);
        
        // 计算反转余数
        printf("\r\n[反转余数]\r\n");
        for (uint8_t i = 0; i < reverse_zero_count; i++)
        {
            float remainder = fmod(reverse_zero_angles[i], DIVISOR);
            if (remainder < 0) remainder += DIVISOR;
            reverse_remainder_sum += remainder;
            printf("  点%d: %.6f rad 余数: %.6f rad\r\n", 
                   i+1, reverse_zero_angles[i], remainder);
        }
        reverse_remainder_avg = reverse_remainder_sum / reverse_zero_count;
        printf("  平均值: %.6f rad\r\n", reverse_remainder_avg);
        
        // 计算最终平均值
        final_remainder_avg = (forward_remainder_avg + reverse_remainder_avg) / 2.0f;
        printf("\r\n[最终结果]\r\n");
        printf("  正转余数平均值: %.6f rad (%.3f°)\r\n", 
               forward_remainder_avg, RadToDeg(forward_remainder_avg));
        printf("  反转余数平均值: %.6f rad (%.3f°)\r\n", 
               reverse_remainder_avg, RadToDeg(reverse_remainder_avg));
        printf("  最终平均值（零位）: %.6f rad (%.3f°，%.0f)\r\n", 
               final_remainder_avg, RadToDeg(final_remainder_avg),final_remainder_avg*1000);
        printf("========================\r\n");
				if(forward_zero_angles[0]<forward_zero_angles[1])printf("电机正转方向和编码器：相同\r\n");
				else printf("电机正转方向和编码器：相反\r\n");
				printf("0电角度对应AS5600的零位值: %.0f\r\n", final_remainder_avg*1000);
				printf("极对数: %d\r\n", pole_pairs);
				if(forward_zero_angles[0]<forward_zero_angles[1])printf("#define AS5600DIR 1\r\n");
				else printf("#define AS5600_DIR -1\r\n");
				printf("#define AS5600_OFFSET %.0f\r\n",final_remainder_avg*1000);
				printf("#define MOTOR_POLE_PAIR %d\r\n",pole_pairs);
				
				
       
        
				
				
				
				
    }
    else
    {
        printf("? 识别失败!\r\n");
        printf("正向检测极对数: %d\r\n", pole_pairs);
        printf("反向检测极对数: %d\r\n", reverse_pole_pairs);
        
        if (pole_pairs != reverse_pole_pairs)
        {
            printf("正反转检测结果不一致!\r\n");
        }
    }
    
    printf("\r\n停止电机...\r\n");
    for (vq = VQ_MAX; vq >= 0; vq -= 20)
    {
        FOC_VoltageOutput(0, vq, _3PI_2_1000);
        HAL_Delay(10);
    }
    FOC_VoltageOutput(0, 0, 0);
    
    printf("===== 识别完成 =====\r\n\r\n");
}



void foc_config_main(){
	HAL_Delay(100);
	if(!adc_buffer[0]||!adc_buffer[1]){
		while(1){
			printf("ADC的读数异常，请重启%d,%d\r\n",adc_buffer[0],adc_buffer[1]);
		}
	}
	if(!AS5600GetAngle_DMA(as5600_angle)){
		while(1){
			printf("编码器的读数异常，请重启%d\r\n",as5600_angle);
		}
	}
	// PA6
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_Delay(100);
	ADC_ZeroCalibration();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	
	
	MotorParameterIdentification();
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	
}

//代码示例，放在初始化之后
//	//我的代码开始

//	HAL_Delay(1000);
//	AS5600_Read_Angle();  // 启动第一次读取
//	HAL_Delay(100);

//	
//	// 启动TIM2的4个PWM通道（用于三相逆变器控制）
//	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);


//	// 启动ADC DMA转换
//	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE * 2) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_Period-10);
//	
//	foc_config_main();
//	while(1){
//	
//	}

