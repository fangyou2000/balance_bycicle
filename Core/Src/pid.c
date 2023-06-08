//PID控制
#include <stdio.h>
// 定义 PD 控制器参数
#define KP -4.0f
#define KD 0.005f

// 定义预设角度和速度
#define SETPOINT_ANGLE 0.0f
#define SETPOINT_SPEED 100 
#define mid_angle 90.0f ; 
// 计算PD控制器输出
float calculate_pd_output(float current_angle,float dt) {
    static float last_angle = 0.0f; //定义上次角度
    float error, derivative;
    error = current_angle - SETPOINT_ANGLE ;
    // 计算微分
    derivative = (current_angle - last_angle) / dt;//使用离散状态下计算角速度
    // 更新保存角度
    last_angle = current_angle;
    // PD控制器输出舵机转向角度
    return (int)KP * error + KD * derivative;
}

// 控制主函数

//roll为倾向角
//zuo +
int pd_control(float roll) {
		static float angle = 0.0f;
		int output = 0; 
    if(roll > 100.0)
		{
				angle = 180.0 - roll;
		    output =  calculate_pd_output(angle,0.05f); 

		}
		else if(roll < -100.0){
				angle = -180.0 - roll;
				output =  calculate_pd_output(angle,0.05f); 
		}
		printf("%f    ",angle);
		printf("%d\n",output);
			//换算成舵机转动PWM
				int servo_pulse = 1200 + output*100/9; 
    
				return servo_pulse;
}



//清除定时器初始化过程中的更新中断标志，避免定时器一启动就中断
//_HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
//使能定时器3更新中断并启动定时器

