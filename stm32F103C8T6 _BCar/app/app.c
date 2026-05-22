#include <stdio.h>
#include "stm32f10x.h"
#include "delay.h"
#include "app.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "oled.h"
#include "my_math.h"
#include "usart.h"
#include "pid.h"
#include "hc-sr04.h"
/*
@简介:主控程序
*/

//进程函数执行时间
#define FALL_CHECK_TIME     5 //倒地检测周期ms
#define OLED_SYNC_TIME      20 // OLED刷新周期ms
#define CONTROL_SYNC_TIME   10 // 主循环周期ms
#define DISTANCE_SYNC_TIME  200 // 超声波测距周期ms
#define LINE_SYNC_TIME      50 // 循迹检测周期ms
//避障距离阈值 
#define DISTANCE_MAX        20.0f //cm
//倒地停机角度阈值
#define FALLEN_ANGLE_back   45.0f
#define FALLEN_ANGLE_fore  -45.0f
/*蓝牙控制*/
#define SPEED_STEP          10.0f  //单次速度增量  （度/s）
#define SPEED_MAX           400.0f //速度限制     （度/s）
/*循迹*/
#define LINE_SPD_MAX        180.0f  //循迹速度限制   （度/s）
#define KP_LINE             15.0f    // 比例系数
#define KD_LINE             5.0f     // 微分系数

static uint8_t obstacle_flag = 0; //避障标志位
static uint8_t falldowm_flag = 0; //倒地标志位
extern volatile uint8_t flag_speed; //速度控制标志位
extern volatile uint8_t flag_turn;  //转向控制标志位
extern volatile uint8_t flag_stop;  //停止标志位

static uint32_t control_time = 0;//主控制时间计数
static uint32_t oled_time = 0;//OLED刷新时间计数
static uint32_t distance_time = 0;//超声波测距时间计数
static uint32_t fall_check_time = 0;//倒地检测时间计数
static uint32_t line_time = 0;//循迹检测时间计数

static float distance = 0.0f; //超声波测得距离
static float speed = 0.0f;//小车当前速度
static float PWM_OUT = 0.0f;//电机PWM输出值、
static float PWM_L = 0.0f;//左电机PWM输出值
static float PWM_R = 0.0f;//右电机PWM输出值

static float Speed_Control_out;//速度环输出
static float Upright_Control_out;//直立环输出
static float Turn_Control_out;//转向环输出
//速度环参数
static float S_Target_Speed = 0.0f; //目标速度
//转向环参数
static float T_Target_Angle = 0.0f; //目标转向角度

//定时回调函数 每1ms调用一次
static void time_callback(void)
{
    if(control_time > 0)
        control_time --;
    if(oled_time > 0)
        oled_time --;
    if(distance_time > 0)
        distance_time --;
    if(fall_check_time > 0)
        fall_check_time --;
    if(line_time > 0)    
        line_time --;
}

//注册定时回调函数
static void callback_init(void)
{
    function_address_passing(time_callback);
}

void app_init(void)
{
    tim_tick_init();//初始化TIM3作为延迟和系统滴答计数
    my_math_init();//快速三角函数
    mpu6050_init();
    oled_init();
    tim_delay_ms(3000);
    mpu6050_calibrate();//校准MPU6050
    Motor_Init();
    USART3_Init();//蓝牙串口初始化
    Encoder_Init();
    callback_init();//注册定时回调函数
    HC_SR04_Init();
    line_gpio_init();//循迹传感器GPIO初始化
}

static void command_control(void)
{
    if(flag_stop == 1)
    {
        if(S_Target_Speed < 0.0f)
        {
            S_Target_Speed += 10.0f; //快速减速停止
            if(S_Target_Speed >= 0.0f)
            {
                S_Target_Speed = 0.0f;
                flag_stop = 0;
            }
        }
        else
        {
            S_Target_Speed -= 10.0f; //快速减速停止
            if(S_Target_Speed <= 0.0f)
            {
                S_Target_Speed = 0.0f;
                flag_stop = 0;
            }
        }
        flag_speed = 0;
        flag_turn = 0;
        return;
    }

    switch(flag_speed)
    {
        case 1: 
            S_Target_Speed += SPEED_STEP;
            if(S_Target_Speed > SPEED_MAX)
                S_Target_Speed = SPEED_MAX;
            break; //前进
        case 2: 
            S_Target_Speed -= SPEED_STEP;
            if(S_Target_Speed < -SPEED_MAX)
                S_Target_Speed = -SPEED_MAX;
            break; //后退
        default:
            break;
        }
    switch(flag_turn)
    {
        case 1: 
            T_Target_Angle += 45.0f; //左转
            flag_turn = 0;
            break;
        case 2: 
            T_Target_Angle -= 45.0f; //右转
            flag_turn = 0;
            break;
        default:
            break;
    }
}

static void fall_check(void)
{
    if(fall_check_time <= 0)
        fall_check_time = FALL_CHECK_TIME;
    else    return;
    if(MPU6050_Angle.pitch > FALLEN_ANGLE_back || MPU6050_Angle.pitch < FALLEN_ANGLE_fore)
    {
        //倒地，停止电机
        Motor_L_speed(0);
        Motor_R_speed(0);
        //重置目标速度和转向角度
        S_Target_Speed = 0.0f;
        T_Target_Angle = 0.0f;
        //等待复位
        falldowm_flag = 1;
    }
    else
    {
        falldowm_flag = 0;
    }
}

static void line_sensor(void)
{
    if(line_time <= 0)
        line_time = LINE_SYNC_TIME;
    else    return;
    static float Line_Error = 0.0f;    // 当前循迹误差
    static float Line_Error_Last = 99.9f;// 上一次的循迹误差 (用于计算D项)
    uint8_t line_state = read_line_sensor();

    switch (line_state) {
        // --- 情况A：在正中间 (误差为0) ---
        case 0b11011: 
        case 0b10001: 
        case 0b10011: 
        case 0b11001: 
            S_Target_Speed += 20.0f; //在正中间时加速
            if(S_Target_Speed > LINE_SPD_MAX)
                S_Target_Speed = LINE_SPD_MAX;
            Line_Error = 0.0f; 
            break;

        // --- 情况B：轻微偏左 (需要向右修正，误差为小正数) ---
        case 0b10111: Line_Error = 1.0f; break; 
        case 0b00011: Line_Error = 1.5f; break; 

        // --- 情况C：中等偏左 (误差为中正数) ---
        case 0b00111: Line_Error = 2.5f; break; 

        // --- 情况D：严重偏左 (快丢线了，误差为大正数) ---
        case 0b01111: Line_Error = 6.0f; break; 

        // --- 情况E：轻微偏右 (误差为小负数) ---
        case 0b11101: Line_Error = -1.0f; break; 
        case 0b11000: Line_Error = -1.5f; break; 

        // --- 情况F：中等偏右 (误差为中负数) ---
        case 0b11100: Line_Error = -2.5f; break; 

        // --- 情况G：严重偏右 (误差为大负数) ---
        case 0b11110: Line_Error = -6.0f; break; 

        case 0b00000:
            Line_Error_Last = Line_Error; 
            break;

        case 0b11111:
            if(Line_Error_Last == 0.0f)
                S_Target_Speed -= 20.0f; //循迹结束停止
            else 
                Line_Error_Last = Line_Error;
             if(S_Target_Speed < 0.0f)
                S_Target_Speed = 0.0f;
            break;
        default:
            break;
    }

    float turn_output = KP_LINE * Line_Error + KD_LINE * (Line_Error - Line_Error_Last);

    // 6. 最终输出 
    T_Target_Angle = turn_output;

    // 7. 更新历史数据
    Line_Error_Last = Line_Error;    
}

static void control(void)//函数执行需要时间4.7ms
{   
    if(control_time <= 0)
        control_time = CONTROL_SYNC_TIME;
    else    return;
    //1、计算欧拉角
    angle_kalman();//卡尔曼滤波
    // mpu6050_process();//互补滤波

    if(falldowm_flag == 1)//如果倒地则不执行控制，等待复位
        return;

    //2、获取编码器速度
    Encoder_Get_Speed();
    speed = (speed_L + speed_R) / 2.0f;//取平均速度作为当前速度
    // 蓝牙控制目标速度
    command_control();
    //3、速度环计算
    Speed_Control_out = Speed_Control(S_Target_Speed, speed);
    //4、直立环计算
    // 注意：pitch 对应陀螺仪的 X 轴（deg/s）
    Upright_Control_out = Upright_Control(Speed_Control_out, MPU6050_Angle.pitch, MPU6050_ReadData.gx);
    //5、转向环计算
    Turn_Control_out = Turn_Control(T_Target_Angle, MPU6050_Angle.yaw, MPU6050_ReadData.gz);
    //6、计算最终PWM输出
    PWM_OUT = Upright_Control_out;
    PWM_L = PWM_OUT + Turn_Control_out;
    PWM_R = PWM_OUT - Turn_Control_out;
    // 7、输出PWM到电机
    Motor_L_speed(PWM_L);
    Motor_R_speed(PWM_R);
}

static void mpu6050_oled(void)//函数执行需要时间5.7ms
{
    char buf[30];
    sprintf(buf, "yaw: %.2f", MPU6050_Angle.yaw);
    oled_write_string(0, 0, buf, 8);
    sprintf(buf, "pitch: %.2f", MPU6050_Angle.pitch);
    oled_write_string(0, 1, buf, 8);
    sprintf(buf, "roll: %.2f",  MPU6050_Angle.roll);
    oled_write_string(0, 2, buf, 8);
}

static void MPU6050_oled_show(void)
{
    if(oled_time <= 0)
        oled_time = OLED_SYNC_TIME;
    else    return;
    mpu6050_oled();
}

static void distance_measure(void)
{
    if(distance_time <= 0)
        distance_time = DISTANCE_SYNC_TIME;
    else    return;

    distance = hc_sr04_distance(); 
    if(distance <= DISTANCE_MAX && obstacle_flag == 0)
    {
        T_Target_Angle += 90.0f; //避障左转
        obstacle_flag = 1;//左转遇到障碍物标志
    }
    else if(distance <= DISTANCE_MAX && obstacle_flag == 1)
    {
        T_Target_Angle -= 90.0f; //右转回到前方继续检测
        obstacle_flag = 2;//右转遇到障碍物标志
    }
    else if(distance <= DISTANCE_MAX && obstacle_flag == 2)
    {
        T_Target_Angle -= 90.0f; //右转避障
        obstacle_flag = 3;//后退遇到障碍物标志
    }
    else if(distance <= DISTANCE_MAX && obstacle_flag == 3)
    {
        T_Target_Angle -= 90.0f; //右转向后方继续检测
        obstacle_flag = 4;//清除避障标志
    }
    else if(distance > DISTANCE_MAX && obstacle_flag == 4)
    {
        S_Target_Speed = 0.0f; //停止
        obstacle_flag = 0;//清除避障标志
    }
}

void app_loop(void)
{
    fall_check();
    line_sensor();//循迹检测
    control();
    distance_measure();
    MPU6050_oled_show();
}
