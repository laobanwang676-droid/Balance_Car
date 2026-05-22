// #include <math.h>
// #include "kalman.h"
// #include <stdint.h>
// #include "mpu6050.h"
// #include "delay.h"

// #define RAD_TO_DEG 57.295779513082320876798154814105

// extern MPU6050_Data_t MPU6050_ReadData;
// extern MPU6050_Angle_t MPU6050_Angle;

// static uint64_t timer = 0;

// Kalman_t KalmanX = {
//     .Q_angle = 0.001f,//过程噪声方差，衡量系统对真实值变化的响应速度，值越大响应越快，但抖动也越大
//     .Q_bias = 0.003f,//过程噪声方差，衡量系统对偏差变化的响应速度，值越大响应越快，但抖动也越大
//     .R_measure = 0.03f//测量噪声方差，衡量测量值的可信度，值越大表示测量值越不可信，滤波结果更平滑，但响应更慢
// };

// Kalman_t KalmanY = {
//     .Q_angle = 0.001f,
//     .Q_bias = 0.003f,
//     .R_measure = 0.03f,
// };

// // 更新卡尔曼滤波结果，使用全局的 MPU6050_ReadData / MPU6050_Angle
// void Kalman_Update(void)
// {
//     uint64_t now = tim_get_ms();
//     double dt = 0.0;
//     if (timer != 0) {
//         dt = (double)(now - timer) / 1000.0;
//     }
//     timer = now;

//     double ax = MPU6050_ReadData.ax;
//     double ay = MPU6050_ReadData.ay;
//     double az = MPU6050_ReadData.az;
//     double gx = MPU6050_ReadData.gx;
//     double gy = MPU6050_ReadData.gy;

//     double roll;
//     double roll_sqrt = sqrt(ax * ax + az * az);
//     if (roll_sqrt != 0.0)
//     {
//         roll = atan(ay / roll_sqrt) * RAD_TO_DEG;
//     }
//     else
//     {
//         roll = 0.0;
//     }
//     double pitch = atan2(-ax, az) * RAD_TO_DEG;

//     if ((pitch < -90 && KalmanY.angle > 90) || (pitch > 90 && KalmanY.angle < -90))
//     {
//         KalmanY.angle = pitch;
//         MPU6050_Angle.pitch = pitch;
//     }
//     else
//     {
//         MPU6050_Angle.pitch = Kalman_getAngle(&KalmanY, pitch, gy, dt);
//     }

//     double gx_for_roll = gx;
//     if (fabs(MPU6050_Angle.pitch) > 90)
//         gx_for_roll = -gx_for_roll;

//     MPU6050_Angle.roll = Kalman_getAngle(&KalmanX, roll, gx_for_roll, dt);
// }

// // 兼容旧调用，若工程中仍有对 MPU6050_Read_All 的调用，则调用 Kalman_Update
// void MPU6050_Read_All(void)
// {
//     Kalman_Update();
// }
// double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
// {
//     double rate = newRate - Kalman->bias;
//     Kalman->angle += dt * rate;

//     Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
//     Kalman->P[0][1] -= dt * Kalman->P[1][1];
//     Kalman->P[1][0] -= dt * Kalman->P[1][1];
//     Kalman->P[1][1] += Kalman->Q_bias * dt;

//     double S = Kalman->P[0][0] + Kalman->R_measure;
//     double K[2];
//     K[0] = Kalman->P[0][0] / S;
//     K[1] = Kalman->P[1][0] / S;

//     double y = newAngle - Kalman->angle;
//     Kalman->angle += K[0] * y;
//     Kalman->bias += K[1] * y;

//     double P00_temp = Kalman->P[0][0];
//     double P01_temp = Kalman->P[0][1];

//     Kalman->P[0][0] -= K[0] * P00_temp;
//     Kalman->P[0][1] -= K[0] * P01_temp;
//     Kalman->P[1][0] -= K[1] * P00_temp;
//     Kalman->P[1][1] -= K[1] * P01_temp;

//     return Kalman->angle;
// };

