/*
 * gyro.c
 *
 *  Created on: 2023年11月14日
 *      Author: 33738
 */
#include "zf_common_headfile.h"
imu_info_struct imu;
complementation_info_struct buff_angle;

void gyro_data_init(void)
{
    imu.gyro.count = 0;
    imu.gyro.zero_calc_flag = true;
}

void get_imu_data(void)
{
#if IMU_TYPE
    imu963ra_get_acc();         // 获取   963的加速度测量数值
    imu963ra_get_gyro();        // 获取963的角速度测量数值
    //imu963ra_get_mag();         // 获取963的磁力计测量数值

#else
    imu660ra_get_acc();         // 获取963的加速度测量数值
    imu660ra_get_gyro();        // 获取963的角速度测量数值
#endif
}

//660ra
//陀螺仪获取角度
void gyro_get_angle(void)
{
#if IMU_TYPE
        // 简单积分
        imu.gyro.angle[Z] += imu963ra_gyro_z /  GYRO_SPL * 0.005;
        imu.gyro.angle[Y] += imu963ra_gyro_y /  GYRO_SPL * 0.005;
        imu.gyro.angle[X] += imu963ra_gyro_x /  GYRO_SPL* 0.005;
#else
        // 简单积分
        imu.gyro.angle[Z] += imu660ra_gyro_z / GYRO_SPL* 0.005;
        imu.gyro.angle[Y] += imu660ra_gyro_y / GYRO_SPL* 0.005;
        imu.gyro.angle[X] += imu660ra_gyro_x / GYRO_SPL* 0.005;
#endif


    if (imu.gyro.zero_calc_flag) // 计算零漂标志
    {
        // 计算零漂
        if (imu.gyro.count < ZERO_CALC_COUNT) // 零漂计算次数
        {
            imu.gyro.zero_angle[Z] += imu.gyro.angle[Z] - imu.gyro.last_angle[Z];
            imu.gyro.last_angle[Z] = imu.gyro.angle[Z];

            imu.gyro.zero_angle[Y] += imu.gyro.angle[Y] - imu.gyro.last_angle[Y];
            imu.gyro.last_angle[Y] = imu.gyro.angle[Y];

            imu.gyro.zero_angle[X] += imu.gyro.angle[X] - imu.gyro.last_angle[X];
            imu.gyro.last_angle[X] = imu.gyro.angle[X];

            imu.gyro.count++;
        }
        // 算出零漂
        else
        {
            imu.gyro.angle[Z] -= imu.gyro.zero_angle[Z];
            imu.gyro.zero_angle[Z] /= ZERO_CALC_COUNT;

            imu.gyro.angle[Y] -= imu.gyro.zero_angle[Y];
            imu.gyro.zero_angle[Y] /= ZERO_CALC_COUNT;

            imu.gyro.angle[X] -= imu.gyro.zero_angle[X];
            imu.gyro.zero_angle[X] /= ZERO_CALC_COUNT;

            imu.gyro.zero_calc_flag = false;
            imu.gyro.count = 0;
        }
    }
    // 减去零漂
    else
    {
        imu.gyro.angle[Z] -= imu.gyro.zero_angle[Z];
        imu.gyro.angle[Y] -= imu.gyro.zero_angle[Y];
        imu.gyro.angle[X] -= imu.gyro.zero_angle[X];
    }
}

//加速计获取角度
void acc_get_angle(void)
{
#if IMU_TYPE
        double acc_x = imu963ra_acc_x / ACC_SPL;
        double acc_y = imu963ra_acc_y / ACC_SPL;
        double acc_z = imu963ra_acc_z / ACC_SPL;

        imu.acc.angle[X]= RAD_TO_ANGLE(atan( acc_y / acc_z));
        imu.acc.angle[Y]= -RAD_TO_ANGLE(atan( acc_x / sqrt(acc_y * acc_y +acc_z * acc_z)));
#else
        double acc_x = imu660ra_acc_x / ACC_SPL;
        double acc_y = imu660ra_acc_y / ACC_SPL;
        double acc_z = imu660ra_acc_z / ACC_SPL;

        imu.acc.angle[X] = RAD_TO_ANGLE (atan( acc_y / acc_z));
        imu.acc.angle[Y] = RAD_TO_ANGLE (atan(acc_x / sqrt(acc_y * acc_y +acc_z * acc_z)));
#endif

}

//逐飞方案
//----------------------------------------------------------------

//  @brief      一阶互补滤波

//  @param      angle_m     加速度计数据

//  @param      gyro_m      陀螺仪数据

//  @return     float       数据融合后的角度

//----------------------------------------------------------------
double acc_ratio =0.5;      //加速度计比例
double gyro_ratio = 0.5;    //陀螺仪比例
double angle_calc(double acc_angle, double gyro)
{
    static uint8_t fisrt_flag=0;//第一次运行标志
    double buff_angle;//互补后角度
    double gyro_now;  //当前角速度
    double error_angle;

    static double last_angle;
    if(!fisrt_flag)//判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        fisrt_flag = 1;
        last_angle = acc_angle;
    }
    gyro_now = gyro * gyro_ratio;       //陀螺仪比例
    //在短时间内，可以将角度的变化量近似看作是角速度。这种近似适用于小时间间隔和相对较小的角度变化
    error_angle = (acc_angle - last_angle)*acc_ratio;
    //结合加速度计和陀螺仪的数据，用于更新当前的角度值
    buff_angle = last_angle + (error_angle + gyro_now)*0.005;
    //保存当前角度值
    last_angle = buff_angle;
    return buff_angle;
}

void leastSquares(int n, float x[], float y[], float* slope, float* intercept) {
    double sum_x = 0.0, sum_y = 0.0, sum_x_squared = 0.0, sum_xy = 0.0;

    for (int i = 0; i < n; i++) {
        sum_x += x[i];
        sum_y += y[i];
        sum_x_squared += x[i] * x[i];
        sum_xy += x[i] * y[i];
    }

    double denominator = n * sum_x_squared - sum_x * sum_x;

    if (denominator != 0) {
        *slope = (n * sum_xy - sum_x * sum_y) / denominator;
        *intercept = (sum_y - (*slope) * sum_x) / n;
    }
    else {
        // 若计算斜率时出现除零错误，则设置斜率和截距为无效值
        *slope = 0.0;
        *intercept = 0.0;
    }
}

