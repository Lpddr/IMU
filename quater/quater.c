
#include "zf_common_headfile.h"

/**********************使用说明*************************/
 //默认使用的是imu963
 //如果使用的是imu660将IMU_TYPE宏定义值改为0


/*************************主函数初始化样例*************************/
/*
#if IMU_TYPE
    imu963ra_init();
#else
    imu660ra_init();
#endif

//计算陀螺仪数据漂移
gyroOffsetInit();
*/

/**********************1ms中断调用样例*************************/
//注意陀螺仪数据1ms获取一次
//角度解算5ms解算一次
// **************************** PIT中断函数 ****************************
/*
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

    static int imu_t;

    imu_t++;

    //获取陀螺仪数据
     get_imu_data();

    if(imu_t % 5 == 0){
        imu_t = 0;
        //陀螺仪处理
        imu_task();
    }
}
*/

/******************************变量定义************************************/
//quater_info_struct quater = {1, 0, 0, 0};                //四元数初始化
double q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // 初始位置姿态角为：0、0、0，对应四元数为：1、0、0、0
eulerAngle_info_struct eulerAngle;                            //欧拉角
gyroOffset_info_struct gyroOffset;
accOffset_info_struct accOffset;

double accoffsetx=120,accoffsety=-170,accoffsetz=-18;
double I_ex, I_ey, I_ez;                                  // 误差积分
//double imu_kp= 0.3;   //0.17                                              //加速度计的收敛速率比例增益
//double imu_ki= 0.004;        //0.004                                        //陀螺仪收敛速率的积分增益

//1ms
//double imu_kp= 1.5;   //0.17                                              //加速度计的收敛速率比例增益
//double imu_ki= 0.001;        //0.004                                        //陀螺仪收敛速率的积分增益

//1ms
double imu_kp= 1.5;   //0.17                                              //加速度计的收敛速率比例增益
double imu_ki= 0.0005;        //0.004                                        //陀螺仪收敛速率的积分增益

/******************************变量定义************************************/

//陀螺仪解算姿态任务函数
void imu_task(void)
{
    //get_imu_data();
    acc_get_angle();
    gyro_get_angle();

    //数据处理
    imu_data_deal();

    //解算角度
    Update_Angle();


//    Update_Angle(imu963ra_mag_x/3000.0,imu963ra_mag_y/3000.0,imu963ra_mag_z/3000.0);
}

/*
 * @brief 计算陀螺仪零漂
 * 通过采集一定数据求均值计算陀螺仪零点偏移值。
 * 后可以用 陀螺仪读取的数据 - 零飘值，即可去除零点偏移量。
 */
#if IMU_TYPE
    //陀螺仪偏移量
    void gyroOffsetInit(void)
    {
        gyroOffset.Xdata = 0;
        gyroOffset.Ydata = 0;
        gyroOffset.Zdata = 0;

        for (uint16_t i = 0; i < 200; i++)
        {
            imu963ra_get_gyro();
//            gyroOffset.Xdata += imu963ra_gyro_x;
//            gyroOffset.Ydata += imu963ra_gyro_y;
            gyroOffset.Zdata += imu963ra_gyro_z;
            system_delay_ms(5);
        }
        gyroOffset.Xdata /= 200;
        gyroOffset.Ydata /= 200;
        gyroOffset.Zdata /= 200;
    }
    //加速度计偏移量
    void accOffsetInit(void)
    {
       accOffset.Xdata = 0;
       accOffset.Ydata = 0;
       accOffset.Zdata = 0;

       for (uint16_t i = 0; i < 200; i++)
       {
           imu963ra_get_acc();
           accOffset.Xdata += imu963ra_acc_x;
           accOffset.Ydata += imu963ra_acc_y;
           accOffset.Zdata += imu963ra_acc_z;
           system_delay_ms(5);
       }

       accOffset.Xdata /= 200;
       accOffset.Ydata /= 200;
       accOffset.Zdata /= 200;
    }
#else
    void gyroOffsetInit(void)
    {
        gyroOffset.Xdata = 0;
        gyroOffset.Ydata = 0;
        gyroOffset.Zdata = 0;

        for (uint16_t i = 0; i < 200; ++i)
        {
            imu660ra_get_gyro();
            gyroOffset.Xdata += imu660ra_gyro_x;
            gyroOffset.Ydata += imu660ra_gyro_y;
            gyroOffset.Zdata += imu660ra_gyro_z;

            system_delay_ms(5);
        }

        gyroOffset.Xdata /= 200;
        gyroOffset.Ydata /= 200;
        gyroOffset.Zdata /= 200;
    }
    void accOffsetInit(void)
    {
      accOffset.Xdata = 0;
      accOffset.Ydata = 0;
      accOffset.Zdata = 0;

      for (uint16_t i = 0; i < 200; ++i)
      {
          imu660ra_get_acc();
          accOffset.Xdata += imu660ra_acc_x;
          accOffset.Ydata += imu660ra_acc_y;
          accOffset.Zdata += imu660ra_acc_z;

          system_delay_ms(5);
      }

      accOffset.Xdata /= 200;
      accOffset.Ydata /= 200;
      accOffset.Zdata /= 200;
   }
#endif

//imu数据处理
void imu_data_deal(void)
{
    float alpha = 0.2;  //0.35

#if IMU_TYPE
        //一阶低通滤波，单位g
       imu.acc.acc[X] =  (imu963ra_acc_x - accOffset.Xdata) / ACC_SPL * alpha  + imu.acc.acc[X] * (1 - alpha);
       imu.acc.acc[Y] =  (imu963ra_acc_y - accOffset.Ydata) / ACC_SPL * alpha  + imu.acc.acc[Y] * (1 - alpha);
       imu.acc.acc[Z] =  (imu963ra_acc_z - accOffset.Zdata) / ACC_SPL * alpha  + imu.acc.acc[Z] * (1 - alpha);

       imu.gyro.gyro[X] = ANGLE_TO_RAD((imu963ra_gyro_x - gyroOffset.Xdata) / GYRO_SPL);
       imu.gyro.gyro[Y] = ANGLE_TO_RAD((imu963ra_gyro_y - gyroOffset.Ydata) / GYRO_SPL);
       imu.gyro.gyro[Z] = ANGLE_TO_RAD((imu963ra_gyro_z - gyroOffset.Zdata) / GYRO_SPL);

#else
       //一阶低通滤波，单位g
      imu.acc.acc[X] =  (imu660ra_acc_x - accOffset.Xdata) / ACC_SPL * alpha  + imu.acc.acc[X] * (1 - alpha);
      imu.acc.acc[Y] =  (imu660ra_acc_y - accOffset.Ydata) / ACC_SPL * alpha  + imu.acc.acc[Y] * (1 - alpha);
      imu.acc.acc[Z] =  (imu660ra_acc_z - accOffset.Zdata) / ACC_SPL * alpha  + imu.acc.acc[Z] * (1 - alpha);

      imu.gyro.gyro[X] = ANGLE_TO_RAD((imu660ra_gyro_x - gyroOffset.Xdata) / GYRO_SPL);
      imu.gyro.gyro[Y] = ANGLE_TO_RAD((imu660ra_gyro_y - gyroOffset.Ydata) / GYRO_SPL);
      imu.gyro.gyro[Z] = ANGLE_TO_RAD((imu660ra_gyro_z - gyroOffset.Zdata) / GYRO_SPL);
#endif
}

/**
 * @brief 用互补滤波算法解算陀螺仪姿态(即利用加速度计修正陀螺仪的积分误差)
 * 加速度计对振动之类的噪声比较敏感，长期数据计算出的姿态可信；陀螺仪对振动噪声不敏感，短期数据可信，
 * 但长期使用积分误差严重(内部积分算法放大静态误差)。
 * 因此使用姿态互补滤波，短期相信陀螺仪，长期相信加速度计。
 */
double ex, ey, ez;     // 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差
void Update_Angle(void)
{
    double halfT = 0.5 * delta_T;    // 采样周期一半

    double vx, vy, vz;     // 当前姿态计算得来的重力在三轴上的分量

    double q0q0 = q0 * q0;//先相乘，方便后续计算
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    //double q0q3 = q0 * q3;//未使用
    double q1q1 = q1 * q1;
    //double q1q2 = q1 * q2;//未使用
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;

    // 正常静止状态为-g 反作用力。
    if(imu.acc.acc[X] * imu.acc.acc[Y] * imu.acc.acc[Z] == 0) // 加速度计处于自由落体状态时(此时g = 0)不进行姿态解算，因为会产生分母无穷大的情况
        return;

    // 对加速度数据进行归一化得到单位加速度
    //加速度计<测量>的重力加速度向量(机体坐标系)
    double norm = 1/sqrt(imu.acc.acc[X] *imu.acc.acc[X] + imu.acc.acc[Y]*imu.acc.acc[Y] + imu.acc.acc[Z]* imu.acc.acc[Z]);
    imu.acc.acc[X] = imu.acc.acc[X] * norm;
    imu.acc.acc[Y] = imu.acc.acc[Y] * norm;
    imu.acc.acc[Z] = imu.acc.acc[Z] * norm;

    //陀螺仪积分<估计>重力向量(机体坐标系)
    // 机体坐标系下重力在三个轴上的分量
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //将加速度计获得的重力向量归一化后的值与提取的姿态矩阵的重力向量叉乘获取姿态误差
    ex = imu.acc.acc[Y] * vz - imu.acc.acc[Z]* vy;
    ey = imu.acc.acc[Z] * vx - imu.acc.acc[X]* vz;
    ez = imu.acc.acc[X] * vy - imu.acc.acc[Y]* vx;

    //对误差进行积分
    I_ex += ex;
    I_ey += ey;
    I_ez += ez;

    //将误差输入PID控制器后与陀螺仪测得的角速度相加，修正角速度值
    imu.gyro.gyro[X] = imu.gyro.gyro[X] + imu_kp* ex + imu_ki* I_ex;
    imu.gyro.gyro[Y] = imu.gyro.gyro[Y] + imu_kp* ey + imu_ki* I_ey;
    imu.gyro.gyro[Z] = imu.gyro.gyro[Z] + imu_kp* ez + imu_ki* I_ez;

    // 一阶龙格库塔法求解四元数微分方程，其中halfT为测量周期的1/2
    q0 = q0 + (-q1 * imu.gyro.gyro[X] - q2 * imu.gyro.gyro[Y] - q3 * imu.gyro.gyro[Z]) * halfT;
    q1 = q1 + ( q0 * imu.gyro.gyro[X] + q2 * imu.gyro.gyro[Z] - q3 * imu.gyro.gyro[Y]) * halfT;
    q2 = q2 + ( q0 * imu.gyro.gyro[Y] - q1 * imu.gyro.gyro[Z] + q3 * imu.gyro.gyro[X]) * halfT;
    q3 = q3 + ( q0 * imu.gyro.gyro[Z] + q1 * imu.gyro.gyro[Y] - q2 * imu.gyro.gyro[X]) * halfT;

    // 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，下面算法类似线性代数里的正交变换
    norm = 1/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    eulerAngle.pitch  = RAD_TO_ANGLE(atan2(2 * q2 * q3 + 2* q0 * q1, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3));//-180~180
    eulerAngle.roll   = RAD_TO_ANGLE(asin(2 * q0 * q2 - 2 * q1 * q3));//-90~90
    eulerAngle.yaw    = RAD_TO_ANGLE(atan2(2 * q1 * q2 + 2 * q0 * q3,  q0 * q0 +q1 * q1 - q2 * q2 - q3 * q3));//0~360

}

