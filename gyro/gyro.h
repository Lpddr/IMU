
#ifndef CODE_GYRO_GYRO_H_
#define CODE_GYRO_GYRO_H_
#define ZERO_CALC_COUNT  1000

//三轴
typedef enum
{
    X,
    Y,
    Z,
    NUM_XYZ
}imu_info_enum;

//陀螺仪
typedef struct
{

   bool zero_calc_flag;//零漂计算标志

   double angle[NUM_XYZ];//简单积分得到的角度

   double last_angle[NUM_XYZ];//上一次角度

   double zero_angle[NUM_XYZ];//总漂移量

   double gyro[NUM_XYZ];//角速度

   int count;

}gyroscope_info_struct;

//加速度计
typedef struct
{
    double angle[NUM_XYZ];//加速度计得到的角度
    double acc[NUM_XYZ];//角加速度
}acc_info_struct;

//加速度计
typedef struct
{
    double mag[NUM_XYZ];//角加速度
}mag_info_struct;

//imu
typedef struct
{
    gyroscope_info_struct gyro; //陀螺仪
    acc_info_struct acc; //加速度计
    mag_info_struct mag;//磁力计
}imu_info_struct;

//最终角度
typedef struct
{
   double yaw;
   double pitch;
   double roll;
}complementation_info_struct;

//===================================================变量声明===================================================
extern imu_info_struct imu;
extern complementation_info_struct buff_angle;
extern double acc_ratio;      //加速度计比例
extern double gyro_ratio;    //陀螺仪比例
//===================================================变量声明===================================================

//===================================================函数声明===================================================
void get_imu_data(void);
void gyro_get_angle(void);
void acc_get_angle(void);
void leastSquares(int n, float x[], float y[], float* slope, float* intercept) ;
void gyro_data_init(void);
double angle_calc(double acc_angle, double gyro);
//===================================================函数声明===================================================

#endif /* CODE_GYRO_GYRO_H_ */
