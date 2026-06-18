#include "xunji.h"
#include "ti_msp_dl_config.h"
#include "board.h"
//直角次数标志位，定义的位全局变量，在每次任务前进行清零
u8 right_angle_flag=0; //直角次数标志位
u8 detecting_right_angle = 0; // 直角检测状态：0-未检测到，1-已检测到正在处理


//float Right_err(void);
/************************传感器误差计算***************************/
// 宏定义（中间为4、5通道，左右对称分布）
#define SENSOR_NUM 8                  // 传感器数量
#define SENSOR_MAX_ERR      50        // 传感器最大误差
#define GYRO_SCALE          3.5f      // 角速度→误差缩放系数
#define FUSION_ALPHA        0.98f     // 互补滤波系数(0.9-0.98)
#define DT                  0.002f    // 控制周期(5ms)
#define SCALE_FACTOR 10                // 缩放因子

// 八路传感器权重（中间4、5为0，左右对称分布，间距1.0单位）
#define L3  -3.0f  // 左3（通道1）
#define L2  -2.0f  // 左2（通道2）
#define L1  -1.0f  // 左1（通道3）
#define M1   0.0f  // 中间1（通道4，误差0）
#define M2   0.0f  // 中间2（通道5，误差0）
#define R1   1.0f  // 右1（通道6）
#define R2   2.0f  // 右2（通道7）
#define R3   3.0f  // 右3（通道8）

/* **************** 结构体 **************** */
typedef struct 
{
    float integrated_err; // 陀螺仪积分误差
    int last_sensor_err;  // 上次传感器误差
} FusionState;

typedef struct 
{
    float position;  // 传感器位置坐标（对称分布）
    float channel;   // 传感器对应的通道号（1-8）
} SensorMap;

/* **************** 全局变量 **************** */
static FusionState fstate = {0, 0};
int i1,i2,i3,i4,i5,i6,i7,i8,err=0;  // 对应8路传感器状态（i4、i5为中间）

// 传感器位置-通道映射（中间4、5，左右对称）
static const SensorMap sensorMap[SENSOR_NUM] = 
{
    {L3, 1},   // 左3（通道1）
    {L2, 2},   // 左2（通道2）
    {L1, 3},   // 左1（通道3）
    {M1, 4},   // 中间1（通道4，误差0）
    {M2, 5},   // 中间2（通道5，误差0）
    {R1, 6},   // 右1（通道6）
    {R2, 7},   // 右2（通道7）
    {R3, 8}    // 右3（通道8）
};

/* ****************计算传感器偏差 **************** */
int Error_Calculate(void)
{		
    static int last_valid_err = 0;
    int active_sum = 0;
    int active_count = 0;
    for (int i = 0; i < SENSOR_NUM; i++) 		
    {
        if (digital(sensorMap[i].channel))  		
        {
            // 中间4、5的position为0，激活时不贡献误差（误差为0）
            active_sum += sensorMap[i].position; 
            active_count++; 						
        }
    }
	
    // 计算误差（中间4、5因position=0，不影响结果）
    err = active_count ? (active_sum * SCALE_FACTOR) / active_count : 0;	
    err = err * Right_err();				// 直角偏差放大
    //printf("err:%d\r\n", err);              // 串口打印误差值
    if (!active_count) {
        err = last_valid_err;
    } else if (err != 0) {
        last_valid_err = err;
    }
    return err;           
}

/* **************** 陀螺仪辅助计算 **************** */
int get_fused_error(int sensor_err, short gyro_z) 
{
    /**** 步骤1：陀螺仪积分处理 ****/
    float delta_err = gyro_z * GYRO_SCALE;
    float dt = 0.01f; 
    fstate.integrated_err += delta_err * dt;
    
    /**** 步骤2：互补滤波融合 ****/
    int fused_err = 0;
    // 中间4或5有信号时，信任传感器（核心修改：中间为4、5通道）
    if (sensor_err != 0 || digital(4) == 1 || digital(5) == 1)  //为1时有信号，检测到黑线
    {			
        fstate.integrated_err = fstate.integrated_err * 0.6f + sensor_err * 0.4f;
        fused_err = (1 - FUSION_ALPHA) * fstate.integrated_err + FUSION_ALPHA * sensor_err;
    } 
    else
    {
        fused_err = fstate.integrated_err;			
    }
   
    /**** 步骤3：限幅处理 ****/
    fused_err = (fused_err > SENSOR_MAX_ERR) ? SENSOR_MAX_ERR : 
               ((fused_err < -SENSOR_MAX_ERR) ? -SENSOR_MAX_ERR : fused_err);

    return (int)fused_err;
}

int get_fused_error_2(int sensor_err, float yaw) 
{
//    /**** 步骤1：陀螺仪积分处理 ****/
//    float delta_err = gyro_z * GYRO_SCALE;
//    float dt = 0.01f; 
//    fstate.integrated_err += delta_err * dt;
    
    /**** 步骤2：互补滤波融合 ****/
      int fused_err = 0;
      // 中间4或5有信号时，信任传感器（核心修改：中间为4、5通道）
      if (sensor_err != 0 || digital(4) == 1 || digital(5) == 1)  //为1时有信号，检测到黑线
     {			
          fstate.integrated_err = (yaw*5)* 0.2f + sensor_err * 0.8f;
          fused_err = (1 - FUSION_ALPHA) * (-yaw*5)+ FUSION_ALPHA * sensor_err;
      } 
     else
     {
          fused_err = (yaw*5);			
      }
	// fused_err = 5*yaw;
    // printf("%d,%.2f\n",fused_err,yaw);	
    /**** 步骤3：限幅处理 ****/
    fused_err = (fused_err > SENSOR_MAX_ERR) ? SENSOR_MAX_ERR : 
               ((fused_err < -SENSOR_MAX_ERR) ? -SENSOR_MAX_ERR : fused_err);
		
    return fused_err;
}


/* **************** 直角放大偏差 **************** *///检测到黑线为低电平，偏向右误差为负，小车向左修正
float Right_err()
{
    // 左直角检测（右侧传感器激活，左侧3个同时信号有效）
    if (digital(1) == 1 && digital(8) == 0 && digital(7) == 0 && digital(6) == 0 )
    {
        if( detecting_right_angle == 0)
        {
            right_angle_flag++;
        }
          // 直角次数加1
        //记录直角次数，在每次执行新的任务时进行清零
        return 1.0f;
    }
    // // 右直角检测（左侧传感器激活，右侧3个同时低电平）
    // else if (digital(1) == 0 && digital(2) == 0 && digital(3) == 0  &&digital(8) == 1 )
    // {  
        
    //     return -35.0f;  // 右直角返回负值

    // }
    else 
    {}
        return 1.0f;    // 正常循迹状态
}
