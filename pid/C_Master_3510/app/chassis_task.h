/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "user_lib.h"
#include "can_user.h"
#include "cmsis_os.h"
//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357




//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis 3508 max motor control current
//底盘2006最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 29000.0f


////m3508 rmp change to chassis torque,
////m3508转化成底盘速度(m/s)的比例，
//#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define M3510_MOTOR_CURRENT_SET 10000.0f
#define M3510_MOTOR_TORQUE_SET 100.0f
#define M3510_MOTOR_ECD_SET    2000.0f
//chassis motor torque PID
//底盘电机速度环PID
/** 能用，但力小了点（10000左右）
#define M3510_MOTOR_TORQUE_PID_KP 28.0f
#define M3510_MOTOR_TORQUE_PID_KI 0.03f
#define M3510_MOTOR_TORQUE_PID_KD 0.01f  
**/
/**第二版 ，力(14000-15000)
#define M3510_MOTOR_TORQUE_PID_KP 25.0f
#define M3510_MOTOR_TORQUE_PID_KI 0.05f
**/
/**第二版 ，力(14000-15000)
#define M3510_MOTOR_TORQUE_PID_KP 20.0f
#define M3510_MOTOR_TORQUE_PID_KI 0.5f
**/

#define M3510_MOTOR_TORQUE_PID_KP 5.0f
#define M3510_MOTOR_TORQUE_PID_KI 0.03f
#define M3510_MOTOR_TORQUE_PID_KD 0.01f
#define M3510_MOTOR_TORQUE_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3510_MOTOR_TORQUE_PID_MAX_IOUT 6000.0f

//chassis motor position PID
//底盘电机位置环PID
/**
#define M3510_MOTOR_POSITION_PID_KP 2.0f
#define M3510_MOTOR_POSITION_PID_KI 0.0f
#define M3510_MOTOR_POSITION_PID_KD 1.0f
**/
#define M3510_MOTOR_POSITION_PID_KP 1.0f
#define M3510_MOTOR_POSITION_PID_KI 0.0f
#define M3510_MOTOR_POSITION_PID_KD 0.5f
#define M3510_MOTOR_POSITION_PID_MAX_OUT   800.0f//从320 改成390 ，力大概从15000变成16000左右
#define M3510_MOTOR_POSITION_PID_MAX_IOUT   0.0f



typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 torque;
	fp32 ecd;
	fp32 ecd_set;
  fp32 torque_set;
  int16_t give_voltage;
} chassis_motor_t;

typedef struct
{
  chassis_motor_t motor_chassis[4];          //chassis motor data.底盘电机数据
  pid_type_def motor_torque_pid[4];             //motor torque PID.底盘电机速度pid
  pid_type_def chassis_angle_pid[4];              //follow angle PID.底盘跟随角度pid

} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);



#endif
