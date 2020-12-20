/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357




//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis 3508 max motor control current
//����2006���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 30000.0f


////m3508 rmp change to chassis speed,
////m3508ת���ɵ����ٶ�(m/s)�ı�����
//#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define M6020_MOTOR_CURRENT_SET 10000.0f
#define M6020_MOTOR_SPEED_SET 100.0f
#define M6020_MOTOR_ECD_SET    6000.0f
//chassis motor speed PID
//���̵���ٶȻ�PID
#define M6020_MOTOR_SPEED_PID_KP 2.25f
#define M6020_MOTOR_SPEED_PID_KI 0.03f
#define M6020_MOTOR_SPEED_PID_KD 0.0f
#define M6020_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 3000.0f

//chassis motor position PID
//���̵��λ�û�PID
#define M6020_MOTOR_POSITION_PID_KP 10.0f
#define M6020_MOTOR_POSITION_PID_KI 0.0f
#define M6020_MOTOR_POSITION_PID_KD 0.5f
#define M6020_MOTOR_POSITION_PID_MAX_OUT   130.0f
#define M6020_MOTOR_POSITION_PID_MAX_IOUT   20.0f



typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
	fp32 ecd;
	fp32 current;
	fp32 ecd_set;
  fp32 speed_set;
	fp32 current_set;
  int16_t give_voltage;
} chassis_motor_t;

typedef struct
{
  chassis_motor_t motor_chassis[4];          //chassis motor data.���̵������
	pid_type_def  motor_current_pid[4];        //motor current PID.���̵������pid
  pid_type_def motor_speed_pid[4];             //motor speed PID.���̵���ٶ�pid
  pid_type_def chassis_angle_pid[4];              //follow angle PID.���̸���Ƕ�pid

} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);



#endif
