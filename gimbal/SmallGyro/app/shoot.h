/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"
#include "can_user.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"



//������俪��ͨ������
#define SHOOT_RC_MODE_CHANNEL       1
//��̨ģʽʹ�õĿ���ͨ��

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME     15
//��곤���ж�
#define PRESS_LONG_TIME             400
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME              2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME                 80
//�����������ֵ��Χ
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//���rmp �仯�� ��ת�ٶȵı���
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
//�����ٶ�
#define TRIGGER_SPEED               10.0f
#define CONTINUE_TRIGGER_SPEED      15.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1


//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f
//Ħ����set
#define Friction_SPEED_SET       8800.0f
//Ħ�����ٶȻ�pid
#define Friction_SPEED_PID_KP       15.0f
#define Friction_SPEED_PID_KI        0.1f
#define Friction_SPEED_PID_KD        0.0f
#define Friction_SPEED_PID_MAX_OUT   16384.0f
#define Friction_SPEED_PID_MAX_IOUT  3000.0f

//�����ֵ��PID�Ƕ�
#define PLUCK_ANGLE_PID_KP        1.0f
#define PLUCK_ANGLE_PID_KI        0.0f
#define PLUCK_ANGLE_PID_KD        0.1f
#define PLUCK_BULLET_PID_MAX_OUT  500.0f
#define PLUCK_BULLET_PID_MAX_IOUT   0.0f
//�����ֵ��PID�ٶ�
#define PLUCK_SPEED_PID_KP        15.0f
#define PLUCK_SPEED_PID_KI        0.1f
#define PLUCK_SPEED_PID_KD        0.0f
#define PLUCK_SPEED_PID_MAX_OUT   10000.0f
#define PLUCK_SPEED_PID_MAX_IOUT  1000.0f

#define PLUCK_READY_PID_MAX_OUT   10000.0f
#define PLUCK_READY_PID_MAX_IOUT  7000.0f


#define SHOOT_HEAT_REMAIN_VALUE     80

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_BULLET,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_pluck_measure;
	  const motor_measure_t *shoot_Friction1_measure;
	  const motor_measure_t *shoot_Friction2_measure;
	
    pid_type_def pluck_motor_angle_pid;
	  pid_type_def pluck_motor_speed_pid;
	  pid_type_def friction1_motor_speed_pid;
		pid_type_def friction2_motor_speed_pid;
    fp32 friction1_speed_set;
	  fp32 friction2_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
		fp32 msg_cnt;
    int16_t given_current;
	  int16_t friction1_given_current;
	  int16_t friction2_given_current;
    int8_t ecd_count;

} shoot_control_t;

//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

#endif
