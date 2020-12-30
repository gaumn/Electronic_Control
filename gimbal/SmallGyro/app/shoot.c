/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
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

#include "shoot.h"
#include "main.h"
#include "cmsis_os.h"
#include "user_lib.h"

#include "can_user.h"
#include "usart.h"

#include "pid.h"
#include "gimbal_task.h"

//#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
//#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
//#define shoot_fric_off()    fric_off()      //�ر�����Ħ����


/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);



/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

shoot_control_t shoot_control;          //�������

/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{
    static const fp32 pluck_angle_pid[3] = {PLUCK_ANGLE_PID_KP, PLUCK_ANGLE_PID_KI, PLUCK_ANGLE_PID_KD};
		static const fp32 pluck_speed_pid[3] = {PLUCK_SPEED_PID_KP, PLUCK_SPEED_PID_KP, PLUCK_SPEED_PID_KD};
		static const fp32 friction_speed_pid[3] = {Friction_SPEED_PID_KP, Friction_SPEED_PID_KI, Friction_SPEED_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
		//Ħ���ֵ��ָ��
    shoot_control.shoot_Friction1_measure = get_Friction1_motor_measure_point();
		shoot_control.shoot_Friction2_measure = get_Friction2_motor_measure_point();
    //���ָ�루�����֣�
    shoot_control.shoot_pluck_measure = get_pluck_motor_measure_point();
		shoot_control.set_angle=shoot_control.shoot_pluck_measure->total_ecd;
    //��ʼ��PID
    PID_init(&shoot_control.pluck_motor_angle_pid, PID_POSITION, pluck_angle_pid, PLUCK_BULLET_PID_MAX_OUT, PLUCK_BULLET_PID_MAX_IOUT);
		PID_init(&shoot_control.pluck_motor_speed_pid, PID_POSITION, pluck_speed_pid, PLUCK_SPEED_PID_MAX_OUT, PLUCK_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control.friction1_motor_speed_pid, PID_DELTA, friction_speed_pid, Friction_SPEED_PID_MAX_OUT, Friction_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control.friction2_motor_speed_pid, PID_DELTA, friction_speed_pid, Friction_SPEED_PID_MAX_OUT, Friction_SPEED_PID_MAX_IOUT);
		//Ħ���ֵ��speed�趨
		shoot_control.friction1_speed_set=Friction_SPEED_SET;
		shoot_control.friction2_speed_set=-Friction_SPEED_SET;
		//��������
    shoot_feedback_update();
}

/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{
    shoot_feedback_update(); //��������

	  shoot_control.speed_set=PID_calc(&shoot_control.pluck_motor_angle_pid,shoot_control.shoot_pluck_measure->total_ecd,shoot_control.set_angle);
    shoot_control.given_current=PID_calc(&shoot_control.pluck_motor_speed_pid,shoot_control.shoot_pluck_measure->speed_rpm,shoot_control.speed_set);

    return shoot_control.given_current;
}


/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{
		if(switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])){
					shoot_control.shoot_mode=SHOOT_STOP;
		}
		if(switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])){
				shoot_control.shoot_mode=SHOOT_READY;
		}
		if(shoot_control.shoot_mode!=SHOOT_STOP){
		//����Ħ����
		  shoot_control.friction1_given_current=PID_calc(&shoot_control.friction1_motor_speed_pid,shoot_control.shoot_Friction1_measure->speed_rpm,shoot_control.friction1_speed_set);
		  shoot_control.friction2_given_current=PID_calc(&shoot_control.friction2_motor_speed_pid,shoot_control.shoot_Friction2_measure->speed_rpm,shoot_control.friction2_speed_set);
			CAN_cmd_gimbal(shoot_control.friction1_given_current,shoot_control.friction2_given_current,0,0);
//			CAN_cmd_gimbal(15000,-15000,0,0);
      }else{
				CAN_cmd_gimbal(0,0,0,0);
			}
		if(shoot_control.shoot_mode==SHOOT_READY &&switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])){
			shoot_control.set_angle=shoot_control.shoot_pluck_measure->total_ecd-8192*3;
			shoot_control.msg_cnt=shoot_control.shoot_pluck_measure->msg_cnt;
			shoot_control.shoot_mode=SHOOT_DONE;
		}
		if(shoot_control.shoot_mode==SHOOT_DONE){
		 if((shoot_control.shoot_pluck_measure->msg_cnt-shoot_control.msg_cnt)>500){
			 shoot_control.shoot_mode=SHOOT_CONTINUE_BULLET;
			 shoot_control.set_angle=shoot_control.shoot_pluck_measure->total_ecd-8192*36;
		 }
		}

		if(switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])&&shoot_control.shoot_mode==SHOOT_READY){
				shoot_control.shoot_mode=SHOOT_STOP;
		}
}


/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
    //ÿ�β��� 1/4PI�ĽǶ�
	  shoot_control.speed_set=PID_calc(&shoot_control.pluck_motor_angle_pid,shoot_control.shoot_pluck_measure->total_ecd,shoot_control.set_angle);
    shoot_control.given_current=PID_calc(&shoot_control.pluck_motor_speed_pid,shoot_control.shoot_pluck_measure->speed_rpm,shoot_control.speed_set);

}
