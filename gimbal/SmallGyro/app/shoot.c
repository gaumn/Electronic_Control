/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

//#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
//#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
//#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮


/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);



/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

shoot_control_t shoot_control;          //射击数据

/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{
    static const fp32 pluck_angle_pid[3] = {PLUCK_ANGLE_PID_KP, PLUCK_ANGLE_PID_KI, PLUCK_ANGLE_PID_KD};
		static const fp32 pluck_speed_pid[3] = {PLUCK_SPEED_PID_KP, PLUCK_SPEED_PID_KP, PLUCK_SPEED_PID_KD};
		static const fp32 friction_speed_pid[3] = {Friction_SPEED_PID_KP, Friction_SPEED_PID_KI, Friction_SPEED_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
		//摩擦轮电机指针
    shoot_control.shoot_Friction1_measure = get_Friction1_motor_measure_point();
		shoot_control.shoot_Friction2_measure = get_Friction2_motor_measure_point();
    //电机指针（拨弹轮）
    shoot_control.shoot_pluck_measure = get_pluck_motor_measure_point();
		shoot_control.set_angle=shoot_control.shoot_pluck_measure->total_ecd;
    //初始化PID
    PID_init(&shoot_control.pluck_motor_angle_pid, PID_POSITION, pluck_angle_pid, PLUCK_BULLET_PID_MAX_OUT, PLUCK_BULLET_PID_MAX_IOUT);
		PID_init(&shoot_control.pluck_motor_speed_pid, PID_POSITION, pluck_speed_pid, PLUCK_SPEED_PID_MAX_OUT, PLUCK_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control.friction1_motor_speed_pid, PID_DELTA, friction_speed_pid, Friction_SPEED_PID_MAX_OUT, Friction_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control.friction2_motor_speed_pid, PID_DELTA, friction_speed_pid, Friction_SPEED_PID_MAX_OUT, Friction_SPEED_PID_MAX_IOUT);
		//摩擦轮电机speed设定
		shoot_control.friction1_speed_set=Friction_SPEED_SET;
		shoot_control.friction2_speed_set=-Friction_SPEED_SET;
		//更新数据
    shoot_feedback_update();
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{
    shoot_feedback_update(); //更新数据

	  shoot_control.speed_set=PID_calc(&shoot_control.pluck_motor_angle_pid,shoot_control.shoot_pluck_measure->total_ecd,shoot_control.set_angle);
    shoot_control.given_current=PID_calc(&shoot_control.pluck_motor_speed_pid,shoot_control.shoot_pluck_measure->speed_rpm,shoot_control.speed_set);

    return shoot_control.given_current;
}


/**
  * @brief          射击数据更新
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
		//开启摩擦轮
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
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
    //每次拨动 1/4PI的角度
	  shoot_control.speed_set=PID_calc(&shoot_control.pluck_motor_angle_pid,shoot_control.shoot_pluck_measure->total_ecd,shoot_control.set_angle);
    shoot_control.given_current=PID_calc(&shoot_control.pluck_motor_speed_pid,shoot_control.shoot_pluck_measure->speed_rpm,shoot_control.speed_set);

}
