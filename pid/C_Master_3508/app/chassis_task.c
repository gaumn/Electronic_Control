/**
  ****************************(C) COPYRIGHT 2020****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0    2020.12.13       gaumn             
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020****************************
  */
#include "chassis_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "can_user.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

fp32 real_ecd;
fp32 ecd_set;
fp32 real_rpm;
fp32 rpm_set;		
/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);


/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle�� robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//�����˶�����
chassis_move_t chassis_move;

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
void chassis_task(void const *pvParameters)
{
    //wait a time 
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    //���̳�ʼ��
    chassis_init(&chassis_move);
    while (1)
    {
        
        //chassis data update
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
//        
				CAN_cmd_chassis(15000,-15000,0,0);
//        CAN_cmd_chassis(chassis_move.motor_chassis[0].give_voltage, chassis_move.motor_chassis[1].give_voltage,
//                        chassis_move.motor_chassis[2].give_voltage, chassis_move.motor_chassis[3].give_voltage);  
        //os delay
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
			
    }
}
/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    //chassis motor speed PID
    //���̵�����pidֵ
		const static fp32 motor_speed_pid[3]     = {M6020_MOTOR_SPEED_PID_KP   ,M6020_MOTOR_SPEED_PID_KI   ,M6020_MOTOR_SPEED_PID_KD};
		const static fp32 motor_position_pid[3]  = {M6020_MOTOR_POSITION_PID_KP,M6020_MOTOR_POSITION_PID_KI,M6020_MOTOR_POSITION_PID_KD};
		int i=0;
		
    //chassis motor speed PID
    //�����ٶȻ�pidֵ
   
//    //chassis angle PID
//    //���̽Ƕ�pidֵ
//    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
//    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
//    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
//    uint8_t i;
    
    //��ȡ���̵������ָ�룬��ʼ��PID 
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i],   PID_POSITION,    motor_speed_pid  , M6020_MOTOR_SPEED_PID_MAX_OUT  , M6020_MOTOR_SPEED_PID_MAX_IOUT  );
		    PID_init(&chassis_move_init->chassis_angle_pid[i],PID_POSITION,motor_position_pid, M6020_MOTOR_POSITION_PID_MAX_OUT  , M6020_MOTOR_POSITION_PID_MAX_IOUT);
		}
	
//    //��ȡ���̵������ָ�룬��ʼ��PID 
//    for (i = 0; i < 4; i++)
//    {
//        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
//    }
    //initialize angle PID
    //��ʼ���Ƕ�PID
   // PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //update data
    //����һ������
    chassis_feedback_update(chassis_move_init);
}




static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
		int i=0;
    for (i = 0; i < 4; i++)
    {
			
        //update motor speed, accel is differential of speed PID
        //���µ������,�ٶȣ����ٶ����ٶȵ�PID΢��
		  	chassis_move_update->motor_chassis[i].current = chassis_move_update->motor_chassis[i].chassis_motor_measure->current;
        chassis_move_update->motor_chassis[i].speed = chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
			  chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0]*CHASSIS_CONTROL_TIME;
			  chassis_move_update->motor_chassis[i].ecd = chassis_move_update->motor_chassis[i].chassis_motor_measure->total_ecd;
 
		}
		real_rpm = (chassis_move_update->motor_chassis[0].speed);
		rpm_set= (chassis_move_update->motor_chassis[0].speed_set);
		real_ecd= (chassis_move_update->motor_chassis[0].ecd);
		ecd_set = (chassis_move_update->motor_chassis[0].ecd_set);
		//chassis control pid calculate
		//���̿���PID����
		chassis_control_loop(&chassis_move);
 
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
		int i=0;
    //calculate pid
    //����pid
    for (i = 0; i < 4; i++)
    {
			//  chassis_move_control_loop->motor_chassis[i].current_set=M6020_MOTOR_CURRENT_SET;
			 chassis_move_control_loop->motor_chassis[i].ecd_set=M6020_MOTOR_ECD_SET;
			 chassis_move_control_loop->motor_chassis[i].speed_set=PID_calc(&chassis_move_control_loop->chassis_angle_pid[i], chassis_move_control_loop->motor_chassis[i].ecd, chassis_move_control_loop->motor_chassis[i].ecd_set);
       chassis_move_control_loop->motor_chassis[i].give_voltage=PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set); 
       // chassis_move_control_loop->motor_chassis[i].give_voltage=PID_calc(&chassis_move_control_loop->motor_current_pid[i], chassis_move_control_loop->motor_chassis[i].current, chassis_move_control_loop->motor_chassis[i].current_set);			

		}


//    //��ֵ����ֵ
//    for (i = 0; i < 4; i++)
//    {
//        chassis_move_control_loop->motor_chassis[i].give_voltage = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
//    }
   


}
