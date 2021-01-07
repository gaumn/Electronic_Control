
#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "user_lib.h"
#include "remote_control.h"
#include "INS_task.h"
#include "pid.h"
#include "shoot.h"

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


//motor enconde value format, range[0-8191]
//�������ֵ���� 0��8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif
static void gimbal_init(gimbal_control_t *init);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
static void gimbal_control_loop(gimbal_control_t *control_loop);
static void gimbal_set_control(gimbal_control_t *set_control);
static void gimbal_set_mode(gimbal_control_t *set_mode);
#if GIMBAL_TEST_MODE
//j-scope ����pid����
static void J_scope_gimbal_test(void);
#endif

//gimbal control data
//��̨���������������
gimbal_control_t gimbal_control;

//motor current 
//���͵ĵ������
 int16_t yaw_can_set_current = 0, pitch_can_set_current = 0,shoot_can_set_current=0;
/**
  * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */

void gimbal_task(void const *pvParameters)
{
    //�ȴ������������������������
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    //��̨��ʼ��
    gimbal_init(&gimbal_control);
	  //�����ʼ��
    shoot_init();
	
   	vTaskDelay(GIMBAL_CONTROL_TIME);   
	  gimbal_feedback_update(&gimbal_control);             //��̨���ݷ���
    while (1)
    {
       
        gimbal_feedback_update(&gimbal_control);             //��̨���ݷ���
			  gimbal_set_mode(&gimbal_control);                    //��̨ģʽ�趨 
			  gimbal_set_control(&gimbal_control);                 //������̨������
        gimbal_control_loop(&gimbal_control);                //��̨����PID����
        shoot_can_set_current = shoot_control_loop();        //����������ѭ��
#if YAW_TURN
        yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
        pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif
        CAN_cmd_chassis(yaw_can_set_current, pitch_can_set_current, shoot_can_set_current, 0);

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif
      
			vTaskDelay(GIMBAL_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

static void gimbal_init(gimbal_control_t *init)
{
		fp32 Pitch_palstance_pid[3] = {PITCH_PALSTANCE_PID_KP, PITCH_PALSTANCE_PID_KI, PITCH_PALSTANCE_PID_KD};
    fp32 Yaw_palstance_pid[3] = {YAW_PALSTANCE_PID_KP, YAW_PALSTANCE_PID_KI, YAW_PALSTANCE_PID_KD};
		fp32 Pitch_gyro_angle_pid[3] = {PITCH_GYRO_ANGLE_PID_KP, PITCH_GYRO_ANGLE_PID_KI, PITCH_GYRO_ANGLE_PID_KD};
    fp32 Yaw_gyro_angle_pid[3] = {YAW_GYRO_ANGLE_PID_KP, YAW_GYRO_ANGLE_PID_KI, YAW_GYRO_ANGLE_PID_KD};
		fp32 Yaw_encode_angle_pid[3] = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD};
	  const static fp32 motor_speed_pid[3]     = {M6020_MOTOR_SPEED_PID_KP   ,M6020_MOTOR_SPEED_PID_KI   ,M6020_MOTOR_SPEED_PID_KD};
		const static fp32 motor_position_pid[3]  = {M6020_MOTOR_POSITION_PID_KP,M6020_MOTOR_POSITION_PID_KI,M6020_MOTOR_POSITION_PID_KD};
    //�������ָ���ȡ
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //����������ָ���ȡ
    init->gimbal_INT_angle_point = get_INS_angle_point();//angle �Ƕ�
    init->gimbal_INT_gyro_point = get_gyro_data_point();//palstance ���ٶ�
    //ң��������ָ���ȡ
    init->gimbal_rc_ctrl = get_remote_control_point();
    //��ʼ��yaw���pid(������ģʽ)
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_encode_angle_pid, PID_POSITION, Yaw_encode_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_OUT);
    //��ʼ��yaw���pid��������ģʽ��
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_palstance_pid, YAW_PALSTANCE_PID_MAX_OUT, YAW_PALSTANCE_PID_MAX_IOUT);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_angle_pid, PID_POSITION, Yaw_gyro_angle_pid, YAW_GYRO_ANGLE_PID_MAX_OUT, YAW_GYRO_ANGLE_PID_MAX_IOUT);
    //��ʼ��pitch���pid
//    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_palstance_pid, PITCH_PALSTANCE_PID_MAX_OUT, PITCH_PALSTANCE_PID_MAX_IOUT);
//    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_angle_pid, PID_POSITION, Pitch_gyro_angle_pid, PITCH_GYRO_ANGLE_PID_MAX_OUT, PITCH_GYRO_ANGLE_PID_MAX_IOUT);
    
		PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid,   PID_POSITION,    motor_speed_pid  , M6020_MOTOR_SPEED_PID_MAX_OUT  , M6020_MOTOR_SPEED_PID_MAX_IOUT  );
		PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_angle_pid,PID_POSITION,motor_position_pid, M6020_MOTOR_POSITION_PID_MAX_OUT  , M6020_MOTOR_POSITION_PID_MAX_IOUT);

    gimbal_feedback_update(init);
    init->gimbal_yaw_motor.encode_angle_set=YAW_ENCODE_MID;
		
    init->gimbal_yaw_motor.gyro_angle_set = init->gimbal_yaw_motor.gyro_angle;
    init->gimbal_yaw_motor.motor_gyro_palstance_set = init->gimbal_yaw_motor.motor_gyro_palstance;
		
//    init->gimbal_pitch_motor.gyro_angle_set = motor_ecd_to_angle_change(init->gimbal_pitch_motor.gimbal_motor_measure->ecd,init->gimbal_pitch_motor.gimbal_motor_measure->offset_ecd);
    init->gimbal_pitch_motor.ecd_set= 2000.0f;


}



/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //��̨���ݸ���
    //��̨yaw��е�Ƕ�ת��
		feedback_update->gimbal_yaw_motor.encode_angle=motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_yaw_motor.offset_ecd);
	  feedback_update->gimbal_yaw_motor.motor_speed=feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
		//motor_ecd_to_angle_change��
    feedback_update->gimbal_yaw_motor.gyro_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
		feedback_update->gimbal_yaw_motor.motor_gyro_palstance=*(feedback_update->gimbal_INT_gyro_point + INS_YAW_ADDRESS_OFFSET);
		//pitch��������
//		feedback_update->gimbal_pitch_motor.gyro_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
//		feedback_update->gimbal_pitch_motor.motor_gyro_palstance = *(feedback_update->gimbal_INT_gyro_point + INS_PITCH_ADDRESS_OFFSET);

    //pitch�ᴿ��е�Ƕ�
    feedback_update->gimbal_pitch_motor.motor_speed=feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
		feedback_update->gimbal_pitch_motor.ecd=feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;
		
//		feedback_update->gimbal_pitch_motor.gyro_angle= motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_pitch_motor.gimbal_motor_measure->offset_ecd);
//		feedback_update->gimbal_pitch_motor.motor_gyro_palstance=feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
//		

}
/***
ģʽ����
**/
static void gimbal_set_mode(gimbal_control_t *set_mode){
  if (set_mode == NULL)
    {
        return;
    }
		
    //���ؿ��� ��̨״̬
    if (switch_is_down(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        set_mode->mode  = GIMBAL_ZERO_FORCE;
    }
    else if (switch_is_mid(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
         set_mode->mode = GIMBAL_RELATIVE_ANGLE;
    }
    else if (switch_is_up(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
         set_mode->mode = GIMBAL_ABSOLUTE_ANGLE;
    }
		
		//������̨��Ϊ״̬�����õ��״̬��
    if ( set_mode->mode == GIMBAL_ZERO_FORCE)
    {
        set_mode->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        set_mode->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (set_mode->mode == GIMBAL_ABSOLUTE_ANGLE)
    {
        set_mode->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        set_mode->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (set_mode->mode == GIMBAL_RELATIVE_ANGLE)
    {
        set_mode->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        set_mode->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
  
}

static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
		if(control_loop->mode==GIMBAL_ABSOLUTE_ANGLE){//��̨������
		
	  control_loop->gimbal_yaw_motor.motor_gyro_palstance_set=PID_calc(&control_loop->gimbal_yaw_motor.gimbal_motor_gyro_angle_pid,control_loop->gimbal_yaw_motor.gyro_angle,control_loop->gimbal_yaw_motor.gyro_angle_set);
		control_loop->gimbal_yaw_motor.given_current=PID_calc(&control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid,control_loop->gimbal_yaw_motor.motor_gyro_palstance,control_loop->gimbal_yaw_motor.motor_gyro_palstance_set);
		}
		if(control_loop->mode==GIMBAL_RELATIVE_ANGLE){//��̨������ģʽ
	  control_loop->gimbal_yaw_motor.motor_gyro_palstance_set=PID_calc(&control_loop->gimbal_yaw_motor.gimbal_motor_encode_angle_pid,control_loop->gimbal_yaw_motor.encode_angle,control_loop->gimbal_yaw_motor.encode_angle_set);
		control_loop->gimbal_yaw_motor.given_current=PID_calc(&control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid,control_loop->gimbal_yaw_motor.motor_gyro_palstance,control_loop->gimbal_yaw_motor.motor_gyro_palstance_set);
		}
//	  control_loop->gimbal_pitch_motor.motor_gyro_palstance_set=PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_angle_pid,control_loop->gimbal_pitch_motor.gyro_angle,control_loop->gimbal_pitch_motor.gyro_angle_set);
//		control_loop->gimbal_pitch_motor.given_current=PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid,control_loop->gimbal_pitch_motor.motor_gyro_palstance,control_loop->gimbal_pitch_motor.motor_gyro_palstance_set);
		
    control_loop->gimbal_pitch_motor.speed_set=PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_angle_pid,control_loop->gimbal_pitch_motor.ecd,control_loop->gimbal_pitch_motor.ecd_set);
    control_loop->gimbal_pitch_motor.given_current=PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid, control_loop->gimbal_pitch_motor.motor_speed,control_loop->gimbal_pitch_motor.speed_set); 
   
}

/**
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

		if(set_control->mode==GIMBAL_ZERO_FORCE){
			set_control->gimbal_yaw_motor.gyro_angle_set=set_control->gimbal_yaw_motor.gyro_angle;
			set_control->gimbal_yaw_motor.encode_angle_set=set_control->gimbal_yaw_motor.encode_angle;
		}
		if(set_control->mode==GIMBAL_RELATIVE_ANGLE){
			set_control->gimbal_yaw_motor.gyro_angle_set=set_control->gimbal_yaw_motor.gyro_angle;
			set_control->gimbal_yaw_motor.encode_angle_set=YAW_ENCODE_MID;
//			add_yaw_angle = yaw_channel * YAW_RC_SEN - set_control->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
//			set_control->gimbal_yaw_motor.encode_angle_set+=add_yaw_angle;
//      set_control->gimbal_yaw_motor.gyro_angle_set=set_control->gimbal_yaw_motor.gyro_angle;
//			if(set_control->gimbal_yaw_motor.encode_angle>3.2 ||set_control->gimbal_yaw_motor.encode_angle<-3.2){
//			set_control->gimbal_yaw_motor.encode_angle_set=set_control->gimbal_yaw_motor.encode_angle;
//			}	
		}
		if(set_control->mode==GIMBAL_ABSOLUTE_ANGLE){
			set_control->gimbal_yaw_motor.encode_angle_set=set_control->gimbal_yaw_motor.encode_angle;
		}
		add_pitch_angle = pitch_channel * PITCH_RC_SEN + set_control->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
	  set_control->gimbal_pitch_motor.ecd_set-=add_pitch_angle;
		if(set_control->gimbal_pitch_motor.ecd_set>=PITCH_ENCODE_MAX){
			set_control->gimbal_pitch_motor.ecd_set=PITCH_ENCODE_MAX;
		}
		if(set_control->gimbal_pitch_motor.ecd_set<=PITCH_ENCODE_MIN){
				set_control->gimbal_pitch_motor.ecd_set=PITCH_ENCODE_MIN;
				
		}
		
}


#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_palstance * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_palstance_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_palstance * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_palstance_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif


