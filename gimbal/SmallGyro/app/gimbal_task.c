
#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "user_lib.h"
#include "remote_control.h"
#include "INS_task.h"
#include "pid.h"


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
#if GIMBAL_TEST_MODE
//j-scope ����pid����
static void J_scope_gimbal_test(void);
#endif

//gimbal control data
//��̨���������������
gimbal_control_t gimbal_control;

//motor current 
//���͵ĵ������
 int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;
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

    //wait for all motor online
    //�жϵ���Ƿ�����    
	vTaskDelay(GIMBAL_CONTROL_TIME);   
	gimbal_feedback_update(&gimbal_control);             //��̨���ݷ���
    while (1)
    {
       
        gimbal_feedback_update(&gimbal_control);             //��̨���ݷ���
        gimbal_control_loop(&gimbal_control);                //��̨����PID����

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
        CAN_cmd_chassis(yaw_can_set_current, pitch_can_set_current, 0, 0);

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
    //�������ָ���ȡ
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //����������ָ���ȡ
    init->gimbal_INT_angle_point = get_INS_angle_point();//angle �Ƕ�
    init->gimbal_INT_gyro_point = get_gyro_data_point();//palstance ���ٶ�
    //ң��������ָ���ȡ
    init->gimbal_rc_ctrl = get_remote_control_point();

    //��ʼ��yaw���pid
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_palstance_pid, YAW_PALSTANCE_PID_MAX_OUT, YAW_PALSTANCE_PID_MAX_IOUT);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_angle_pid, PID_POSITION, Yaw_gyro_angle_pid, YAW_GYRO_ANGLE_PID_MAX_OUT, YAW_GYRO_ANGLE_PID_MAX_IOUT);
    //��ʼ��pitch���pid


    gimbal_feedback_update(init);

    init->gimbal_yaw_motor.gyro_angle_set = init->gimbal_yaw_motor.gyro_angle;
    init->gimbal_yaw_motor.motor_gyro_palstance_set = init->gimbal_yaw_motor.motor_gyro_palstance;


//    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
//    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
//    init->gimbal_pitch_motor.motor_gyro_palstance_set = init->gimbal_pitch_motor.motor_gyro_palstance;
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
    feedback_update->gimbal_pitch_motor.gyro_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
    feedback_update->gimbal_yaw_motor.gyro_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
		feedback_update->gimbal_pitch_motor.motor_gyro_palstance = *(feedback_update->gimbal_INT_gyro_point + INS_PITCH_ADDRESS_OFFSET);
		feedback_update->gimbal_yaw_motor.motor_gyro_palstance=*(feedback_update->gimbal_INT_gyro_point + INS_YAW_ADDRESS_OFFSET);

		

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
		control_loop->gimbal_yaw_motor.gyro_angle_set=0;
	  control_loop->gimbal_yaw_motor.motor_gyro_palstance_set=control_loop->gimbal_yaw_motor.motor_gyro_palstance_set=PID_calc(&control_loop->gimbal_yaw_motor.gimbal_motor_gyro_angle_pid,control_loop->gimbal_yaw_motor.gyro_angle,control_loop->gimbal_yaw_motor.gyro_angle_set);
		control_loop->gimbal_yaw_motor.given_current=control_loop->gimbal_yaw_motor.motor_gyro_palstance_set=PID_calc(&control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid,control_loop->gimbal_yaw_motor.motor_gyro_palstance,control_loop->gimbal_yaw_motor.motor_gyro_palstance_set);

    
   
}



//static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
//{
//    if (gimbal_motor == NULL)
//    {
//        return;
//    }

//    //�ǶȻ����ٶȻ�����pid����
//    gimbal_motor->motor_gyro_palstance_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro_palstance);
//    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro_palstance, gimbal_motor->motor_gyro_palstance_set);
//    //����ֵ��ֵ
//    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
//}

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


