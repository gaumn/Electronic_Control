#ifndef  _CAN_USER_H
#define  _CAN_USER_H


#include "stm32f4xx_hal.h"
#include "can.h"
#include "gpio.h"
typedef enum
{

	CAN_2006Moto_ALL_ID = 0x200,
	CAN_2006Moto1_ID = 0x201,
	CAN_2006Moto2_ID = 0x202,
	CAN_2006Moto3_ID = 0x203,
	CAN_2006Moto4_ID = 0x204,
	
}CAN_Message_ID;
#define FILTER_BUF_LEN		5

typedef struct
{
	  int16_t last_speed_rpm;
    int16_t speed_rpm;
	
//    int16_t given_current;
	  int16_t last_current;
	  int16_t current;
	
    uint8_t temperate;
	  uint16_t ecd;
    int16_t last_ecd;
	  int16_t round_cnt;
	
	  int32_t total_ecd;
	  int32_t last_total_ecd;
	  int16_t offset_ecd;
    int32_t	msg_cnt;
	
}motor_measure_t;

/* Extern  ------------------------------------------------------------------*/
extern motor_measure_t  motor_chassis[];

void vApp_User_CAN_Configuration(void);
void vApp_CAN_TxHeader_Init(CAN_TxHeaderTypeDef    * pHeader,
                                                        uint32_t                             StdId, 
                                                        uint32_t                             ExtId, 
                                                        uint32_t                             IDE, 
                                                        uint32_t                             RTR, 
                                                        uint32_t                             DLC);
void vApp_CAN_Filter_Init(CAN_FilterTypeDef * pFilter,
                                                    uint32_t IdHigh,
                                                    uint32_t IdLow,
                                                    uint32_t MaskIdHigh,
                                                    uint32_t MaskIdLow,
                                                    uint32_t FIFOAssignment,
                                                    uint32_t Bank,
                                                    uint32_t Mode,
                                                    uint32_t Scale,
                                                    uint32_t Activation,
                                                    uint32_t SlaveStartFilterBank);
void vApp_CAN_Configuration(CAN_HandleTypeDef *hcan,CAN_TxHeaderTypeDef    * pTxHeader,
                                                        CAN_FilterTypeDef     * pFilter,
                                                        uint32_t                             StdId, 
                                                        uint32_t                             ExtId, 
                                                        uint32_t                             IDE, 
                                                        uint32_t                             RTR, 
                                                        uint32_t                             DLC,
                                                        uint32_t                             IdHigh,
                                                        uint32_t                             IdLow,
                                                        uint32_t                             MaskIdHigh,
                                                        uint32_t                             MaskIdLow,
                                                        uint32_t                             FIFOAssignment,
                                                        uint32_t                             Bank,
                                                        uint32_t                             Mode,
                                                        uint32_t                             Scale,
                                                        uint32_t                             Activation,
                                                        uint32_t                             SlaveStartFilterBank);
void get_moto_offset(motor_measure_t* ptr, uint8_t data[]);
void get_motor_measure(motor_measure_t *ptr, uint8_t data[]);
void set_motor_current(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef *pTxHeader, uint16_t iq1, uint16_t iq2, uint16_t iq3, uint16_t iq4);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif
