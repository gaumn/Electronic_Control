#include "can_user.h"
#include "usart.h"

CAN_TxHeaderTypeDef hCAN1_TxHeader; //CAN1发送消息
CAN_RxHeaderTypeDef	hCAN1_RxHeader;      	//CAN1接收消息
CAN_FilterTypeDef   hCAN1_Filter;					//CAN1滤波器
CAN_TxHeaderTypeDef hCAN2_TxHeader; //CAN2发送消息
CAN_RxHeaderTypeDef	hCAN2_RxHeader;      	//CAN2接收消息
CAN_FilterTypeDef   hCAN2_Filter;					//CAN2滤波器
char flag=0;
motor_measure_t motor_chassis[7];//chassis motor
CAN_TxHeaderTypeDef  gimbal_tx_message;
uint8_t              gimbal_can_send_data[8];
CAN_TxHeaderTypeDef  chassis_tx_message;
uint8_t              chassis_can_send_data[8];

/*******************************************************************************
* Function Name  : vApp_CAN_TxHeader_Init
* Description    : 初始化发送帧头句柄A_master\A_master.axf: Error: L6218E: Undefined symbol moto_chassis (referred from can_user.o).
* Input          : pHeader 发送帧头指针
                   StdId 标识符
                   ExtId 扩展标识符
                   IDE 0:标准帧 1:拓展帧
                   RTR 0:数据帧 1:远程帧
                   DLC 数据长度
* Output         : None
* Return         : None
****************************************************************************** */
void vApp_CAN_TxHeader_Init(CAN_TxHeaderTypeDef    * pHeader,
                                                        uint32_t                             StdId, 
                                                        uint32_t                             ExtId, 
                                                        uint32_t                             IDE, 
                                                        uint32_t                             RTR, 
                                                        uint32_t                             DLC)
{
    pHeader->StdId    = StdId;    //11位     标准标识符
    pHeader->ExtId    = ExtId;    //29位     扩展标识符
    pHeader->IDE        = IDE;        //1位        0:标准帧 1:拓展帧
    pHeader->RTR        = RTR;      //1位   0:数据帧 1:远程帧
    pHeader->DLC        = DLC;        //4位   发送的数据的长度
    pHeader->TransmitGlobalTime    =    ENABLE;
}   
/*******************************************************************************
* Function Name  : vApp_CAN_Filter_Init
* Description    : 初始化滤波器
* Input          : pFilter 滤波器句柄，初始化全部值
                                     IdHigh,
                   IdLow,
                   MaskIdHigh,
                   MaskIdLow,
                   FIFOAssignment,
                   Bank,
                   Mode,
                   Scale,
                   Activation,
                   SlaveStartFilterBank
* Output         : None
* Return         : None
****************************************************************************** */
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
                                                    uint32_t SlaveStartFilterBank)
{
    pFilter->FilterIdHigh                 = IdHigh;
    pFilter->FilterIdLow                  = IdLow;
    pFilter->FilterMaskIdHigh         =    MaskIdHigh;
    pFilter->FilterMaskIdLow             =    MaskIdLow;
    pFilter->FilterFIFOAssignment = CAN_FILTER_FIFO0;
    pFilter->FilterBank                     = Bank;
    pFilter->FilterMode                     = CAN_FILTERMODE_IDMASK;
    pFilter->FilterScale                     = CAN_FILTERSCALE_32BIT;
    pFilter->FilterActivation         = ENABLE;
    pFilter->SlaveStartFilterBank = SlaveStartFilterBank;
}
/*******************************************************************************
* Function Name  : vApp_User_CAN_Configuration
* Description    : 初始化CAN（用户修改）
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void vApp_User_CAN_Configuration(void)
{
	/*----------------- CAN初始化配置 --------------------------*/
	vApp_CAN_Configuration(&hcan1,&hCAN1_TxHeader, &hCAN1_Filter,
												/* TxHeader 句柄配置 */
												/* StdId ExtId IDE RTR DLC */
												0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 0x08,
												/* Filter   句柄配置 */
												/* IdHigh IdLow MaskIdHigh MaskIdLow FIFOAssignment Bank Mode Scale Activation SlaveStartFilterBank */
												0, 0, 0, 0, CAN_FILTER_FIFO0, 0, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, ENABLE, 14);
	vApp_CAN_Configuration(&hcan2,&hCAN2_TxHeader, &hCAN2_Filter,
												/* TxHeader 句柄配置 */
												/* StdId ExtId IDE RTR DLC */
												0x1FF, 0, CAN_ID_STD, CAN_RTR_DATA, 0x08,
												/* Filter   句柄配置 */
												/* IdHigh IdLow MaskIdHigh MaskIdLow FIFOAssignment Bank Mode Scale Activation SlaveStartFilterBank */
												0, 0, 0, 0, CAN_FILTER_FIFO0, 14, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, ENABLE, 27);
}
/*******************************************************************************
* Function Name  : vApp_CAN_Configuration
* Description    : CAN初始化配置，配置发送帧头，配置滤波器
* Input          : (...)
* Output         : None
* Return         : None
****************************************************************************** */
void vApp_CAN_Configuration(CAN_HandleTypeDef *hcan,
                                                        CAN_TxHeaderTypeDef    * pTxHeader,
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
                                                        uint32_t                             SlaveStartFilterBank)
{
    /*-1- 初始化TxHeader句柄 ----------------------------------------*/
    vApp_CAN_TxHeader_Init(pTxHeader, StdId, ExtId, IDE, RTR, DLC);
    
    /*-2- 初始化滤波器句柄 ------------------------------------------*/
    vApp_CAN_Filter_Init(pFilter, IdHigh, IdLow, MaskIdHigh, MaskIdLow, FIFOAssignment, Bank, Mode, Scale, Activation, SlaveStartFilterBank);
    HAL_CAN_ConfigFilter(hcan, pFilter);
    
    /*-3- 启动CAN ---------------------------------------------------*/
    while(HAL_CAN_Start(hcan) != HAL_OK )
    {
			  // printf("\nCAN_Start Failed!!");
        HAL_Delay(100);
    }
  // printf("\nCAN_Start Success!!");
    
    /*-4- 使能中断通知 ----------------------------------------------*/
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}




/*******************************************************************************
* Function Name  : vApp_User_CAN1_TxMessage
* Description    : 使用CAN1发送数据
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */


/*******************************************************************************
* Function Name  : vApp_CAN_TxMessage
* Description    : 邮箱发送数据
* Input          : hcan 
                                     pTxHeader 发送帧头
                                     aData 数据段
                                     DLC 数据段长度
* Output         : None
* Return         : None
****************************************************************************** */
void vApp_CAN_TxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef * pTxHeader, uint8_t aData[], uint8_t DLC)
{
    uint32_t Tx_MailBox;
    /*-1- 配置数据段长度 ----------------------------------------*/
    pTxHeader->DLC    =    DLC;
    /*-2- 发送aData ---------------------------------------------*/
    while(HAL_CAN_AddTxMessage(hcan, pTxHeader, aData, &Tx_MailBox) != HAL_OK)
    {
    // printf("TxMsg Failed!!");
        HAL_Delay(10);
    }
  //printf("\nSend Tx Message Success!!Tx_Mail:%d", Tx_MailBox);

}




/*******************************************************************************
* Function Name  : HAL_CAN_RxFifo0MsgPendingCallback
* Description    : 消息接收回调函数
* Input          : hcan
* Output         : None
* Return         : None         HAL_CAN_RxFifo0MsgPendingCallback(*hcan);
****************************************************************************** */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t aRxData[8], i;
    
     if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hCAN1_RxHeader, aRxData) == HAL_OK)
    {
       // printf("\nGet Rx Message Success!!\nData:");
    
			
    }
		switch(hCAN1_RxHeader.StdId){
		                case CAN_2006Moto1_ID:
		                case CAN_2006Moto2_ID:
		                case CAN_2006Moto3_ID:
		                case CAN_2006Moto4_ID:
			                {
			                  	i = hCAN1_RxHeader.StdId - CAN_2006Moto1_ID;
		                 		  motor_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&motor_chassis[i], aRxData) : get_motor_measure(&motor_chassis[i],aRxData);
												  break;
											}
										default:
												{
														break;
												}						
    }
		
}
/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(motor_measure_t* ptr, uint8_t data[])
{
    ptr->ecd        =(uint16_t)((data)[0] << 8 | (data)[1]);
    ptr->offset_ecd = ptr->ecd;
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2020/01/10
 *******************************************************************************************/
void get_motor_measure(motor_measure_t *ptr, uint8_t data[])
{
//	ptr->last_angle = ptr->angle;
//	ptr->angle = (uint16_t)(data[0]<<8 | data[1]) ;
//	ptr->speed_rpm  = (int16_t)(data[2]<<8 | data[3]);
//	ptr->real_current = (data[4]<<8 | data[5])*5.f/16384.f;
//	ptr->hall =data[6];
//	if(ptr->angle - ptr->last_angle > 4096)
//		ptr->round_cnt --;
//	else if (ptr->angle - ptr->last_angle < -4096)
//		ptr->round_cnt ++;
//	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
	      (ptr)->last_ecd = (ptr)->ecd; 
		    (ptr)->last_current = (ptr)->current; 
		    (ptr)->last_speed_rpm = (ptr)->speed_rpm; 
	      (ptr)->last_total_ecd = (ptr)->total_ecd; 
				
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      
        (ptr)->current = (uint16_t)((data)[4] << 8 | (data)[5]);  
        (ptr)->temperate = (data)[6];           
				if(flag==1){
						flag=0;
					 (ptr)->speed_rpm= ((ptr)->speed_rpm+ (ptr)->last_speed_rpm)/2;
				}else{
						flag=1;
				}
				if(ptr->ecd - ptr->last_ecd > 4096)
					ptr->round_cnt --;
				else if (ptr->ecd - ptr->last_ecd < -4096)
					ptr->round_cnt ++;
				ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;	
				
}



void set_motor_current(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef *pTxHeader,uint16_t iq1, uint16_t iq2, uint16_t iq3, uint16_t iq4){

 uint8_t aData[8]={(iq1 >> 8),iq1,(iq2 >> 8),iq2,(iq3 >> 8),iq3,(iq4 >> 8),iq4}; 
 vApp_CAN_TxMessage(hcan,pTxHeader,aData,8);
// vApp_User_CAN_TxMessage(hcan,aData,8);
}	

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4){
set_motor_current(&hcan1,&hCAN1_TxHeader,motor1,motor2,motor3,motor4);
}
void CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4){
set_motor_current(&hcan2,&hCAN1_TxHeader,motor1,motor2,motor3,motor4);
}

const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[0];
}
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
